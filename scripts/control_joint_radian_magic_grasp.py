#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib
import time
import math
import numpy as np
import threading
# 导入 Twisted reactor 以正确处理关机
from twisted.internet import reactor

# --- 1. Configuration ---
ROS_BRIDGE_HOST = '192.168.0.105'  # 重要：请确保这是您的机器人IP地址
ROS_BRIDGE_PORT = 9090

# --- 最小急动控制器配置 ---
LEFT_ACTION_NAME = '/left_move_min_jerk'
RIGHT_ACTION_NAME = '/right_move_min_jerk'
ACTION_TYPE = 'magic_dual_arm_control/MoveMinJerkAction'
ERROR_THRESHOLD = 0.02  # 误差阈值（弧度）
DURATION = 2.0  # 运动持续时间（秒）
DT = 0.005  # 控制周期（秒）
FOLLOW = True  # 高跟随模式
MAX_WAIT_TIME = 10  # 最大等待时间（秒）

# --- 手部控制配置 ---
LEFT_HAND_CMD_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
RIGHT_HAND_CMD_TOPIC = '/r_arm/rm_driver/Hand_SetAngle'
HAND_MSG_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 预定义姿态（弧度值）---
JOINT_POSES_RAD = {
    "neutral":      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home":         [0.0175, 0.0175, 0.0175, 0.0175, 0.0175, 0.0175, 0.0175],  # 约1度
    "pre_grasp_l":  [-0.6256, -0.2060, -0.9691, -1.0310, 0.3118, -0.9777, 0.6438],  # 左臂预抓取
    "pre_grasp_r":  [0.7679, -1.1868, 0.0, 0.0, 0.0, 0.0, -0.2094],   # 右臂预抓取
    "ready_pose_l": [0.0, -0.5236, 0.0, -1.5708, 0.0, 1.0472, 0.0],   # 左臂准备姿态
    "ready_pose_r": [0.0, -0.5236, 0.0, -1.5708, 0.0, 1.0472, 0.0],   # 右臂准备姿态
    "reach_forward_l": [0.0, 0.5236, 0.0, 1.0472, 0.0, -0.5236, 0.0], # 左臂前伸
    "reach_forward_r": [0.0, 0.5236, 0.0, 1.0472, 0.0, -0.5236, 0.0], # 右臂前伸
    "side_reach_l": [1.5708, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0],       # 左臂侧向
    "side_reach_r": [-1.5708, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0],     # 右臂侧向
    "lift_up_l":    [0.0, -1.0472, 0.0, -0.5236, 0.0, 1.5708, 0.0],  # 左臂举起
    "lift_up_r":    [0.0, -1.0472, 0.0, -0.5236, 0.0, 1.5708, 0.0],  # 右臂举起
}

# --- 手部抓握姿态字典 ---
GRIP_POSES = {
    "open":         [1000, 1000, 1000, 1000, 1000, 1000],
    "light_grasp":  [800, 800, 800, 800, 800, 800],
    "medium_grasp": [500, 500, 500, 500, 500, 500],
    "firm_grasp":   [200, 200, 200, 200, 200, 200],
    "tight_grasp":  [100, 100, 100, 100, 100, 100],
    "closed":       [0, 0, 0, 0, 0, 0],
}

# --- 2. Global Variables ---
ros_client = None
left_action_client = None
right_action_client = None
left_hand_pub = None
right_hand_pub = None
action_queue = []
current_left_angles_rad = []
current_right_angles_rad = []

# 全局变量用于监控关节状态
left_joint_pos = None
right_joint_pos = None

def execute_next_action():
    """从动作队列中取出一个动作并执行。"""
    global action_queue, current_left_angles_rad, current_right_angles_rad
    if not action_queue:
        print("\n[INFO] 动作序列已全部执行完毕。正在关闭脚本。")
        ros_client.call_later(1.0, lambda: reactor.callFromThread(reactor.stop))
        return

    action_type, action_params = action_queue.pop(0)
    print(f"\n---> 正在执行动作: {action_type}, 参数: {action_params}")

    if action_type == 'move_joint_rad':
        arm, pose_name = action_params
        if pose_name not in JOINT_POSES_RAD:
            print(f"[ERROR] 未知的关节姿态名称: '{pose_name}'。")
            execute_next_action()
            return
        
        target_angles_rad = JOINT_POSES_RAD[pose_name]
        send_min_jerk_goal(arm, target_angles_rad)
        
    elif action_type == 'move_joint_custom':
        arm, target_angles = action_params
        if not isinstance(target_angles, list) or len(target_angles) != 7:
            print(f"[ERROR] 自定义关节角必须是7个弧度值的列表。")
            execute_next_action()
            return
        
        send_min_jerk_goal(arm, target_angles)
        
    elif action_type == 'move_both_arms':
        left_pose, right_pose = action_params
        if left_pose not in JOINT_POSES_RAD or right_pose not in JOINT_POSES_RAD:
            print(f"[ERROR] 未知的关节姿态名称。")
            execute_next_action()
            return
            
        left_target = JOINT_POSES_RAD[left_pose]
        right_target = JOINT_POSES_RAD[right_pose]
        send_dual_arm_goals(left_target, right_target)
        
    elif action_type == 'set_grip':
        arm, grip_name = action_params
        set_hand_grip(arm, grip_name)
        ros_client.call_later(2.0, execute_next_action)
        
    elif action_type == 'wait':
        print(f"      等待 {action_params} 秒...")
        ros_client.call_later(action_params, execute_next_action)
        
    else:
        print(f"[WARNING] 未知的动作类型: {action_type}。跳过此动作。")
        execute_next_action()


def get_joint_state_once(topic_name, arm_name):
    """获取一次关节状态"""
    global left_joint_pos, right_joint_pos
    joint_pos = None
    subscriber = roslibpy.Topic(ros_client, topic_name, 'sensor_msgs/JointState')
    
    def on_joint_state(msg):
        nonlocal joint_pos
        joint_pos = msg['position']
        subscriber.unsubscribe()

    subscriber.subscribe(on_joint_state)
    start_time = time.time()
    while joint_pos is None:
        time.sleep(0.1)
        if time.time() - start_time > 3.0:
            subscriber.unsubscribe()
            raise TimeoutError(f"无法从 {topic_name} 获取 {arm_name} 臂的关节状态")
    
    if arm_name == 'left':
        left_joint_pos = joint_pos
    else:
        right_joint_pos = joint_pos
    return joint_pos


def monitor_joint_state(topic_name, target_pos, timeout, arm_name, result_dict):
    """监控关节状态并检查误差"""
    joint_pos = None
    error_norm = float('inf')
    subscriber = roslibpy.Topic(ros_client, topic_name, 'sensor_msgs/JointState')
    
    def on_joint_state(msg):
        nonlocal joint_pos, error_norm
        joint_pos = msg['position']
        if (isinstance(joint_pos, list) and len(joint_pos) == 7 and
                all(isinstance(x, (int, float)) for x in joint_pos)):
            error = np.array(target_pos) - np.array(joint_pos)
            error_norm = np.linalg.norm(error)
    
    subscriber.subscribe(on_joint_state)
    start_time = time.time()
    while error_norm > ERROR_THRESHOLD:
        time.sleep(0.01)
        if time.time() - start_time > timeout:
            subscriber.unsubscribe()
            result_dict[arm_name] = (None, None, f"目标在 {timeout} 秒内未到达")
            return
        if joint_pos is None:
            continue
    
    subscriber.unsubscribe()
    result_dict[arm_name] = (joint_pos, error_norm, None)


def send_min_jerk_goal(arm, target_angles_rad):
    """发送单臂最小急动目标"""
    global current_left_angles_rad, current_right_angles_rad
    
    if arm == 'left':
        action_client = left_action_client
        current_pos = current_left_angles_rad
        topic_name = '/get_left_arm_degree'
    elif arm == 'right':
        action_client = right_action_client
        current_pos = current_right_angles_rad
        topic_name = '/get_right_arm_degree'
    else:
        print(f"[ERROR] 未知的机械臂: {arm}")
        execute_next_action()
        return

    if not action_client:
        print(f"[ERROR] {arm} 臂的Action Client未初始化。")
        execute_next_action()
        return

    print(f"--- 正在发送 {arm} 臂最小急动目标 ---")
    print(f"      起始 (弧度): {[f'{angle:.4f}' for angle in current_pos]}")
    print(f"      起始 (度数): {[f'{math.degrees(angle):.2f}' for angle in current_pos]}")
    print(f"      目标 (弧度): {[f'{angle:.4f}' for angle in target_angles_rad]}")
    print(f"      目标 (度数): {[f'{math.degrees(angle):.2f}' for angle in target_angles_rad]}")
    print("-" * 50)

    goal_message = {
        'start_pos': current_pos,
        'end_pos': target_angles_rad,
        'duration': DURATION,
        'dt': DT,
        'follow': FOLLOW
    }

    goal = roslibpy.actionlib.Goal(action_client, goal_message)
    goal.send()
    print(f"{arm} 臂目标已发送，正在监控执行...")

    # 创建监控线程
    result_dict = {}
    monitor_thread = threading.Thread(
        target=monitor_joint_state, 
        args=(topic_name, target_angles_rad, MAX_WAIT_TIME, arm, result_dict)
    )
    monitor_thread.start()
    monitor_thread.join()

    # 处理结果
    if arm in result_dict:
        joint_pos, error_norm, error_msg = result_dict[arm]
        if error_msg:
            print(f"[ERROR] {arm} 臂执行失败: {error_msg}")
        else:
            print(f"[SUCCESS] {arm} 臂目标完成！")
            print(f"      最终位置 (弧度): {[f'{angle:.4f}' for angle in joint_pos]}")
            print(f"      误差范数: {error_norm:.6f} 弧度")
            
            # 更新当前角度
            if arm == 'left':
                current_left_angles_rad = joint_pos
            else:
                current_right_angles_rad = joint_pos
    else:
        print(f"[ERROR] 未收到 {arm} 臂的执行结果")

    execute_next_action()


def send_dual_arm_goals(left_target, right_target):
    """同时发送双臂目标"""
    global current_left_angles_rad, current_right_angles_rad
    
    print("--- 正在发送双臂协调目标 ---")
    print(f"      左臂目标 (弧度): {[f'{angle:.4f}' for angle in left_target]}")
    print(f"      右臂目标 (弧度): {[f'{angle:.4f}' for angle in right_target]}")
    print("-" * 50)

    # 准备目标消息
    left_goal_msg = {
        'start_pos': current_left_angles_rad,
        'end_pos': left_target,
        'duration': DURATION,
        'dt': DT,
        'follow': FOLLOW
    }
    
    right_goal_msg = {
        'start_pos': current_right_angles_rad,
        'end_pos': right_target,
        'duration': DURATION,
        'dt': DT,
        'follow': FOLLOW
    }

    # 发送目标
    left_goal = roslibpy.actionlib.Goal(left_action_client, left_goal_msg)
    right_goal = roslibpy.actionlib.Goal(right_action_client, right_goal_msg)
    
    left_goal.send()
    right_goal.send()
    print("双臂目标已发送，正在监控执行...")

    # 创建监控线程
    result_dict = {}
    monitor_threads = []
    
    left_thread = threading.Thread(
        target=monitor_joint_state, 
        args=('/get_left_arm_degree', left_target, MAX_WAIT_TIME, 'left', result_dict)
    )
    right_thread = threading.Thread(
        target=monitor_joint_state, 
        args=('/get_right_arm_degree', right_target, MAX_WAIT_TIME, 'right', result_dict)
    )
    
    monitor_threads.extend([left_thread, right_thread])
    
    for thread in monitor_threads:
        thread.start()
    
    for thread in monitor_threads:
        thread.join()

    # 处理结果
    for arm_name in ['left', 'right']:
        if arm_name in result_dict:
            joint_pos, error_norm, error_msg = result_dict[arm_name]
            if error_msg:
                print(f"[ERROR] {arm_name} 臂执行失败: {error_msg}")
            else:
                print(f"[SUCCESS] {arm_name} 臂目标完成！")
                print(f"      最终位置: {[f'{angle:.4f}' for angle in joint_pos]}")
                print(f"      误差范数: {error_norm:.6f} 弧度")
                
                # 更新当前角度
                if arm_name == 'left':
                    current_left_angles_rad = joint_pos
                else:
                    current_right_angles_rad = joint_pos
        else:
            print(f"[ERROR] 未收到 {arm_name} 臂的执行结果")

    execute_next_action()


def set_hand_grip(arm, grip_name):
    """发布手部抓握指令"""
    if arm == 'left':
        hand_pub = left_hand_pub
    elif arm == 'right':
        hand_pub = right_hand_pub
    else:
        print(f"[ERROR] 未知的机械臂: {arm}")
        return
        
    if not hand_pub:
        print(f"[ERROR] {arm} 臂手部指令发布器未初始化。")
        return
        
    if grip_name not in GRIP_POSES:
        print(f"[ERROR] 未知的抓握名称: '{grip_name}'。")
        return
        
    grip_values = GRIP_POSES[grip_name]
    hand_msg = roslibpy.Message({'hand_angle': grip_values})
    print(f"      正在设置 {arm} 臂手部姿态为 '{grip_name}' (数值: {grip_values})")
    hand_pub.publish(hand_msg)


def print_joint_angles_conversion():
    """打印所有预定义姿态的弧度和度数对照表"""
    print("\n=== 关节姿态弧度-度数对照表 ===")
    for pose_name, angles_rad in JOINT_POSES_RAD.items():
        angles_deg = [math.degrees(rad) for rad in angles_rad]
        print(f"{pose_name:15} (弧度): {[f'{angle:7.4f}' for angle in angles_rad]}")
        print(f"{pose_name:15} (度数): {[f'{angle:7.2f}' for angle in angles_deg]}")
        print()


def main_logic_on_ready(client):
    """连接成功后执行的主要逻辑函数"""
    global left_action_client, right_action_client, left_hand_pub, right_hand_pub
    global action_queue, current_left_angles_rad, current_right_angles_rad
    print("与ROS Bridge连接成功。")
    
    try:
        # 初始化Action Clients
        left_action_client = roslibpy.actionlib.ActionClient(
            client, LEFT_ACTION_NAME, ACTION_TYPE
        )
        right_action_client = roslibpy.actionlib.ActionClient(
            client, RIGHT_ACTION_NAME, ACTION_TYPE
        )
        print("双臂最小急动 Action Clients 已初始化。")

        # 初始化手部发布器
        left_hand_pub = roslibpy.Topic(client, LEFT_HAND_CMD_TOPIC, HAND_MSG_TYPE)
        right_hand_pub = roslibpy.Topic(client, RIGHT_HAND_CMD_TOPIC, HAND_MSG_TYPE)
        print("双臂手部指令发布器已初始化。")

        # 打印关节角对照表
        print_joint_angles_conversion()

        # 获取初始关节状态
        print("正在获取初始关节状态...")
        left_joint_pos = get_joint_state_once('/get_left_arm_degree', 'left')
        right_joint_pos = get_joint_state_once('/get_right_arm_degree', 'right')
        
        current_left_angles_rad = left_joint_pos
        current_right_angles_rad = right_joint_pos
        
        print(f"左臂初始位置 (弧度): {[f'{angle:.4f}' for angle in current_left_angles_rad]}")
        print(f"右臂初始位置 (弧度): {[f'{angle:.4f}' for angle in current_right_angles_rad]}")

        # 定义执行序列
        action_queue = [
            # 双臂回到中性位置
            ('move_both_arms', ('neutral', 'neutral')),
            ('wait', 1.0),
            
            # 打开双手
            ('set_grip', ('left', 'open')),
            ('set_grip', ('right', 'open')),
            ('wait', 1.0),

            ('move_both_arms', ('pre_grasp_l', 'neutral')),
            # ('set_grip', ('left', 'firm_grasp')),
            ('wait', 5.5),
            ('set_grip', ('left', 'firm_grasp')),
            ('move_both_arms', ('neutral', 'neutral')),
            # # 双臂移动到准备姿态
            # ('move_both_arms', ('ready_pose_l', 'ready_pose_r')),
            # ('wait', 1.5),
            
            # # 双臂前伸
            # ('move_both_arms', ('reach_forward_l', 'reach_forward_r')),
            # ('wait', 1.0),
            
            # # 轻抓握
            # ('set_grip', ('left', 'light_grasp')),
            # ('set_grip', ('right', 'light_grasp')),
            # ('wait', 1.0),
            
            # # 举起
            # ('move_both_arms', ('lift_up_l', 'lift_up_r')),
            # ('wait', 1.5),
            
            # # 牢固抓握
            # ('set_grip', ('left', 'firm_grasp')),
            # ('set_grip', ('right', 'firm_grasp')),
            # ('wait', 1.0),
            
            # # 侧向展示
            # ('move_both_arms', ('side_reach_l', 'side_reach_r')),
            # ('wait', 2.0),
            
            # # 演示自定义角度控制（左臂）
            # ('move_joint_custom', ('left', [0.5, -0.3, 0.2, -1.2, 0.1, 0.8, -0.1])),
            # ('wait', 1.0),
            
            # # 演示单臂控制（右臂）
            # ('move_joint_rad', ('right', 'pre_grasp_r')),
            # ('wait', 1.0),
            
            # # 松开抓握
            # ('set_grip', ('left', 'open')),
            # ('set_grip', ('right', 'open')),
            # ('wait', 1.0),
            
            # # 回到起始位置
            # ('move_both_arms', ('home', 'home')),
            # ('wait', 1.0),
            
            # # 最终回到中性位置
            # ('move_both_arms', ('neutral', 'neutral')),
        ]
        
        print(f"\n准备执行 {len(action_queue)} 个动作...")
        execute_next_action()

    except Exception as e:
        print(f"在设置阶段发生错误: {e}")
        import traceback
        traceback.print_exc()
        reactor.stop()


def main():
    """主执行函数，设置ROS客户端并启动事件循环"""
    global ros_client
    print("=" * 60)
    print("--- 双臂机器人弧度值最小急动控制脚本 ---")
    print("本脚本使用最小急动控制器精确控制双臂关节角度")
    print("=" * 60)
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_client.on_ready(lambda: main_logic_on_ready(ros_client))
    
    print(f"正在尝试连接到 ROS Bridge: {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    
    try:
        ros_client.run_forever()
    except KeyboardInterrupt:
        print("\n检测到键盘中断。正在关闭...")
    except Exception as e:
        print(f"运行时错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        print("脚本执行完毕。")


if __name__ == '__main__':
    main()