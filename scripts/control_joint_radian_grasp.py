#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib
import time
import math
# 导入 Twisted reactor 以正确处理关机
from twisted.internet import reactor

# --- 1. Configuration ---
ROS_BRIDGE_HOST = '192.168.0.105'  # 重要：请确保这是您的机器人IP地址
ROS_BRIDGE_PORT = 9090

# --- MoveIt! 配置 ---
MOVE_GROUP_ACTION_NAME = '/move_group'
MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
PLANNING_GROUP_LEFT_ARM = 'l_arm'
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_joint{i+1}' for i in range(7)]

# --- 手部控制配置 ---
LEFT_HAND_CMD_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
HAND_MSG_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 预定义姿态（弧度值）---
JOINT_POSES_RAD = {
    "neutral":      [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home":         [0.0175, 0.0175, 0.0175, 0.0175, 0.0175, 0.0175, 0.0175],  # 约1度
    "pre_grasp":    [-0.6256, -0.2060, -0.9691, -1.0310, 0.3118, -0.9777, 0.6438],  # 转换为弧度
    "ready_pose":   [0.0, -0.5236, 0.0, -1.5708, 0.0, 1.0472, 0.0],  # 准备姿态
    "reach_forward":[0.0, 0.5236, 0.0, 1.0472, 0.0, -0.5236, 0.0],   # 前伸姿态
    "side_reach":   [1.5708, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0],      # 侧向抓取
    "lift_up":      [0.0, -1.0472, 0.0, -0.5236, 0.0, 1.5708, 0.0],  # 举起姿态
    "inspection":   [0.0, 0.0, 0.0, -2.0944, 0.0, 2.0944, 0.0],      # 检查姿态
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
moveit_action_client = None
left_hand_pub = None
action_queue = []
current_known_angles_rad = []

def execute_next_action():
    """从动作队列中取出一个动作并执行。"""
    global action_queue, current_known_angles_rad
    if not action_queue:
        print("\n[INFO] 动作序列已全部执行完毕。正在关闭脚本。")
        ros_client.call_later(1.0, lambda: reactor.callFromThread(reactor.stop))
        return

    action_type, action_params = action_queue.pop(0)
    print(f"\n---> 正在执行动作: {action_type}, 参数: {action_params}")

    if action_type == 'move_joint_rad':
        if action_params not in JOINT_POSES_RAD:
             print(f"[ERROR] 未知的关节姿态名称: '{action_params}'。")
             execute_next_action()
             return
        target_angles_rad = JOINT_POSES_RAD[action_params]
        
        send_moveit_joint_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            joint_names=LEFT_ARM_JOINT_NAMES_INTERNAL,
            target_joint_angles_rad=target_angles_rad,
            start_joint_angles_rad=current_known_angles_rad
        )
    elif action_type == 'move_joint_custom':
        # 支持自定义弧度值列表
        if not isinstance(action_params, list) or len(action_params) != 7:
            print(f"[ERROR] 自定义关节角必须是7个弧度值的列表。")
            execute_next_action()
            return
        
        send_moveit_joint_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            joint_names=LEFT_ARM_JOINT_NAMES_INTERNAL,
            target_joint_angles_rad=action_params,
            start_joint_angles_rad=current_known_angles_rad
        )
    elif action_type == 'set_grip':
        set_hand_grip(action_params)
        ros_client.call_later(2.0, execute_next_action)
    elif action_type == 'wait':
        print(f"      等待 {action_params} 秒...")
        ros_client.call_later(action_params, execute_next_action)
    else:
        print(f"[WARNING] 未知的动作类型: {action_type}。跳过此动作。")
        execute_next_action()


def set_hand_grip(grip_name):
    """发布一个手部抓握指令。"""
    if not left_hand_pub:
        print("[ERROR] 手部指令发布器 (left_hand_pub) 未初始化。")
        return
    if grip_name not in GRIP_POSES:
        print(f"[ERROR] 未知的抓握名称: '{grip_name}'。")
        return
    grip_values = GRIP_POSES[grip_name]
    hand_msg = roslibpy.Message({'hand_angle': grip_values})
    print(f"      正在设置手部姿态为 '{grip_name}' (数值: {grip_values})")
    left_hand_pub.publish(hand_msg)


def _on_moveit_result(result, goal):
    """处理MoveIt!动作结果的回调函数。"""
    global current_known_angles_rad
    error_code = result['error_code']['val']
    
    if error_code == 1:
        print("\n[SUCCESS] MoveIt! 目标已成功完成。")
        current_known_angles_rad = goal.target_angles 
        print(f"      当前关节角 (弧度): {[f'{angle:.4f}' for angle in current_known_angles_rad]}")
        print(f"      当前关节角 (度数): {[f'{math.degrees(angle):.2f}' for angle in current_known_angles_rad]}")
        execute_next_action()
    else:
        print(f"\n[FAILURE] MoveIt! 目标失败，错误代码: {error_code}。")
        print("[WARNING] 移动失败，但将按要求继续执行下一个动作。")
        execute_next_action()


def send_moveit_joint_goal(planning_group_name, joint_names, target_joint_angles_rad, start_joint_angles_rad):
    """构建并发送一个关节空间目标至MoveIt!，并明确指定起始状态。"""
    if not moveit_action_client or not ros_client.is_connected:
        print("[ERROR] MoveIt! Action Client 不可用或ROS未连接。")
        return

    print(f"--- 正在发送MoveIt! 关节目标 ---")
    print(f"      规划组: {planning_group_name}")
    print(f"      起始 (弧度): {[f'{angle:.4f}' for angle in start_joint_angles_rad]}")
    print(f"      起始 (度数): {[f'{math.degrees(angle):.2f}' for angle in start_joint_angles_rad]}")
    print(f"      目标 (弧度): {[f'{angle:.4f}' for angle in target_joint_angles_rad]}")
    print(f"      目标 (度数): {[f'{math.degrees(angle):.2f}' for angle in target_joint_angles_rad]}")
    print("-" * 50)

    # 验证关节角范围（通常为 -π 到 π）
    for i, angle in enumerate(target_joint_angles_rad):
        if abs(angle) > math.pi:
            print(f"[WARNING] 关节 {i+1} 角度 {angle:.4f} 弧度超出正常范围 [-π, π]")

    joint_constraints = [
        {
            'joint_name': name, 
            'position': pos, 
            'tolerance_above': 0.01, 
            'tolerance_below': 0.01, 
            'weight': 1.0
        } 
        for name, pos in zip(joint_names, target_joint_angles_rad)
    ]
    
    goal_constraints = {
        'name': 'joint_space_goal_constraint', 
        'joint_constraints': joint_constraints
    }

    start_state = {
        'joint_state': {
            'name': joint_names,
            'position': start_joint_angles_rad
        }
    }

    move_group_goal_message = {
        'request': {
            'group_name': planning_group_name,
            'goal_constraints': [goal_constraints],
            'start_state': start_state,
            'num_planning_attempts': 5,
            'allowed_planning_time': 10.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.4,  # 稍微降低速度以提高精度
            'max_acceleration_scaling_factor': 0.4
        },
        'planning_options': {'plan_only': False, 'replan': True, 'replan_attempts': 3}
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.target_angles = target_joint_angles_rad
    
    goal.on('result', lambda result: _on_moveit_result(result, goal))
    
    goal.send()
    print(f"MoveIt! 关节目标已发送 (Goal ID: {goal.goal_id})。正在等待结果...")


def print_joint_angles_conversion():
    """打印所有预定义姿态的弧度和度数对照表"""
    print("\n=== 关节姿态弧度-度数对照表 ===")
    for pose_name, angles_rad in JOINT_POSES_RAD.items():
        angles_deg = [math.degrees(rad) for rad in angles_rad]
        print(f"{pose_name:12} (弧度): {[f'{angle:7.4f}' for angle in angles_rad]}")
        print(f"{pose_name:12} (度数): {[f'{angle:7.2f}' for angle in angles_deg]}")
        print()


def main_logic_on_ready(client):
    """连接成功后执行的主要逻辑函数"""
    global moveit_action_client, left_hand_pub, action_queue, current_known_angles_rad
    print("与ROS Bridge连接成功。")
    
    try:
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        print("MoveIt! Action Client 已初始化。")

        left_hand_pub = roslibpy.Topic(client, LEFT_HAND_CMD_TOPIC, HAND_MSG_TYPE)
        print(f"手部指令发布器已初始化 (Topic: {LEFT_HAND_CMD_TOPIC})。")

        # 打印关节角对照表
        print_joint_angles_conversion()

        # 初始化当前已知角度为中性位置
        initial_pose_rad = JOINT_POSES_RAD['neutral']
        current_known_angles_rad = initial_pose_rad.copy()

        # 定义执行序列（使用弧度控制）
        action_queue = [
            ('move_joint_rad', 'neutral'),
            ('set_grip', 'open'),
            ('wait', 1.5),
            ('move_joint_rad', 'pre_grasp'),
            ('wait', 1.0),
            # ('move_joint_rad', 'reach_forward'),
            ('set_grip', 'medium_grasp'),
            # ('wait', 1.0),
            # ('move_joint_rad', 'lift_up'),
            # ('set_grip', 'firm_grasp'),
            # ('wait', 1.5),
            # ('move_joint_rad', 'inspection'),
            # ('wait', 2.0),
            # ('move_joint_rad', 'side_reach'),
            # ('wait', 1.0),
            # # 演示自定义弧度值控制
            # ('move_joint_custom', [0.5, -0.3, 0.2, -1.2, 0.1, 0.8, -0.1]),
            # ('wait', 1.0),
            # ('set_grip', 'open'),
            # ('move_joint_rad', 'home'),
            # ('wait', 1.0),
            ('move_joint_rad', 'neutral'),
        ]
        
        print(f"\n准备执行 {len(action_queue)} 个动作...")
        execute_next_action()

    except Exception as e:
        print(f"在设置阶段发生错误: {e}")
        reactor.stop()


def main():
    """主执行函数，设置ROS客户端并启动事件循环"""
    global ros_client
    print("=" * 60)
    print("--- 机器人关节弧度值控制脚本 ---")
    print("本脚本使用弧度值精确控制机械臂关节角度")
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
    finally:
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        print("脚本执行完毕。")


if __name__ == '__main__':
    main()