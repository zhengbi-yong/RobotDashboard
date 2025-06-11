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

# --- 预定义姿态 ---
JOINT_POSES_DEG = {
    "home":         [1, 1, 1, 1, 1, 1, 1],
    "pre_grasp":    [-30, -39, 0, 0, 0, 0, -21],
    "lift_up":      [0, 0, 0, 0, 0, 0, 0],
}

# --- 手部抓握姿态字典 ---
GRIP_POSES = {
    "open":         [1000, 1000, 1000, 1000, 1000, 1000],
    "small_grasp":  [100, 100, 100, 100, 100, 100],
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

    if action_type == 'move_joint':
        if action_params not in JOINT_POSES_DEG:
             print(f"[ERROR] 未知的关节姿态名称: '{action_params}'。")
             execute_next_action()
             return
        target_angles_deg = JOINT_POSES_DEG[action_params]
        target_angles_rad = [math.radians(deg) for deg in target_angles_deg]
        
        send_moveit_joint_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            joint_names=LEFT_ARM_JOINT_NAMES_INTERNAL,
            target_joint_angles_rad=target_angles_rad,
            start_joint_angles_rad=current_known_angles_rad
        )
    elif action_type == 'set_grip':
        set_hand_grip(action_params)
        ros_client.call_later(3.0, execute_next_action)
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
    print(f"      起始 (rad): {[f'{angle:.3f}' for angle in start_joint_angles_rad]}")
    print(f"      目标 (rad): {[f'{angle:.3f}' for angle in target_joint_angles_rad]}")
    print("---------------------------------")

    joint_constraints = [{'joint_name': name, 'position': pos, 'tolerance_above': 0.01, 'tolerance_below': 0.01, 'weight': 1.0} for name, pos in zip(joint_names, target_joint_angles_rad)]
    goal_constraints = {'name': 'joint_space_goal_constraint', 'joint_constraints': joint_constraints}

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
            'allowed_planning_time': 15.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.5,
            'max_acceleration_scaling_factor': 0.5
        },
        'planning_options': {'plan_only': False, 'replan': False, 'replan_attempts': 3}
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.target_angles = target_joint_angles_rad
    
    # 【已修正的部分】
    # 使用lambda函数来捕获goal对象，并将其与result一起传递给回调
    goal.on('result', lambda result: _on_moveit_result(result, goal))
    
    goal.send()
    print(f"MoveIt! 关节目标已发送 (Goal ID: {goal.goal_id})。正在等待结果...")


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

        initial_pose_deg = JOINT_POSES_DEG['home']
        current_known_angles_rad = [math.radians(deg) for deg in initial_pose_deg]

        action_queue = [
            ('move_joint', 'home'),
            ('set_grip', 'open'),
            ('wait', 1.0),
            ('move_joint', 'pre_grasp'),
            ('set_grip', 'small_grasp'),
            ('wait', 2.0),
            ('move_joint', 'lift_up'),
            ('wait', 2.0),
            ('move_joint', 'home'),
        ]
        
        execute_next_action()

    except Exception as e:
        print(f"在设置阶段发生错误: {e}")
        reactor.stop()

def main():
    """主执行函数，设置ROS客户端并启动事件循环"""
    global ros_client
    print("--- 机器人关节与抓握协同控制脚本 ---")
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_client.on_ready(lambda: main_logic_on_ready(ros_client))
    
    print(f"正在尝试连接到 {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    
    try:
        ros_client.run_forever()
    except KeyboardInterrupt:
        print("检测到键盘中断。正在关闭。")
    finally:
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        print("脚本执行完毕。")

if __name__ == '__main__':
    main()