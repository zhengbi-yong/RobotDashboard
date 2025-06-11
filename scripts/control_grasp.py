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
PLANNING_FRAME = "l_base_link"         # 机器人规划的参考坐标系
END_EFFECTOR_LINK = "l_hand_base_link" # 要控制的左臂末端连杆名称

# --- 新增：手部控制配置 ---
LEFT_HAND_CMD_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
HAND_MSG_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 新增：定义抓握姿态 ---
GRIP_POSES = {
    "open":         [1000, 1000, 1000, 1000, 1000, 1000], # 完全张开
    "big_grasp":    [700, 700, 700, 700, 700, 700],      # 大握 (抓取大物体)
    "medium_grasp": [400, 400, 400, 400, 400, 400],      # 中握 (通用抓取)
    "small_grasp":  [100, 100, 100, 100, 100, 100],      # 小握 (抓取小物体)
    "closed":       [0, 0, 0, 0, 0, 0],                  # 完全握拳
}

# --- 2. Global Variables ---
ros_client = None
moveit_action_client = None
left_hand_pub = None
action_queue = []

def execute_next_action():
    """
    从动作队列中取出一个动作并执行。
    这是实现顺序控制的核心函数。
    """
    global action_queue
    if not action_queue:
        print("\n[INFO] 动作序列已全部执行完毕。正在关闭脚本。")
        # --- FIX: 使用 lambda 封装 reactor 的关闭调用 ---
        # 延迟一秒关闭，确保最后的消息有时间发出
        ros_client.call_later(1.0, lambda: reactor.callFromThread(reactor.stop))
        return

    action_type, action_params = action_queue.pop(0)
    print(f"\n---> 正在执行动作: {action_type}, 参数: {action_params}")

    if action_type == 'move_pose':
        send_moveit_pose_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            target_pose=action_params,
            end_effector_link=END_EFFECTOR_LINK
        )
    elif action_type == 'set_grip':
        set_hand_grip(action_params)
        # 发布手部指令后，等待一小段时间让物理动作完成，再执行下一步
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


def _on_moveit_result(result):
    """处理MoveIt!动作结果的回调函数。"""
    error_code = result['error_code']['val']
    if error_code == 1:  # SUCCESS
        print("\n[SUCCESS] MoveIt! 目标已成功完成。")
        execute_next_action()
    else:
        print(f"\n[FAILURE] MoveIt! 目标失败，错误代码: {error_code}。")
        print("[ERROR] 由于移动失败，动作序列已终止。")
        reactor.callFromThread(reactor.stop)


def send_moveit_pose_goal(planning_group_name, target_pose, end_effector_link):
    """构建并发送一个末端位姿空间目标至MoveIt!"""
    if not moveit_action_client or not ros_client.is_connected:
        print("[ERROR] MoveIt! Action Client 不可用或ROS未连接。")
        return

    print(f"--- 正在发送MoveIt! 位姿目标 ---")
    print(f"      规划组: {planning_group_name}")
    print(f"      目标位置 (x,y,z): {target_pose['position']}")
    print(f"      目标姿态 (w,x,y,z): {target_pose['orientation']}")
    print("---------------------------------")

    move_group_goal_message = {
        'request': {
            'group_name': planning_group_name,
            'num_planning_attempts': 10,
            'allowed_planning_time': 5.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.4,
            'max_acceleration_scaling_factor': 0.4,
            'goal_constraints': [{
                'name': 'pose_goal_constraint',
                'position_constraints': [{
                    'header': {'frame_id': PLANNING_FRAME},
                    'link_name': end_effector_link,
                    'constraint_region': {
                        'primitive_poses': [{'position': target_pose['position'], 'orientation': {'w': 1.0}}],
                        'primitives': [{'type': 1, 'dimensions': [0.01]}]
                    },
                    'weight': 1.0
                }],
                'orientation_constraints': [{
                    'header': {'frame_id': PLANNING_FRAME},
                    'link_name': end_effector_link,
                    'orientation': target_pose['orientation'],
                    'absolute_x_axis_tolerance': 0.1,
                    'absolute_y_axis_tolerance': 0.1,
                    'absolute_z_axis_tolerance': 0.1,
                    'weight': 1.0
                }],
            }]
        },
        'planning_options': {'plan_only': False, 'replan': True, 'replan_attempts': 5}
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.on('result', _on_moveit_result)
    goal.send()
    print(f"MoveIt! 位姿目标已发送 (Goal ID: {goal.goal_id})。正在等待结果...")


def main_logic_on_ready(client):
    """连接成功后执行的主要逻辑函数"""
    global moveit_action_client, left_hand_pub, action_queue
    print("与ROS Bridge连接成功。")
    
    try:
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        print("MoveIt! Action Client 已初始化。")

        left_hand_pub = roslibpy.Topic(client, LEFT_HAND_CMD_TOPIC, HAND_MSG_TYPE)
        print(f"手部指令发布器已初始化 (Topic: {LEFT_HAND_CMD_TOPIC})。")

        target_pose = {
            'position': {'x': 0.0, 'y': 0.3, 'z': 0.4},
            'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        
        action_queue = [
            ('move_pose', target_pose),
            ('wait', 1.0),
            ('set_grip', 'medium_grasp'),
            ('wait', 2.0),
            ('set_grip', 'open'),
            ('wait', 2.0),
        ]
        
        execute_next_action()

    except Exception as e:
        print(f"在设置阶段发生错误: {e}")
        reactor.stop()

def main():
    """主执行函数，设置ROS客户端并启动事件循环"""
    global ros_client
    print("--- 机器人位姿与抓握协同控制脚本 ---")
    
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