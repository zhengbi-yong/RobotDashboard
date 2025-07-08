#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import roslibpy.actionlib
from twisted.internet import reactor
import time

# --- 全局变量 ---
ros_client = None
action_client = None
current_waypoint_index = 0 # 用于追踪当前路径点的索引

# --- 定义一个连续的关节空间轨迹 ---
# 这是一个包含多个路径点(waypoints)的列表
# 每个路径点都是一个关节角度的列表
# 请根据你的机器人和需求调整这些值
JOINT_WAYPOINTS = [
    [0.2, 0.1, 0.0, 0.5, 0.0, 0.0, 0.7],  # 第 1 个点
    [0.5, 0.3, 0.0, 1.0, 0.0, 0.0, 0.7],  # 第 2 个点
    [0.2, 0.5, 0.0, 1.5, 0.0, 0.0, 0.7],  # 第 3 个点
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 回到初始位置
]

def send_moveit_goal():
    """
    通过 MoveIt! 的 MoveGroup Action Server 发送一个规划和执行的目标。
    """
    global current_waypoint_index

    if not action_client or not ros_client.is_connected:
        print("错误：Action Client 不可用或 ROS 未连接。")
        reactor.callFromThread(reactor.stop)
        return
    
    # 检查是否所有路径点都已执行完毕
    if current_waypoint_index >= len(JOINT_WAYPOINTS):
        print("\n========================================")
        print("所有路径点均已成功执行！")
        print("========================================")
        print("\n测试完成，正在关闭连接。")
        reactor.callFromThread(reactor.stop)
        return

    print(f"\n---> 正在发送路径点 #{current_waypoint_index + 1}...")

    # 从列表中获取当前的目标关节位置
    joint_positions = JOINT_WAYPOINTS[current_waypoint_index]

    # IMPORTANT: 关节名称需要和你的机器人 URDF/SRDF 文件中的完全一致
    # 你可以通过 `rostopic echo /joint_states` 查看关节名称
    joint_names = ['r_joint1', 'r_joint2', 'r_joint3', 'r_joint4', 'r_joint5', 'r_joint6', 'r_joint7']

    # 创建 MoveIt! 的 Goal 消息
    goal_msg = {
        'request': {
            'group_name': 'r_arm',  # 确保这个规划组在你的 MoveIt! 配置中存在
            'goal_constraints': [{
                'name': 'joint_constraint',
                'joint_constraints': [
                    {'joint_name': name, 'position': pos, 'weight': 1.0}
                    for name, pos in zip(joint_names, joint_positions)
                ]
            }],
            'allowed_planning_time': 10.0, # 允许的规划时间
            'num_planning_attempts': 1 # 规划尝试次数
        }
    }

    goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message(goal_msg))

    def on_result(result):
        global current_waypoint_index
        # MoveIt! 的返回结果中有一个 error_code 字段
        if result['error_code']['val'] == 1: # 1 表示成功
            print(f"成功！机器人已移动到路径点 #{current_waypoint_index + 1}！")
            # 移动到下一个路径点
            current_waypoint_index += 1
            send_moveit_goal() # 发送下一个目标
        else:
            print(f"\n失败！机器人未能移动到路径点 #{current_waypoint_index + 1}。错误码: {result['error_code']['val']}")
            print("请检查 roslaunch 终端是否有 [ERROR] 信息。")
            print("测试终止。")
            reactor.callFromThread(reactor.stop)
        
    def on_feedback(feedback):
        # MoveIt! 的 feedback 会提供当前状态
        # 为了避免刷屏，可以只在需要时打印
        # print(f"Feedback: {feedback['state']}")
        pass

    goal.on('result', on_result)
    goal.on('feedback', on_feedback)
    
    print("正在发送 Goal...")
    goal.send()
    print("Goal 已发送，等待结果...")

def on_ros_connect(client):
    global action_client
    print("成功连接到 ROS Bridge。")
    try:
        # MoveIt! 的主 Action Server 是 /move_group
        # 它的消息类型是 moveit_msgs/MoveGroupAction
        action_client = roslibpy.actionlib.ActionClient(
            client,
            '/move_group',
            'moveit_msgs/MoveGroupAction'
        )
        print("Action Client 已初始化，连接到 /move_group。")
        
        # 等待一小段时间，确保 Action Server 完全准备好
        time.sleep(1) 
        
        # 发送轨迹的第一个点
        send_moveit_goal()
    except Exception as e:
        print(f"设置过程中出错: {e}")
        reactor.callFromThread(reactor.stop)

def main():
    global ros_client
    ros_client = roslibpy.Ros(host='192.168.0.105', port=9090)
    ros_client.on_ready(lambda: on_ros_connect(ros_client), run_in_thread=True)
    print(f"正在尝试连接到 192.168.0.105:9090 ...")
    try:
        ros_client.run_forever()
    except Exception as e:
        print(f"发生致命错误: {e}")
    finally:
        print("脚本执行完毕。")

if __name__ == '__main__':
    main()