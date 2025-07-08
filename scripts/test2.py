#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import roslibpy.actionlib
from twisted.internet import reactor
import time

# --- 全局变量 ---
ros_client = None
action_client = None

def send_moveit_goal():
    """
    通过 MoveIt! 的 MoveGroup Action Server 发送一个规划和执行的目标。
    """
    if not action_client or not ros_client.is_connected:
        print("错误：Action Client 不可用或 ROS 未连接。")
        reactor.callFromThread(reactor.stop)
        return

    print("正在发送 MoveIt! 目标：让 'left_arm' 移动到一个关节位置...")

    # 定义目标关节位置
    # IMPORTANT: 关节名称需要和你的机器人 URDF/SRDF 文件中的完全一致
    # 你可以通过 `rostopic echo /joint_states` 查看关节名称
    # joint_names = ['l_joint1', 'l_joint2', 'l_joint3', 'l_joint4', 'l_joint5', 'l_joint6', 'l_joint7']
    joint_names = ['r_joint1', 'r_joint2', 'r_joint3', 'r_joint4', 'r_joint5', 'r_joint6', 'r_joint7']

    # 这是一个示例目标位置，请根据你的机器人进行调整
    # 这些值通常以弧度为单位
    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7]

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
        print(f"\n========================================")
        # MoveIt! 的返回结果中有一个 error_code 字段
        if result['error_code']['val'] == 1: # 1 表示成功
            print("\n成功！机器人已移动到目标位置！")
        else:
            print(f"\n失败！机器人未能移动。错误码: {result['error_code']['val']}")
            print("请检查 roslaunch 终端是否有 [ERROR] 信息。")
        print(f"========================================")
        print("\n测试完成，正在关闭连接。")
        reactor.callFromThread(reactor.stop)
        
    def on_feedback(feedback):
        # MoveIt! 的 feedback 会提供当前状态
        print(f"Feedback: {feedback['state']}")

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