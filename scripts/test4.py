#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import roslibpy.actionlib
from twisted.internet import reactor
import time

# --- 全局变量 ---
ros_client = None
# 需要两个Action Client和一个Service Client
move_group_client = None      # 用于移动到初始位置
execute_action_client = None  # 用于执行已规划的轨迹
cartesian_path_service = None # 用于规划笛卡尔路径

# --- 用户可配置参数 ---

# 1. 机器人的规划组和末端执行器Link
#    - GROUP_NAME 必须与你的 moveit_config 中的组名一致
#    - END_EFFECTOR_LINK 是你希望控制其笛卡尔路径的那个link的名称
GROUP_NAME = 'r_arm'
END_EFFECTOR_LINK = 'r_hand_base_link' # 重要：请替换成你自己的末端link名称

# 2. 轨迹的起始关节位置 (一个安全的、已知的姿态)
#    机器人会首先移动到这里，然后从这个点开始计算笛卡尔路径
START_JOINT_POS = [0.1, -0.785, 0.0, -1.57, 0.0, 1.57, 0.785]

# 3. 笛卡尔路径点 (相对于 base_link 或你指定的参考坐标系)
#    定义一系列末端执行器的姿态 (position + orientation)
#    下面的例子是让末端执行器在Y轴上水平移动0.2米
#    注意：第一个路径点应该是机器人当前姿态，但我们让MoveIt自动处理
#    因此，这里的列表只包含目标路径点。
#    姿态四元数 [x, y, z, w]
CARTESIAN_WAYPOINTS = [
    { # 第一个目标点: 在机器人前方靠右的位置
        'position': {'x': 0.1, 'y': -0.1, 'z': 0.1},
        'orientation': {'x': 0.0, 'y': 0.707, 'z': 0.0, 'w': 0.707} # 让末端朝下
    },
    # { # 第二个目标点: 水平向左移动
    #     'position': {'x': 0.4, 'y': 0.2, 'z': 0.5},
    #     'orientation': {'x': 0.0, 'y': 0.707, 'z': 0.0, 'w': 0.707}
    # },
    # { # 第三个目标点: 向下移动
    #     'position': {'x': 0.4, 'y': 0.2, 'z': 0.3},
    #     'orientation': {'x': 0.0, 'y': 0.707, 'z': 0.0, 'w': 0.707}
    # }
]

# -------------------------------------------------------------

def execute_planned_trajectory(plan):
    """接收规划好的轨迹并发送给执行Action Server"""
    print("\n[步骤 3/3] 正在执行规划好的轨迹...")
    if not execute_action_client or not ros_client.is_connected:
        print("错误：执行轨迹的 Action Client 不可用。")
        reactor.callFromThread(reactor.stop)
        return

    # 创建 ExecuteTrajectory 的 Goal
    goal_msg = {'trajectory': plan}
    goal = roslibpy.actionlib.Goal(execute_action_client, roslibpy.Message(goal_msg))

    def on_execution_result(result):
        # error_code 定义在 moveit_msgs/MoveItErrorCodes.msg, 1=SUCCESS
        if result['error_code']['val'] == 1:
            print("\n========================================")
            print("成功！笛卡尔轨迹执行完毕！")
            print("========================================")
        else:
            print(f"\n执行轨迹失败。错误码: {result['error_code']['val']}")
        
        print("\n测试完成，正在关闭连接。")
        reactor.callFromThread(reactor.stop)

    goal.on('result', on_execution_result)
    goal.send()
    print("轨迹已发送给执行器，等待结果...")

def plan_cartesian_path():
    """调用服务来规划一条笛卡尔路径"""
    print("\n[步骤 2/3] 正在规划笛卡尔路径...")
    if not cartesian_path_service or not ros_client.is_connected:
        print("错误：笛卡尔路径规划服务不可用。")
        reactor.callFromThread(reactor.stop)
        return

    # 创建服务请求
    request_msg = {
        'header': {
            'stamp': {'secs': 0, 'nsecs': 0},
            'frame_id': 'base_link' # 重要: 确保这是你的机器人根坐标系
        },
        'start_state': { # 从机器人当前状态开始规划
            'is_diff': True
        },
        'group_name': GROUP_NAME,
        'link_name': END_EFFECTOR_LINK,
        'waypoints': CARTESIAN_WAYPOINTS,
        'max_step': 0.01,  # 路径点之间的最大距离(米)
        'jump_threshold': 0.0 # 禁用关节空间跳跃检测
    }
    
    request = roslibpy.ServiceRequest(request_msg)

    def on_plan_received(response):
        # fraction 是一个0到1之间的值，表示规划成功部分的比例
        fraction = response['fraction']
        print(f"路径规划完成，成功率为: {fraction:.2%}")

        if fraction > 0.99: # 几乎完全成功
            # 如果规划成功，执行轨迹
            execute_planned_trajectory(response['solution'])
        else:
            print("未能规划出完整的笛卡尔路径，请检查目标点是否可达。")
            reactor.callFromThread(reactor.stop)

    cartesian_path_service.call(request, on_plan_received)

def move_to_start_position():
    """使用关节目标，移动到轨迹的起始位置"""
    print("\n[步骤 1/3] 正在移动到起始关节位置...")
    if not move_group_client or not ros_client.is_connected:
        print("错误：MoveGroup Action Client 不可用。")
        reactor.callFromThread(reactor.stop)
        return

    joint_names = ['r_joint1', 'r_joint2', 'r_joint3', 'r_joint4', 'r_joint5', 'r_joint6', 'r_joint7']
    
    goal_msg = {
        'request': {
            'group_name': GROUP_NAME,
            'goal_constraints': [{
                'joint_constraints': [
                    {'joint_name': name, 'position': pos, 'weight': 1.0}
                    for name, pos in zip(joint_names, START_JOINT_POS)
                ]
            }],
            'allowed_planning_time': 10.0,
            'num_planning_attempts': 1
        }
    }
    goal = roslibpy.actionlib.Goal(move_group_client, roslibpy.Message(goal_msg))

    def on_result(result):
        if result['error_code']['val'] == 1:
            print("成功移动到起始位置！")
            # 成功后，开始规划笛卡尔路径
            plan_cartesian_path()
        else:
            print(f"移动到起始位置失败。错误码: {result['error_code']['val']}")
            reactor.callFromThread(reactor.stop)
    
    goal.on('result', on_result)
    goal.send()
    print("起始位置目标已发送，等待机器人就位...")

def on_ros_connect(client):
    global move_group_client, execute_action_client, cartesian_path_service
    print("成功连接到 ROS Bridge。")
    try:
        # 1. 初始化 /move_group Action Client
        move_group_client = roslibpy.actionlib.ActionClient(
            client, '/move_group', 'moveit_msgs/MoveGroupAction')
        print("MoveGroup Action Client已初始化。")

        # 2. 初始化 /execute_trajectory Action Client
        execute_action_client = roslibpy.actionlib.ActionClient(
            client, '/execute_trajectory', 'moveit_msgs/ExecuteTrajectoryAction')
        print("ExecuteTrajectory Action Client已初始化。")

        # 3. 初始化 /compute_cartesian_path Service Client
        cartesian_path_service = roslibpy.Service(
            client, '/compute_cartesian_path', 'moveit_msgs/GetCartesianPath')
        print("ComputeCartesianPath Service Client已初始化。")
        
        time.sleep(1) # 等待所有客户端和服务完全建立连接
        
        # 启动整个流程
        move_to_start_position()

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