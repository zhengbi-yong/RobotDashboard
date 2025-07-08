#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import roslibpy.actionlib
from twisted.internet import reactor
import time
import copy # 用于深拷贝消息

# --- 全局变量 ---
ros_client = None
move_group_client = None      # 用于规划
execute_action_client = None  # 用于执行

# --- 用户可配置参数 ---

### 修改点 1: 更新规划组名称 ###
GROUP_NAME = 'dual_arm'


### 修改点 2: 定义并合并双臂的关节名称 ###
# 确保这里的关节名称和顺序与您在MoveIt SRDF文件中定义的完全一致
L_JOINT_NAMES = ['l_joint1', 'l_joint2', 'l_joint3', 'l_joint4', 'l_joint5', 'l_joint6', 'l_joint7']
R_JOINT_NAMES = ['r_joint1', 'r_joint2', 'r_joint3', 'r_joint4', 'r_joint5', 'r_joint6', 'r_joint7']
# 合并后的列表，现在有14个关节
JOINT_NAMES = L_JOINT_NAMES + R_JOINT_NAMES

### 修改点 3: 更新关节空间轨迹的关键点 (Waypoints) ###
# 每个路点现在必须包含14个关节的角度值，顺序与上面的JOINT_NAMES对应
# 我为您创建了几个示例路点，您可以根据需要修改或添加
JOINT_WAYPOINTS = [
    # 路点 1: 双臂展开，类似欢迎姿态
    [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂 (l_j1 到 l_j7)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0    # 右臂 (r_j1 到 r_j7)
    ],
    # 路点 2: 您在问题中提供的姿态数据
    [
        0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,# 左臂
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0    # 右臂
    ],
    # 路点 3: 双臂垂下，回到一个比较自然的初始/结束位置
    [
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 左臂
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0   # 右臂
    ]
]

# 为每个运动航段设置不同的速度缩放比例
# 列表长度应与JOINT_WAYPOINTS的长度一致
SEGMENT_SPEEDS = [0.5, 0.8, 0.5]

# -------------------------------------------------------------
# 下面的代码逻辑与原来保持一致，无需修改
# -------------------------------------------------------------

def execute_final_trajectory(final_trajectory):
    print("\n[步骤 3/3] 正在执行拼接好的完整轨迹...")
    if not execute_action_client or not ros_client.is_connected:
        print("错误：执行轨迹的 Action Client 不可用。")
        reactor.callFromThread(reactor.stop)
        return

    goal_msg = {'trajectory': final_trajectory}
    goal = roslibpy.actionlib.Goal(execute_action_client, roslibpy.Message(goal_msg))

    def on_execution_result(result):
        if result['error_code']['val'] == 1:
            print("\n========================================")
            print("成功！连续轨迹执行完毕！")
            print("========================================")
        else:
            print(f"\n执行轨迹失败。错误码: {result['error_code']['val']}")
        
        reactor.callFromThread(reactor.stop)

    goal.on('result', on_execution_result)
    goal.send()
    print("完整轨迹已发送给执行器，等待结果...")


def stitch_and_retime_trajectories(planned_segments):
    print("\n[步骤 2/3] 正在拼接和重定时轨迹...")
    
    if not planned_segments:
        print("错误：没有可拼接的轨迹片段。")
        reactor.callFromThread(reactor.stop)
        return

    final_trajectory = planned_segments[0]
    all_points = []
    time_offset = 0.0

    for segment in planned_segments:
        if not segment['joint_trajectory']['points']:
            continue
        
        for point in segment['joint_trajectory']['points']:
            new_point = copy.deepcopy(point)
            original_time = new_point['time_from_start']['secs'] + new_point['time_from_start']['nsecs'] * 1e-9
            new_total_time = time_offset + original_time
            new_point['time_from_start']['secs'] = int(new_total_time)
            new_point['time_from_start']['nsecs'] = int((new_total_time - int(new_total_time)) * 1e9)
            all_points.append(new_point)
            
        if segment['joint_trajectory']['points']:
            last_point_time = segment['joint_trajectory']['points'][-1]['time_from_start']
            time_offset += last_point_time['secs'] + last_point_time['nsecs'] * 1e-9
    
    final_trajectory['joint_trajectory']['points'] = all_points
        
    print(f"轨迹拼接完成，总共 {len(final_trajectory['joint_trajectory']['points'])} 个点，预计总时长 {time_offset:.2f} 秒。")
    execute_final_trajectory(final_trajectory)


def plan_all_segments(segment_index=0, start_state=None, accumulated_segments=None):
    if accumulated_segments is None:
        accumulated_segments = []

    if segment_index >= len(JOINT_WAYPOINTS):
        print("\n所有轨迹片段规划完成。")
        stitch_and_retime_trajectories(accumulated_segments)
        return

    print(f"\n[步骤 1/{len(JOINT_WAYPOINTS)}] 正在规划第 {segment_index + 1} 段轨迹...")
    
    start_state_msg = {'is_diff': True} if start_state is None else start_state

    target_joint_positions = JOINT_WAYPOINTS[segment_index]
    target_speed = SEGMENT_SPEEDS[segment_index]

    goal_msg = {
        'request': {
            'group_name': GROUP_NAME,
            'start_state': start_state_msg,
            'goal_constraints': [{
                'joint_constraints': [
                    {'joint_name': name, 'position': pos, 'weight': 1.0}
                    for name, pos in zip(JOINT_NAMES, target_joint_positions)
                ]
            }],
            'max_velocity_scaling_factor': target_speed,
            'max_acceleration_scaling_factor': 0.5,
            'allowed_planning_time': 10.0
        },
        'planning_options': {
            'plan_only': True,
            'planning_scene_diff': {'is_diff': True}
        }
    }

    goal = roslibpy.actionlib.Goal(move_group_client, roslibpy.Message(goal_msg))

    def on_plan_result(result):
        if result['error_code']['val'] == 1 and result.get('planned_trajectory'):
            print(f"第 {segment_index + 1} 段轨迹规划成功！")
            planned_segment = result['planned_trajectory']
            accumulated_segments.append(planned_segment)

            next_start_state_point = planned_segment['joint_trajectory']['points'][-1]
            
            next_start_joint_state = {
                'name': planned_segment['joint_trajectory']['joint_names'],
                'position': list(next_start_state_point['positions'])
            }
            
            plan_all_segments(segment_index + 1, {'joint_state': next_start_joint_state}, accumulated_segments)
        else:
            error_code = result.get('error_code', {}).get('val', 'N/A')
            print(f"\n规划第 {segment_index + 1} 段轨迹失败。错误码: {error_code}")
            print("请检查目标点是否可达，或增加 allowed_planning_time。")
            reactor.callFromThread(reactor.stop)
    
    goal.on('result', on_plan_result)
    goal.send()
    print(f"目标已发送，等待第 {segment_index + 1} 段规划结果...")


def on_ros_connect(client):
    global move_group_client, execute_action_client
    print("成功连接到 ROS Bridge。")
    try:
        move_group_client = roslibpy.actionlib.ActionClient(
            client, '/move_group', 'moveit_msgs/MoveGroupAction')
        print("MoveGroup Action Client已初始化。")

        execute_action_client = roslibpy.actionlib.ActionClient(
            client, '/execute_trajectory', 'moveit_msgs/ExecuteTrajectoryAction')
        print("ExecuteTrajectory Action Client已初始化。")
        
        time.sleep(1)
        plan_all_segments()
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
    finally:
        print("脚本执行完毕。")

if __name__ == '__main__':
    main()