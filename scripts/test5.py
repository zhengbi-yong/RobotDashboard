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

GROUP_NAME = 'l_arm'
JOINT_NAMES = ['l_joint1', 'l_joint2', 'l_joint3', 'l_joint4', 'l_joint5', 'l_joint6', 'l_joint7']

# 关节空间轨迹的关键点 (Waypoints)
JOINT_WAYPOINTS = [
    [-1.2897993255615234, -1.8300513095855713, 2.15415012550354, 0.13206159720420838, -2.7281329360961912, -2.1953495765686037, 0.3710917130470276],
    [0.5, -0.5, 0.0, -1.2, 0.0, 1.0, 0.5],
    [-0.5, -0.5, 0.0, -1.2, 0.0, 1.0, -0.5],
    [0.0, -0.785, 0.0, -1.57, 0.0, 1.57, 0.785]
]
    # {
    #     "l_j1": -1.2897993255615234,
    #     "l_j2": -1.8300513095855713,
    #     "l_j3": 2.15415012550354,
    #     "l_j4": 0.13206159720420838,
    #     "l_j5": -2.7281329360961912,
    #     "l_j6": -2.1953495765686037,
    #     "l_j7": 0.3710917130470276,
    #     "r_j1": 0.013122399550676346,
    #     "r_j2": 1.262786653137207,
    #     "r_j3": -0.18123569440841675,
    #     "r_j4": 0.29345664014816286,
    #     "r_j5": -1.4748041797637939,
    #     "r_j6": -0.0017973500020802022,
    #     "r_j7": -1.3152239883422852,
    #     "head_tilt_servo": 401.0,
    #     "head_pan_servo": 504.0,
    #     "l_hand_dof1":600,
    #     "l_hand_dof2":600,
    #     "l_hand_dof3":600,
    #     "l_hand_dof4":600,
    #     "l_hand_dof5":1000,
    #     "l_hand_dof6":1000,
    #     "r_hand_dof1":1000,
    #     "r_hand_dof2":1000,
    #     "r_hand_dof3":1000,
    #     "r_hand_dof4":1000,
    #     "r_hand_dof5":1000,
    #     "r_hand_dof6":1000
    # }

# 为每个运动航段设置不同的速度缩放比例
SEGMENT_SPEEDS = [1.0, 1.0, 1.0, 1.0]

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

    # 使用第一个片段的轨迹结构作为基础
    final_trajectory = planned_segments[0]
    # 将所有轨迹点收集到一个列表中
    all_points = []
    time_offset = 0.0

    for segment in planned_segments:
        if not segment['joint_trajectory']['points']:
            continue # 跳过空的轨迹段
        
        # 重新计算时间戳并添加到总列表中
        for point in segment['joint_trajectory']['points']:
            new_point = copy.deepcopy(point)
            original_time = new_point['time_from_start']['secs'] + new_point['time_from_start']['nsecs'] * 1e-9
            new_total_time = time_offset + original_time
            new_point['time_from_start']['secs'] = int(new_total_time)
            new_point['time_from_start']['nsecs'] = int((new_total_time - int(new_total_time)) * 1e9)
            all_points.append(new_point)
            
        # 更新时间偏移量
        last_point_time = segment['joint_trajectory']['points'][-1]['time_from_start']
        time_offset += last_point_time['secs'] + last_point_time['nsecs'] * 1e-9
    
    # 将拼接好的点列表放回最终轨迹
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
    
    # 如果是第一段，起点是机器人当前状态
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
        if result['error_code']['val'] == 1 and result['planned_trajectory']:
            print(f"第 {segment_index + 1} 段轨迹规划成功！")
            planned_segment = result['planned_trajectory']
            accumulated_segments.append(planned_segment)

            # 获取这段轨迹的终点，作为下一段的起点
            next_start_state_point = planned_segment['joint_trajectory']['points'][-1]
            
            # 构建 sensor_msgs/JointState 消息，字段名必须是 'name'
            next_start_joint_state = {
                'name': planned_segment['joint_trajectory']['joint_names'], # 正确的字段名
                'position': next_start_state_point['positions']
            }
            
            # 递归调用，规划下一段
            # 传递完整的 moveit_msgs/RobotState 结构
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