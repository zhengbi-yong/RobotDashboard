#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib
import time
import math
# 导入 Twisted reactor 以正确处理关机
from twisted.internet import reactor

# --- 1. Configuration ---
ROS_BRIDGE_HOST = '192.168.0.105' # 重要：请确保这是您的机器人IP地址
ROS_BRIDGE_PORT = 9090
MOVE_GROUP_ACTION_NAME = '/move_group'
MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
PLANNING_GROUP_LEFT_ARM = 'l_arm'
# --- 新增的位姿控制配置 ---
PLANNING_FRAME = "l_base_link" # 机器人规划的参考坐标系
END_EFFECTOR_LINK = "l_hand_base_link" # 要控制的左臂末端连杆名称

# --- 2. Global Variables ---
ros_client = None
moveit_action_client = None
shutdown_timer = None

def _on_moveit_result(result):
    """处理MoveIt!动作结果的回调函数"""
    global shutdown_timer
    
    if shutdown_timer and shutdown_timer.active():
        print("收到动作结果，正在取消关机定时器。")
        shutdown_timer.cancel()
        
    error_code = result['error_code']['val']
    if error_code == 1: # SUCCESS
        print("\n[SUCCESS] MoveIt! 目标已成功完成。")
    else:
        print(f"\n[FAILURE] MoveIt! 目标失败，错误代码: {error_code}.")
    
    print("正在请求事件循环关闭...")
    reactor.callFromThread(reactor.stop)


def send_moveit_pose_goal(planning_group_name, target_pose, end_effector_link):
    """构建并发送一个末端位姿空间目标至MoveIt!"""
    if not moveit_action_client or not ros_client.is_connected:
        print("错误: MoveIt! Action Client 不可用或ROS未连接。")
        return

    print(f"\n--- 正在发送MoveIt! 位姿目标 ---")
    print(f"规划组: {planning_group_name}")
    print(f"末端连杆: {end_effector_link}")
    print(f"目标位置 (x,y,z): {target_pose['position']}")
    print(f"目标姿态 (w,x,y,z): {target_pose['orientation']}")
    print("---------------------------------")

    # 这是构建位姿目标的核心，结构比关节目标复杂
    move_group_goal_message = {
        'request': {
            'group_name': planning_group_name,
            'num_planning_attempts': 10, # 位姿规划更难，增加尝试次数
            'allowed_planning_time': 5.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.4,
            'max_acceleration_scaling_factor': 0.4,
            # --- 目标约束 ---
            'goal_constraints': [{
                'name': 'pose_goal_constraint',
                'joint_constraints': [], # 对于位姿目标，这里为空
                
                # 1. 位置约束
                'position_constraints': [{
                    'header': {'frame_id': PLANNING_FRAME},
                    'link_name': end_effector_link,
                    'constraint_region': {
                        'primitive_poses': [{'position': target_pose['position'], 'orientation': {'w': 1.0}}],
                        'primitives': [{'type': 1, 'dimensions': [0.01]}] # 1cm的容差球体
                    },
                    'weight': 1.0
                }],

                # 2. 姿态约束
                'orientation_constraints': [{
                    'header': {'frame_id': PLANNING_FRAME},
                    'link_name': end_effector_link,
                    'orientation': target_pose['orientation'],
                    'absolute_x_axis_tolerance': 0.1, # 约5.7度容差
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

def timeout_shutdown():
    """如果超时未收到结果，此函数将被调用以关闭脚本"""
    print("\n[TIMEOUT] 超时未收到MoveIt!动作结果。正在关闭。")
    reactor.callFromThread(reactor.stop)

def perform_actions_on_ready(client):
    """连接成功后执行的主要逻辑函数"""
    global moveit_action_client, shutdown_timer
    print("与ROS Bridge连接成功。")
    
    try:
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        print("MoveIt! Action Client 已初始化。")

        # --- 在这里定义您期望的末端位姿 ---
        # 位置(position)单位为米，相对于PLANNING_FRAME坐标系
        # 姿态(orientation)为四元数
        target_pose = {
            'position': {'x': 0.0, 'y': 0.3, 'z': 0.4},
            # 这个四元数代表末端朝向正前方，水平状态 (无旋转)
            'orientation': {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # --- 其他姿态示例 ---
            # 末端垂直朝下 (绕Y轴旋转-90度)
            # 'orientation': {'w': 0.707, 'x': 0.0, 'y': -0.707, 'z': 0.0}
        }
        
        send_moveit_pose_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            target_pose=target_pose,
            end_effector_link=END_EFFECTOR_LINK
        )

        timeout_seconds = 20
        print(f"已设置 {timeout_seconds} 秒的关机定时器作为保险。")
        shutdown_timer = client.call_later(timeout_seconds, timeout_shutdown)

    except Exception as e:
        print(f"在on-ready设置阶段发生错误: {e}")
        reactor.stop()

def main():
    """主执行函数，设置ROS客户端并启动事件循环"""
    global ros_client
    print("--- 机器人末端位姿控制脚本 ---")
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_client.on_ready(lambda: perform_actions_on_ready(ros_client))
    
    print(f"正在尝试连接到 {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    
    try:
        ros_client.run_forever()
    except KeyboardInterrupt:
        print("检测到键盘中断。正在关闭。")
    finally:
        print("脚本执行完毕。")

if __name__ == '__main__':
    main()