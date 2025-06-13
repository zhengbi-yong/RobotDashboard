#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib
import time
import math
from twisted.internet import reactor
from loguru import logger

# --- 1. Configuration ---
ROS_BRIDGE_HOST = '192.168.0.105'
ROS_BRIDGE_PORT = 9090

# --- FINAL, CORRECTED Configuration ---
LEFT_ARM_ACTION_NAME = '/left_move_min_jerk'
LEFT_ARM_ACTION_TYPE = 'magic_dual_arm_control/MoveMinJerkAction' # CORRECTED
LEFT_ARM_JOINT_NAMES = [f'l_joint{i+1}' for i in range(7)]

# --- 2. Global Variables ---
ros_client = None
arm_action_client = None
shutdown_timer = None
current_goal = None  # 新增：跟踪当前的目标
waypoints_deg = []
current_waypoint_index = 0
# --- NEW: We need to track the current position to provide the 'start_pos' for the next goal ---
current_position_deg = [0.0] * len(LEFT_ARM_JOINT_NAMES)

def _on_action_result(result, target_pos_for_this_move):
    """A single, unified callback to handle all action results."""
    global shutdown_timer, current_waypoint_index, current_position_deg, current_goal
    
    # 取消超时定时器
    if shutdown_timer and shutdown_timer.active():
        logger.info("Received action result, canceling shutdown timer.")
        shutdown_timer.cancel()
        shutdown_timer = None

    # 检查结果是否表示成功
    logger.info(f"Action result received: {result}")
    
    # 检查动作是否被取消
    if result.get('success') == False and 'preempted' in result.get('message', '').lower():
        logger.warning("Action was preempted/cancelled. Not updating position.")
        # 被取消的动作不应该继续轨迹执行
        return
    
    # 只有在成功时才更新位置
    if result.get('success', True):  # 默认认为成功，除非明确表示失败
        current_position_deg = target_pos_for_this_move
        logger.success(f"Move to {current_position_deg} deg completed successfully.")
        
        # 清除当前目标
        current_goal = None
        
        # 成功完成后继续下一个航点
        ros_client.call_later(0.5, continue_trajectory)
    else:
        logger.error(f"Move failed: {result}")
        # 失败的情况下也继续执行（可选择停止或继续）
        current_goal = None
        ros_client.call_later(1.0, continue_trajectory)

def continue_trajectory():
    """继续执行轨迹的下一个航点"""
    global current_waypoint_index
    current_waypoint_index += 1
    move_to_next_waypoint()

def _on_action_feedback(feedback):
    """处理Action反馈信息"""
    # 减少反馈日志的频率，避免日志过多
    progress = feedback.get('progress', 0)
    if int(progress * 100) % 10 == 0:  # 每20%输出一次，减少日志量
        logger.info(f"Action progress: {progress:.1%}")

def _on_action_timeout():
    """处理Action超时"""
    global current_goal, shutdown_timer, current_position_deg
    
    logger.error("Action goal timed out. This might indicate an issue with the action server.")
    
    # 取消当前目标
    if current_goal:
        try:
            logger.info("Attempting to cancel current goal...")
            current_goal.cancel()
            current_goal = None
        except Exception as e:
            logger.error(f"Failed to cancel goal: {e}")
    
    # 清除超时定时器
    if shutdown_timer:
        shutdown_timer = None
    
    # 超时情况下，保持当前位置不变，但继续下一个航点
    logger.warning("Due to timeout, maintaining current position and continuing to next waypoint...")
    ros_client.call_later(2.0, continue_trajectory)

def move_to_next_waypoint():
    """Sends the goal for the next waypoint in the main sequence."""
    global current_waypoint_index, waypoints_deg, current_position_deg

    if current_waypoint_index >= len(waypoints_deg):
        logger.success("--- Trajectory sequence completed successfully! ---")
        reactor.callFromThread(reactor.stop)
        return

    target_pos_deg = waypoints_deg[current_waypoint_index]
    logger.info(f"--- Moving from {current_position_deg} to waypoint {current_waypoint_index + 1}/{len(waypoints_deg)}: {target_pos_deg} ---")
    
    # 增加持续时间以避免超时 - 从5秒增加到8秒
    duration_secs = 8.0  # 增加到8秒给更多执行时间
    send_min_jerk_goal(current_position_deg, target_pos_deg, duration_secs=duration_secs)

def send_min_jerk_goal(start_pos_deg, end_pos_deg, duration_secs):
    """
    Constructs and sends a goal PERFECTLY MATCHING the MoveMinJerkGoal definition.
    """
    global shutdown_timer, current_goal
    
    if not arm_action_client or not ros_client.is_connected:
        logger.error("Error: Action Client not available or ROS not connected.")
        return

    # 确保没有正在执行的目标
    if current_goal:
        logger.warning("Previous goal still active, cancelling it first...")
        try:
            current_goal.cancel()
            # 等待一段时间确保取消完成
            time.sleep(0.1)
        except Exception as e:
            logger.error(f"Failed to cancel previous goal: {e}")
        current_goal = None

    start_pos_rad = [math.radians(deg) for deg in start_pos_deg]
    end_pos_rad = [math.radians(deg) for deg in end_pos_deg]

    # 使用更保守的控制参数
    goal_message = {
        'start_pos': start_pos_rad,
        'end_pos': end_pos_rad,
        'duration': duration_secs,
        'dt': 0.01,  # 恢复到0.01，提供更稳定的控制
        'follow': True
    }
    
    logger.info(f"Sending Goal to '{LEFT_ARM_ACTION_NAME}': duration={duration_secs}s")
    logger.debug(f"Goal details: {goal_message}")

    current_goal = roslibpy.actionlib.Goal(arm_action_client, goal_message)
    current_goal.on('feedback', _on_action_feedback)
    current_goal.on('result', lambda result: _on_action_result(result, end_pos_deg))
    current_goal.send()
    logger.info(f"Goal sent. Waiting for result...")

    # 增加超时时间，给更多时间执行
    timeout = duration_secs + 20.0  # 从duration + 15增加到duration + 20
    logger.info(f"Setting a {timeout}-second timeout timer as a failsafe.")
    shutdown_timer = ros_client.call_later(timeout, _on_action_timeout)

def timeout_shutdown():
    """This function is called by the timer if no result is received in time."""
    logger.warning("Movement timed out. Shutting down.")
    reactor.callFromThread(reactor.stop)

def perform_actions_on_ready(client):
    """
    Called after connection. Initializes the action client and starts the sequence.
    """
    global arm_action_client, waypoints_deg, current_waypoint_index, current_position_deg
    logger.info("Connection to ROS Bridge successful.")
    logger.info(f"Initializing Action Client for '{LEFT_ARM_ACTION_NAME}'...")
    
    try:
        arm_action_client = roslibpy.actionlib.ActionClient(client, LEFT_ARM_ACTION_NAME, LEFT_ARM_ACTION_TYPE)
        logger.info("Action Client initialized.")

        # 可以增加轨迹步长以减少总的运动时间，或保持原有设置
        waypoints_deg = [
            [5, -10, 0, -20, 0, 15, 0],     # 第一步
            [10, -20, 0, -35, 0, 25, 0],    # 第二步  
            [8, -35, 0, -50, 0, 35, 0],     # 第三步
            [0, 0, 0, 0, 0, 0, 0]           # 返回零位
        ]
        logger.info(f"Loaded a trajectory with {len(waypoints_deg)} waypoints.")
        
        current_position_deg = [0.0] * len(LEFT_ARM_JOINT_NAMES)
        current_waypoint_index = 0
        
        # 减少初始延迟，从3秒减少到1秒
        logger.info("Starting trajectory execution in 1 second...")
        ros_client.call_later(1.0, move_to_next_waypoint)
        
    except Exception as e:
        logger.error(f"An error occurred during on-ready setup: {e}")
        reactor.stop()

def main():
    """Main execution function (no changes here)."""
    global ros_client
    logger.info("--- Robot Arm Control Script (Corrected Direct Controller Mode) ---")
    logger.info(f"Connecting to ROS Bridge at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_client.on_ready(lambda: perform_actions_on_ready(ros_client))
    def on_error(e):
        logger.error(f"ROS Bridge Connection Error: {e}")
        if reactor.running:
            reactor.stop()
    ros_client.on('error', on_error)
    ros_client.on('close', lambda _: logger.info("Connection to ROS Bridge has been closed."))
    try:
        logger.info("Starting ROS Bridge event loop. Waiting for connection...")
        ros_client.run_forever()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt detected. Shutting down.")
    finally:
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        logger.info("Script finished.")

if __name__ == '__main__':
    main()