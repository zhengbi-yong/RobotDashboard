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
MOVE_GROUP_ACTION_NAME = '/move_group'
MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
PLANNING_GROUP_LEFT_ARM = 'l_arm'
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_joint{i+1}' for i in range(7)]
MOVEMENT_TIMEOUT_SECONDS = 45  # 增加超时时间

# --- 添加关节状态订阅主题 ---
LEFT_ARM_STATE_TOPIC = '/get_left_arm_degree'
ARM_STATE_MSG_TYPE = 'sensor_msgs/JointState'

# --- 2. Global Variables ---
ros_client = None
moveit_action_client = None
joint_state_subscriber = None
shutdown_timer = None
waypoints_deg = []
current_waypoint_index = 0
current_joint_states = None
max_failed_attempts = 2  # 减少重试次数
current_failed_attempts = 0
target_reached_tolerance = 0.1  # 目标到达容差（弧度）

# --- 使用更保守的轨迹 ---
SAFE_INITIAL_POSITION_DEG = [0, 0, 0, 0, 0, 0, 0]

def on_joint_state_received(message):
    """处理关节状态消息"""
    global current_joint_states
    if message and 'position' in message:
        current_joint_states = message['position']
        logger.debug(f"Received joint states: {[f'{pos:.3f}' for pos in current_joint_states]}")

def wait_for_valid_joint_state(callback, timeout=10.0):
    """等待有效的关节状态，然后执行回调"""
    start_time = time.time()
    
    def check_state():
        if current_joint_states is not None:
            logger.info("Valid joint state received. Proceeding...")
            callback()
        elif time.time() - start_time > timeout:
            logger.warning("Timeout waiting for joint state. Proceeding anyway...")
            callback()
        else:
            ros_client.call_later(0.5, check_state)
    
    check_state()

def check_target_reached(target_angles_rad, tolerance=None):
    """检查是否到达目标位置"""
    if tolerance is None:
        tolerance = target_reached_tolerance
    
    if current_joint_states is None or len(current_joint_states) < 7:
        return False
    
    for i, (current, target) in enumerate(zip(current_joint_states[:7], target_angles_rad)):
        diff = abs(current - target)
        if diff > tolerance:
            logger.debug(f"Joint {i+1}: current={current:.3f}, target={target:.3f}, diff={diff:.3f}")
            return False
    
    return True

def wait_for_target_position(target_angles_rad, callback, max_wait_time=10.0):
    """等待机器人到达目标位置"""
    start_time = time.time()
    check_interval = 0.5
    
    def check_position():
        elapsed = time.time() - start_time
        
        if check_target_reached(target_angles_rad):
            logger.success("Target position reached!")
            callback()
        elif elapsed > max_wait_time:
            logger.warning(f"Timeout waiting for target position after {max_wait_time}s")
            callback()
        else:
            ros_client.call_later(check_interval, check_position)
    
    check_position()

def _on_initial_pos_result(result):
    """Callback ONLY for the initial positioning movement."""
    global shutdown_timer, current_failed_attempts
    
    if shutdown_timer and shutdown_timer.active():
        shutdown_timer.cancel()

    error_code = result['error_code']['val']
    logger.info(f"Initial position movement result: error_code = {error_code}")
    
    if error_code == 1:  # SUCCESS
        logger.success("Robot has successfully moved to the initial position.")
        current_failed_attempts = 0
        # 等待机器人实际到达目标位置
        initial_pos_rad = [math.radians(deg) for deg in SAFE_INITIAL_POSITION_DEG]
        wait_for_target_position(initial_pos_rad, lambda: ros_client.call_later(2.0, move_to_next_waypoint))
    else:
        logger.warning(f"Failed to move to initial position with error code: {error_code}.")
        current_failed_attempts += 1
        
        if current_failed_attempts < max_failed_attempts:
            logger.info(f"Retrying initial position (attempt {current_failed_attempts + 1}/{max_failed_attempts})...")
            ros_client.call_later(3.0, lambda: move_to_initial_position())
        else:
            logger.info("Max retry attempts reached. Starting trajectory from current position...")
            current_failed_attempts = 0
            ros_client.call_later(2.0, move_to_next_waypoint)

def _on_waypoint_result(result):
    """Callback for handling results of the main trajectory waypoints."""
    global shutdown_timer, current_waypoint_index, current_failed_attempts
    
    if shutdown_timer and shutdown_timer.active():
        shutdown_timer.cancel()
        
    error_code = result['error_code']['val']
    logger.info(f"Waypoint {current_waypoint_index + 1} result: error_code = {error_code}")
    
    if error_code == 1:  # SUCCESS
        logger.success(f"Waypoint {current_waypoint_index + 1} reached successfully.")
        current_failed_attempts = 0
        
        # 等待机器人实际到达目标位置
        target_angles_deg = waypoints_deg[current_waypoint_index]
        target_angles_rad = [math.radians(deg) for deg in target_angles_deg]
        
        def proceed_to_next():
            global current_waypoint_index
            current_waypoint_index += 1
            ros_client.call_later(3.0, move_to_next_waypoint)  # 增加等待时间
        
        wait_for_target_position(target_angles_rad, proceed_to_next, max_wait_time=8.0)
        
    else:
        error_messages = {
            -1: "FAILURE",
            -2: "PLANNING_FAILED", 
            -3: "INVALID_MOTION_PLAN",
            -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -5: "CONTROL_FAILED",
            -6: "UNABLE_TO_AQUIRE_SENSOR_DATA / INVALID_ROBOT_STATE",
            -7: "TIMED_OUT",
            -10: "INVALID_GROUP_NAME",
            -11: "INVALID_GOAL_CONSTRAINTS",
            -12: "INVALID_ROBOT_STATE",
            -13: "INVALID_LINK_NAME"
        }
        
        error_msg = error_messages.get(error_code, f"UNKNOWN_ERROR_{error_code}")
        logger.warning(f"MoveIt! goal failed at waypoint {current_waypoint_index + 1}: {error_msg}")
        
        # 对于超时错误，直接跳到下一个航点
        if error_code == -7:
            logger.info("Timeout error, moving to next waypoint...")
            current_waypoint_index += 1
            current_failed_attempts = 0
            ros_client.call_later(3.0, move_to_next_waypoint)
            return
        
        current_failed_attempts += 1
        
        if current_failed_attempts < max_failed_attempts and error_code in [-6, -12]:
            logger.info(f"Retrying waypoint {current_waypoint_index + 1} (attempt {current_failed_attempts + 1}/{max_failed_attempts})...")
            ros_client.call_later(5.0, lambda: retry_current_waypoint())
        else:
            logger.info("Moving to next waypoint...")
            current_waypoint_index += 1
            current_failed_attempts = 0
            ros_client.call_later(3.0, move_to_next_waypoint)

def retry_current_waypoint():
    """重试当前航点"""
    if current_waypoint_index < len(waypoints_deg):
        logger.info(f"--- Retrying waypoint {current_waypoint_index + 1}/{len(waypoints_deg)} ---")
        target_angles_deg = waypoints_deg[current_waypoint_index]
        target_angles_rad = [math.radians(deg) for deg in target_angles_deg]
        send_moveit_joint_goal(target_angles_rad, _on_waypoint_result)

def move_to_next_waypoint():
    """Sends the goal for the next waypoint in the main sequence."""
    global current_waypoint_index, waypoints_deg

    if current_waypoint_index >= len(waypoints_deg):
        logger.success("--- Trajectory sequence completed! ---")
        # 等待3秒后关闭，让最后的位置稳定
        ros_client.call_later(3.0, lambda: reactor.callFromThread(reactor.stop))
        return

    logger.info(f"--- Moving to waypoint {current_waypoint_index + 1}/{len(waypoints_deg)} ---")
    target_angles_deg = waypoints_deg[current_waypoint_index]
    logger.info(f"Target angles (deg): {target_angles_deg}")
    target_angles_rad = [math.radians(deg) for deg in target_angles_deg]

    send_moveit_joint_goal(target_angles_rad, _on_waypoint_result)

def send_moveit_joint_goal(target_joint_angles_rad, result_callback):
    """Generic function to construct and send a joint space goal with improved error handling."""
    global shutdown_timer
    
    if not moveit_action_client or not ros_client.is_connected:
        logger.error("Error: Action Client not available or ROS not connected.")
        return

    logger.info(f"Target angles (rad): {[f'{angle:.3f}' for angle in target_joint_angles_rad]}")
    
    # 构建简化的MoveIt!请求，减少可能的错误源
    joint_constraints = [
        {
            'joint_name': name, 
            'position': pos, 
            'tolerance_above': 0.1,  # 增加容差
            'tolerance_below': 0.1, 
            'weight': 1.0
        } 
        for name, pos in zip(LEFT_ARM_JOINT_NAMES_INTERNAL, target_joint_angles_rad)
    ]
    
    goal_constraints = {
        'name': 'joint_space_goal_constraint', 
        'joint_constraints': joint_constraints
    }

    move_group_goal_message = {
        'request': {
            'group_name': PLANNING_GROUP_LEFT_ARM, 
            'goal_constraints': [goal_constraints],
            'num_planning_attempts': 10,
            'allowed_planning_time': 10.0,
            'planner_id': "RRTConnect", 
            'max_velocity_scaling_factor': 0.3,  # 稍微提高速度
            'max_acceleration_scaling_factor': 0.3
        },
        'planning_options': {
            'plan_only': False, 
            'replan': True, 
            'replan_attempts': 5
        }
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.on('result', result_callback)
    goal.send()
    logger.info(f"MoveIt! goal sent (ID: {goal.goal_id}). Waiting for result...")
    
    # 设置超时
    shutdown_timer = ros_client.call_later(MOVEMENT_TIMEOUT_SECONDS, timeout_shutdown)

def move_to_initial_position():
    """移动到初始位置"""
    logger.info(f"Moving to initial position: {SAFE_INITIAL_POSITION_DEG} deg")
    initial_pos_rad = [math.radians(deg) for deg in SAFE_INITIAL_POSITION_DEG]
    send_moveit_joint_goal(initial_pos_rad, _on_initial_pos_result)

def timeout_shutdown():
    """This function is called by the timer if no result is received in time."""
    global current_waypoint_index, current_failed_attempts
    
    logger.warning("Movement timed out.")
    current_failed_attempts += 1
    
    if current_failed_attempts < max_failed_attempts:
        logger.info(f"Retrying current waypoint due to timeout (attempt {current_failed_attempts + 1}/{max_failed_attempts})...")
        ros_client.call_later(2.0, retry_current_waypoint)
    else:
        logger.info("Max timeout retries reached. Moving to next waypoint...")
        current_waypoint_index += 1
        current_failed_attempts = 0
        ros_client.call_later(2.0, move_to_next_waypoint)

def perform_actions_on_ready(client):
    """Called after connection. Sets up subscribers and starts trajectory."""
    global moveit_action_client, joint_state_subscriber, waypoints_deg
    
    logger.info("Connection to ROS Bridge successful. Setting up subscribers and action client...")
    
    try:
        # 初始化关节状态订阅
        joint_state_subscriber = roslibpy.Topic(client, LEFT_ARM_STATE_TOPIC, ARM_STATE_MSG_TYPE)
        joint_state_subscriber.subscribe(on_joint_state_received)
        logger.info(f"Subscribed to joint state topic: {LEFT_ARM_STATE_TOPIC}")
        
        # 初始化MoveIt! Action Client
        moveit_action_client = roslibpy.actionlib.ActionClient(client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE)
        logger.info("MoveIt! Action Client initialized.")

        # 定义更保守且更少的轨迹点
        waypoints_deg = [
            [10, -10, 0, -20, 0, 15, 0],   # 第一个目标
            [20, -20, 0, -40, 0, 30, 0],   # 第二个目标  
            [10, -10, 0, -20, 0, 15, 0],   # 返回中间位置
            [0, 0, 0, 0, 0, 0, 0]          # 返回零位
        ]
        logger.info(f"Loaded a trajectory with {len(waypoints_deg)} waypoints.")
        
        # 等待关节状态，然后开始初始定位
        logger.info("Waiting for valid joint state before starting...")
        wait_for_valid_joint_state(move_to_initial_position)

    except Exception as e:
        logger.error(f"An error occurred during setup: {e}")
        reactor.stop()

def main():
    """Main execution function."""
    global ros_client
    logger.info("--- Robot Arm Control Script (MoveIt! Trajectory with Position Verification) ---")
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
        if joint_state_subscriber:
            joint_state_subscriber.unsubscribe()
        if ros_client and ros_client.is_connected:
            ros_client.terminate()
        logger.info("Script finished.")

if __name__ == '__main__':
    main()