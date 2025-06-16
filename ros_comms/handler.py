# RobotDashboard/ros_comms/handler.py

import roslibpy
import roslibpy.actionlib
import roslibpy.core
import threading
import time
import json
from loguru import logger # IMPORT LOGURU
# --- FIX: Ensure explicit import of ros_message_callbacks ---
from ..callbacks import ros_message_callbacks 
from .. import config

# --- UNIQUE IDENTIFIER FOR HANDLER.PY VERSION ---
logger.info("--- HANDLER.PY VERSION: 2024-05-27-IMPORT-FIX-V8-LOGURU ---")
# --- END UNIQUE IDENTIFIER ---

# --- Global variables ---
ros_client = None
ros_connection_status = "Disconnected"
ros_setup_done = False

# MoveIt! Action Client
moveit_action_client = None
_last_moveit_error_code = None
_last_moveit_result_timestamp = 0

# Head publisher
head_servo_pub = None
# Hand publishers
left_hand_pub = None
right_hand_pub = None
# Navigation publisher
nav_pub = None

# Subscribers
left_arm_sub = None
right_arm_sub = None
head_servo_sub = None

subscriber_thread = None
ros_connection_thread = None

nav_joy_pub = None 

# Initialize latest_joint_states with hand DoFs
latest_joint_states = {
    **{name: 0.0 for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL},
    **{name: 0.0 for name in config.RIGHT_ARM_JOINT_NAMES_INTERNAL},
    'head_tilt_servo': float(config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral']),
    'head_pan_servo': float(config.HEAD_SERVO_RANGES['head_pan_servo']['neutral']),
    # Add hand DoFs with default values
    **{name: float(config.DEFAULT_HAND_ANGLE) for name in config.LEFT_HAND_DOF_NAMES},
    **{name: float(config.DEFAULT_HAND_ANGLE) for name in config.RIGHT_HAND_DOF_NAMES},
}

def start_ros_subscriber_thread_internal():
    global ros_client, left_arm_sub, right_arm_sub, head_servo_sub, ros_setup_done

    current_thread_subscriptions_ok = False
    ros_setup_done = False

    logger.info("[Subscriber Thread] Waiting for ROS client to be valid and connected...")
    wait_timeout = 10
    start_wait = time.time()
    while not (ros_client and ros_client.is_connected):
        time.sleep(0.1)
        if time.time() - start_wait > wait_timeout:
            logger.error("[Subscriber Thread] Timed out waiting for a connected ros_client (after 10s). Exiting subscriber thread.")
            return
        if not ros_client:
            logger.error("[Subscriber Thread] ros_client became None while waiting. Exiting subscriber thread.")
            return
    
    logger.info("[Subscriber Thread] Client connected. Attempting to set up ROS subscriptions now.")

    try:
        logger.info(f"[Subscriber Thread] Trying to subscribe to left arm topic: '{config.LEFT_ARM_STATE_TOPIC}' (type: '{config.ARM_STATE_MSG_TYPE}')")
        left_arm_sub = roslibpy.Topic(ros_client, config.LEFT_ARM_STATE_TOPIC, config.ARM_STATE_MSG_TYPE)
        # Use the imported ros_message_callbacks directly
        left_arm_sub.subscribe(lambda msg: ros_message_callbacks.left_arm_state_callback(msg, latest_joint_states))
        logger.info(f"[Subscriber Thread] Initiated subscription for left arm topic. Waiting for handshake...")

        logger.info(f"[Subscriber Thread] Trying to subscribe to right arm topic: '{config.RIGHT_ARM_STATE_TOPIC}' (type: '{config.ARM_STATE_MSG_TYPE}')")
        right_arm_sub = roslibpy.Topic(ros_client, config.RIGHT_ARM_STATE_TOPIC, config.ARM_STATE_MSG_TYPE)
        right_arm_sub.subscribe(lambda msg: ros_message_callbacks.right_arm_state_callback(msg, latest_joint_states))
        logger.info(f"[Subscriber Thread] Initiated subscription for right arm topic. Waiting for handshake...")

        logger.info(f"[Subscriber Thread] Trying to subscribe to head servo topic: '{config.HEAD_SERVO_STATE_TOPIC}' (type: '{config.HEAD_SERVO_STATE_MSG_TYPE}')")
        head_servo_sub = roslibpy.Topic(ros_client, config.HEAD_SERVO_STATE_TOPIC, config.HEAD_SERVO_STATE_MSG_TYPE)
        head_servo_sub.subscribe(lambda msg: ros_message_callbacks.head_servo_state_callback(msg, latest_joint_states))
        logger.info(f"[Subscriber Thread] Initiated subscription for head servo topic. Waiting for handshake...")
        
        time.sleep(1.0)

        left_arm_sub_ok = left_arm_sub.is_subscribed
        right_arm_sub_ok = right_arm_sub.is_subscribed
        head_servo_sub_ok = head_servo_sub.is_subscribed

        if left_arm_sub_ok and right_arm_sub_ok and head_servo_sub_ok:
            logger.info("[Subscriber Thread] All subscriptions established successfully (is_subscribed: True). Setting ros_setup_done to True.")
            current_thread_subscriptions_ok = True
            ros_setup_done = True
        else:
            logger.warning("[Subscriber Thread] Not all subscriptions reported as 'is_subscribed: True'. Check ROS environment.")
            if not left_arm_sub_ok: logger.warning(f"  - {config.LEFT_ARM_STATE_TOPIC} (is_subscribed: False). Is this topic published? Is type correct?")
            if not right_arm_sub_ok: logger.warning(f"  - {config.RIGHT_ARM_STATE_TOPIC} (is_subscribed: False). Is this topic published? Is type correct?")
            if not head_servo_sub_ok: logger.warning(f"  - {config.HEAD_SERVO_STATE_TOPIC} (is_subscribed: False). Is this topic published? Is type correct?")
            logger.warning("[Subscriber Thread] Keeping ros_setup_done as False due to failed subscriptions.")

        while ros_client and ros_client.is_connected and current_thread_subscriptions_ok:
            time.sleep(1)
        
        logger.info("[Subscriber Thread] Subscriber loop exited (client disconnected or subscription issue detected).")

    except Exception as e:
        # logger.exception automatically includes the traceback
        logger.exception(f"[Subscriber Thread] CRITICAL ERROR during subscription setup or listening: {type(e).__name__}: {e}")
        ros_setup_done = False
    finally:
        logger.info("[Subscriber Thread] Cleaning up subscriptions and exiting thread.")
        ros_setup_done = False
        try:
            if left_arm_sub and left_arm_sub.is_subscribed:
                logger.info(f"[Subscriber Thread] Unsubscribing from {config.LEFT_ARM_STATE_TOPIC}")
                left_arm_sub.unsubscribe()
            if right_arm_sub and right_arm_sub.is_subscribed:
                logger.info(f"[Subscriber Thread] Unsubscribing from {config.RIGHT_ARM_STATE_TOPIC}")
                right_arm_sub.unsubscribe()
            if head_servo_sub and head_servo_sub.is_subscribed:
                logger.info(f"Unsubscribing from {config.HEAD_SERVO_STATE_TOPIC}")
                head_servo_sub.unsubscribe()
            logger.info("[Subscriber Thread] Attempted to unsubscribe from topics.")
        except Exception as e_unsub:
            logger.error(f"Error during unsubscribing topics: {e_unsub}")


def _ros_connect_thread_target():
    global ros_client, ros_connection_status, ros_setup_done, subscriber_thread
    global head_servo_pub, left_hand_pub, right_hand_pub, nav_pub
    global moveit_action_client
    global nav_joy_pub
    local_ros_client = None
    try:
        logger.info(f"[Connect Thread] Creating ROS client for {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}...")
        local_ros_client = roslibpy.Ros(config.ROS_BRIDGE_HOST, config.ROS_BRIDGE_PORT)
        
        local_ros_client.run()

        if not local_ros_client.is_connected:
            raise Exception("roslibpy.Ros.run() completed but client is not connected. Check network/rosbridge.")

        logger.info("[Connect Thread] ROS Connection successful.")
        ros_client = local_ros_client
        ros_connection_status = f"Connected to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}"

        try:
            moveit_action_client = roslibpy.actionlib.ActionClient(
                ros_client, config.MOVE_GROUP_ACTION_NAME, config.MOVE_GROUP_ACTION_TYPE
            )
            moveit_action_client.on('result', _on_moveit_result)
            moveit_action_client.on('feedback', _on_moveit_feedback)
            moveit_action_client.on('status', _on_moveit_status)
            logger.info(f"[Connect Thread] MoveIt! Action Client initialized for '{config.MOVE_GROUP_ACTION_NAME}'.")
        except Exception as e:
            logger.error(f"[Connect Thread] Error initializing MoveIt! Action Client: {e}")
            raise

        head_servo_pub = roslibpy.Topic(ros_client, config.HEAD_SERVO_CMD_TOPIC, config.HEAD_SERVO_MSG_TYPE)
        left_hand_pub = roslibpy.Topic(ros_client, config.LEFT_HAND_CMD_TOPIC, config.HAND_MSG_TYPE)
        right_hand_pub = roslibpy.Topic(ros_client, config.RIGHT_HAND_CMD_TOPIC, config.HAND_MSG_TYPE)
        nav_pub = roslibpy.Topic(ros_client, config.NAV_CMD_TOPIC, config.NAV_MSG_TYPE)
        # --- 新增代码 ---
        nav_joy_pub = roslibpy.Topic(ros_client, config.NAV_JOY_CONTROL_TOPIC, config.NAV_JOY_CONTROL_MSG_TYPE)
        # --- 结束新增 ---
        logger.info("[Connect Thread] All Publishers initialized.")

        ros_setup_done = False
        if subscriber_thread and subscriber_thread.is_alive():
            logger.warning("[Connect Thread] Old subscriber thread detected alive. Attempting to join.")
            subscriber_thread.join(timeout=0.5)
            if subscriber_thread.is_alive():
                logger.warning("[Connect Thread] Old subscriber thread did not join in time, forcing re-creation.")

        subscriber_thread = threading.Thread(target=start_ros_subscriber_thread_internal, daemon=True)
        subscriber_thread.start()
        logger.info("[Connect Thread] Subscriber thread initiated.")

        while ros_client and ros_client.is_connected:
            time.sleep(1)
        logger.info("[Connect Thread] ROS client disconnected. Exiting connect thread.")

    except roslibpy.core.RosTimeoutError as e:
        error_msg = f"Error: Connection Timed Out - {str(e)}"
        logger.error(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    except ConnectionRefusedError as e:
        error_msg = f"Error: Connection Refused by host - {str(e)}"
        logger.error(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    except Exception as e:
        error_msg = f"An unexpected error occurred in connection thread - {type(e).__name__}: {str(e)}"
        # logger.exception will log the full stack trace
        logger.exception(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    finally:
        if not (ros_client and ros_client.is_connected):
            if local_ros_client:
                try:
                    if local_ros_client.is_connected: local_ros_client.terminate()
                except: pass
            ros_client = None
        ros_setup_done = False
        head_servo_pub = left_hand_pub = right_hand_pub = nav_pub = None
        moveit_action_client = None
        nav_joy_pub = None
        logger.info("[Connect Thread] Connection attempt thread finished.")


def try_connect_ros():
    global ros_connection_thread, ros_connection_status
    ros_connection_status = f"Connecting to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}..."
    logger.info(ros_connection_status)
    if ros_connection_thread and ros_connection_thread.is_alive():
        logger.warning("Previous connection thread still alive before new start. Attempting to join.")
        ros_connection_thread.join(timeout=1.0)
        if ros_connection_thread.is_alive():
            logger.warning("Previous connection thread did not join in time. Not starting new thread.")
            return
    ros_connection_thread = threading.Thread(target=_ros_connect_thread_target, daemon=True)
    ros_connection_thread.start()


def safe_terminate_ros_client():
    global ros_client, ros_setup_done
    global head_servo_pub, left_hand_pub, right_hand_pub, nav_pub
    global left_arm_sub, right_arm_sub, head_servo_sub, ros_connection_status
    global moveit_action_client

    ros_setup_done = False
    head_servo_pub = left_hand_pub = right_hand_pub = nav_pub = None

    if moveit_action_client:
        try:
            pass
        except Exception as e:
            logger.error(f"Error during MoveIt! Action Client cleanup: {e}")
        moveit_action_client = None

    try:
        if left_arm_sub:
            logger.info(f"Attempting to unsubscribe from {config.LEFT_ARM_STATE_TOPIC}")
            left_arm_sub.unsubscribe()
    except Exception as e: logger.error(f"Error unsubscribing left_arm_sub: {e}")
    try:
        if right_arm_sub:
            logger.info(f"Attempting to unsubscribe from {config.RIGHT_ARM_STATE_TOPIC}")
            right_arm_sub.unsubscribe()
    except Exception as e: logger.error(f"Error unsubscribing right_arm_sub: {e}")
    try:
        if head_servo_sub:
            logger.info(f"Attempting to unsubscribe from {config.HEAD_SERVO_STATE_TOPIC}")
            head_servo_sub.unsubscribe()
    except Exception as e: logger.error(f"Error unsubscribing head_servo_sub: {e}")
    left_arm_sub = right_arm_sub = head_servo_sub = None
    
    if ros_client:
        client_to_terminate = ros_client
        ros_client = None
        try:
            is_connected_before_terminate = client_to_terminate.is_connected
            logger.info(f"Attempting to terminate ROS client instance (was_connected: {is_connected_before_terminate})...")
            client_to_terminate.terminate()
            logger.info("ROS client terminate() called on instance.")
        except AttributeError as e:
            logger.error(f"AttributeError during ROS client termination: {e}. Client might have been None.")
        except Exception as e:
            logger.error(f"Unexpected error during ROS client termination: {e}")
        finally:
            logger.info("ROS client instance processed for termination.")
            ros_connection_status = "Disconnected"
    else:
        logger.info("No active ros_client object to terminate.")
        ros_connection_status = "Disconnected"


def cleanup_ros_threads():
    global ros_connection_thread, subscriber_thread
    logger.info("Initiating ROS threads cleanup...")
    safe_terminate_ros_client()
    if ros_connection_thread and ros_connection_thread.is_alive():
        logger.info("Waiting for ROS connection thread to join...")
        ros_connection_thread.join(timeout=2.0)
        if ros_connection_thread.is_alive(): logger.warning("ROS connection thread did not join in time.")
    if subscriber_thread and subscriber_thread.is_alive():
        logger.info("Waiting for ROS subscriber thread to join...")
        subscriber_thread.join(timeout=2.0)
        if subscriber_thread.is_alive(): logger.warning("ROS subscriber thread did not join in time.")
    logger.info("ROS threads cleanup process finished.")

# --- MoveIt! Action Client Callbacks ---
def _on_moveit_result(result):
    global _last_moveit_error_code, _last_moveit_result_timestamp
    logger.info("\n--- MoveIt! Action Result ---")
    if result['error_code']['val'] == 1: # moveit_msgs/MoveItErrorCodes.SUCCESS
        logger.info("MoveIt! goal completed successfully.")
        _last_moveit_error_code = 1 # SUCCESS
    else:
        logger.error(f"MoveIt! goal failed! Error code: {result['error_code']['val']}")
        _last_moveit_error_code = result['error_code']['val']
    _last_moveit_result_timestamp = time.time()

def _on_moveit_feedback(feedback):
    pass

def _on_moveit_status(status):
    pass

# Helper function to send MoveIt! Joint Space Goal
def send_moveit_joint_goal(planning_group_name, joint_names, target_joint_angles_rad,
                           velocity_scaling_factor=None,
                           acceleration_scaling_factor=None,
                           num_planning_attempts=None,
                           allowed_planning_time=None,
                           planner_id=None):
    global moveit_action_client

    # Assign default values from config if not provided
    if velocity_scaling_factor is None:
        velocity_scaling_factor = config.VELOCITY_SCALING_FACTOR
    if acceleration_scaling_factor is None:
        acceleration_scaling_factor = config.ACCELERATION_SCALING_FACTOR
    if num_planning_attempts is None:
        num_planning_attempts = config.NUM_PLANNING_ATTEMPTS
    if allowed_planning_time is None:
        allowed_planning_time = config.ALLOWED_PLANNING_TIME
    if planner_id is None:
        planner_id = config.PLANNER_ID

    if not moveit_action_client or not moveit_action_client.ros.is_connected:
        raise ConnectionError("MoveIt! Action Client is not connected or not initialized.")
    if not ros_setup_done:
        raise Exception("ROS setup not complete. MoveIt! Action Client may not be ready, or state topics are not received.")

    logger.debug("\n--- DEBUG: Inside send_moveit_joint_goal ---")
    logger.debug(f"DEBUG: planning_group_name type: {type(planning_group_name)}, value: {planning_group_name}")
    logger.debug(f"DEBUG: joint_names type: {type(joint_names)}, value: {joint_names}")
    logger.debug(f"DEBUG: target_joint_angles_rad type: {type(target_joint_angles_rad)}, value: {target_joint_angles_rad}")
    logger.debug(f"DEBUG: resolved num_planning_attempts: {num_planning_attempts}, type: {type(num_planning_attempts)}")
    logger.debug("--- END DEBUG PRINTS ---")

    joint_constraints = []
    
    if not isinstance(joint_names, list):
        logger.critical(f"'joint_names' is not a list right before loop! Actual type: {type(joint_names)}, value: {joint_names}")
        raise TypeError(f"'joint_names' parameter must be a list, but received {type(joint_names).__name__}")
    
    if len(joint_names) != len(target_joint_angles_rad):
        logger.error(f"Length mismatch! len(joint_names)={len(joint_names)}, len(target_joint_angles_rad)={len(target_joint_angles_rad)}")
        raise ValueError("Joint names list and target angles list length mismatch.")

    for i in range(len(joint_names)):
        if not isinstance(joint_names[i], str):
            logger.critical(f"joint_names[{i}] is not a string! Actual type: {type(joint_names[i])}, value: {joint_names[i]}")
            raise TypeError(f"Element at joint_names[{i}] must be a string, but received {type(joint_names[i]).__name__}")

        logger.debug(f"Loop iteration {i}. joint_names[{i}] type: {type(joint_names[i])}, value: {joint_names[i]}")
        joint_constraint = {
            'joint_name': joint_names[i],
            'position': target_joint_angles_rad[i],
            'tolerance_above': 0.01,
            'tolerance_below': 0.01,
            'weight': 1.0
        }
        joint_constraints.append(joint_constraint)

    goal_constraints = {
        'name': 'joint_space_goal_constraint',
        'joint_constraints': joint_constraints
    }

    motion_plan_request = {
        'group_name': planning_group_name,
        'goal_constraints': [goal_constraints],
        'num_planning_attempts': num_planning_attempts,
        'allowed_planning_time': allowed_planning_time,
        'planner_id': planner_id,
        'max_velocity_scaling_factor': velocity_scaling_factor,
        'max_acceleration_scaling_factor': acceleration_scaling_factor
    }

    move_group_goal_message = {
        'request': motion_plan_request,
        'planning_options': {
            'plan_only': False,
            'look_around': False,
            'replan': True,
            'replan_attempts': 2,
            'replan_delay': 0.1
        }
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.send()
    logger.info(f"Sent MoveIt! goal to group '{planning_group_name}' for joints: {joint_names}. Goal ID: {goal.goal_id}")
    return goal

# Function to retrieve the latest MoveIt! result for UI feedback
def get_latest_moveit_result():
    global _last_moveit_error_code, _last_moveit_result_timestamp
    return {
        'error_code': _last_moveit_error_code,
        'timestamp': _last_moveit_result_timestamp
    }

# Reset the latest MoveIt! result
def reset_latest_moveit_result():
    global _last_moveit_error_code, _last_moveit_result_timestamp
    _last_moveit_error_code = None
    _last_moveit_result_timestamp = 0

# --- 用下面这个最终修正版的函数完整替换掉旧的同名函数 ---
def send_moveit_pose_goal(planning_group, pose_data, velocity_scaling_factor=0.5):
    """
    Sends a MoveIt! goal based on a target end-effector pose. (FINAL CORRECTED STRUCTURE)
    """
    global moveit_action_client
    if not moveit_action_client:
        logger.warning("MoveIt! action client is not initialized.")
        return

    # MoveGroup.action 的目标 (Goal) 包含 'request' 和 'planning_options' 两个顶级字段
    goal_message = roslibpy.Message({
        'request': {
            'group_name': planning_group,
            'num_planning_attempts': 5,
            'allowed_planning_time': 5.0,
            'max_velocity_scaling_factor': velocity_scaling_factor,
            'planner_id': '',
            # --- 核心修正开始 ---
            'goal_constraints': [{
                # Constraints 消息没有 pose_stamped 字段.
                # 必须将位姿分解为位置和姿态两个独立的约束.
                'name': 'pose_goal', # 约束的名字
                'joint_constraints': [], # 保持为空
                
                # 1. 位置约束 (PositionConstraint)
                'position_constraints': [{
                    'header': {'frame_id': config.PLANNING_FRAME},
                    'link_name': config.END_EFFECTOR_LINKS.get(planning_group, ""), # 需要末端连杆的名称!
                    'constraint_region': {
                        'primitive_poses': [{
                            'position': pose_data['position'],
                            'orientation': {'w': 1.0} # 方向不重要，因为下面是 Bounding Volume
                        }],
                        'primitives': [{
                            'type': 1, # SPHERE
                            'dimensions': [0.01] # 1cm 的容差球体
                        }]
                    },
                    'weight': 1.0
                }],

                # 2. 姿态约束 (OrientationConstraint)
                'orientation_constraints': [{
                    'header': {'frame_id': config.PLANNING_FRAME},
                    'link_name': config.END_EFFECTOR_LINKS.get(planning_group, ""), # 需要末端连杆的名称!
                    'orientation': pose_data['orientation'],
                    'absolute_x_axis_tolerance': 0.05, # 大约3度
                    'absolute_y_axis_tolerance': 0.05,
                    'absolute_z_axis_tolerance': 0.05,
                    'weight': 1.0
                }],

                'visibility_constraints': [] # 保持为空
            }]
            # --- 核心修正结束 ---
        },
        'planning_options': {
            'planning_scene_diff': {'is_diff': True, 'robot_state': {'is_diff': True}},
            'plan_only': False,
            'replan': True,
            'replan_attempts': 5
        }
    })

    # 构建 roslibpy.actionlib.Goal 对象
    goal = roslibpy.actionlib.Goal(moveit_action_client, goal_message)
    
    logger.info(f"Sending POSE goal to planning group '{planning_group}' with FINAL CORRECTED structure...")
    goal.send()

def send_nav_joy_command(linear_vel, angular_vel):
    """
    Publishes a NavigationJoyControl message.
    """
    global nav_joy_pub
    if not nav_joy_pub or not ros_client or not ros_client.is_connected:
        logger.warning("Navigation Joy Publisher is not ready. Cannot send command.")
        return False

    message = roslibpy.Message({
        'linear_velocity': float(linear_vel),
        'angular_velocity': float(angular_vel)
    })
    
    try:
        nav_joy_pub.publish(message)
        logger.info(f"Published Nav Joy Command: linear={linear_vel}, angular={angular_vel}")
        return True
    except Exception as e:
        logger.exception(f"Failed to publish Navigation Joy Command: {e}")
        return False