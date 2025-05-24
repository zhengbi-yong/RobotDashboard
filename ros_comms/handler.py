# RobotDashboard/ros_comms/handler.py

import roslibpy
import roslibpy.core
import threading
import time
import json
from .. import config
from ..callbacks import ros_message_callbacks

# --- Global variables ---
ros_client = None
ros_connection_status = "Disconnected"
ros_setup_done = False

# Arm publishers
left_arm_pub = None
right_arm_pub = None
# Head publisher
head_servo_pub = None
# Hand publishers
left_hand_pub = None
right_hand_pub = None
# Navigation publisher - NEW
nav_pub = None

# Subscribers
left_arm_sub = None
right_arm_sub = None
head_servo_sub = None
# Note: No hand state subscribers defined yet by user

subscriber_thread = None
ros_connection_thread = None

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

    print("[Subscriber Thread] Waiting for ROS client to be valid and connected...")
    wait_timeout = 10
    start_wait = time.time()
    while not (ros_client and ros_client.is_connected):
        if time.time() - start_wait > wait_timeout:
            print("[Subscriber Thread] Timed out waiting for a connected ros_client. Exiting.")
            return
        if not ros_client: # Check if client becomes None during wait
            print("[Subscriber Thread] ros_client became None while waiting. Exiting.")
            return
        time.sleep(0.2)
        

    print("[Subscriber Thread] Client connected. Setting up ROS subscriptions.")
    try:
        left_arm_sub = roslibpy.Topic(ros_client, config.LEFT_ARM_STATE_TOPIC, config.ARM_STATE_MSG_TYPE)
        left_arm_sub.subscribe(lambda msg: ros_message_callbacks.left_arm_state_callback(msg, latest_joint_states))
        print(f"[Subscriber Thread] Subscribed to left arm topic: {config.LEFT_ARM_STATE_TOPIC}")

        right_arm_sub = roslibpy.Topic(ros_client, config.RIGHT_ARM_STATE_TOPIC, config.ARM_STATE_MSG_TYPE)
        right_arm_sub.subscribe(lambda msg: ros_message_callbacks.right_arm_state_callback(msg, latest_joint_states))
        print(f"[Subscriber Thread] Subscribed to right arm topic: {config.RIGHT_ARM_STATE_TOPIC}")

        head_servo_sub = roslibpy.Topic(ros_client, config.HEAD_SERVO_STATE_TOPIC, config.HEAD_SERVO_STATE_MSG_TYPE)
        head_servo_sub.subscribe(lambda msg: ros_message_callbacks.head_servo_state_callback(msg, latest_joint_states))
        print(f"[Subscriber Thread] Subscribed to head servo topic: {config.HEAD_SERVO_STATE_TOPIC} with type {config.HEAD_SERVO_STATE_MSG_TYPE}")
        
        print("[Subscriber Thread] All subscriptions established successfully.")
        current_thread_subscriptions_ok = True
        ros_setup_done = True

        while ros_client and ros_client.is_connected and current_thread_subscriptions_ok:
            time.sleep(1)
        print("[Subscriber Thread] Loop exited (client disconnected or subscription issue).")

    except Exception as e:
        print(f"[Subscriber Thread] Error during subscription setup or listening: {e}")
    finally:
        print("[Subscriber Thread] Cleaning up and exiting.")
        ros_setup_done = False
        try:
            if left_arm_sub and left_arm_sub.is_advertised: left_arm_sub.unadvertise()
            if right_arm_sub and right_arm_sub.is_advertised: right_arm_sub.unadvertise()
            if head_servo_sub and head_servo_sub.is_advertised: head_servo_sub.unadvertise()
            print("[Subscriber Thread] Attempted to unadvertise topics.")
        except Exception as e_unsub:
            print(f"[Subscriber Thread] Error during unadvertising topics: {e_unsub}")


def _ros_connect_thread_target():
    global ros_client, ros_connection_status, ros_setup_done, subscriber_thread
    global left_arm_pub, right_arm_pub, head_servo_pub
    global left_hand_pub, right_hand_pub, nav_pub # MODIFIED: Added nav_pub

    local_ros_client = None
    try:
        print(f"[Connect Thread] Creating ROS client for {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}...")
        local_ros_client = roslibpy.Ros(config.ROS_BRIDGE_HOST, config.ROS_BRIDGE_PORT)
        local_ros_client.run()

        if not local_ros_client.is_connected:
            raise Exception("roslibpy.Ros.run() completed but client is not connected.")

        print("[Connect Thread] ROS Connection successful.")
        ros_client = local_ros_client
        ros_connection_status = f"Connected to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}"

        # Initialize publishers
        left_arm_pub = roslibpy.Topic(ros_client, config.LEFT_ARM_CMD_TOPIC, config.ARM_MSG_TYPE)
        right_arm_pub = roslibpy.Topic(ros_client, config.RIGHT_ARM_CMD_TOPIC, config.ARM_MSG_TYPE)
        head_servo_pub = roslibpy.Topic(ros_client, config.HEAD_SERVO_CMD_TOPIC, config.HEAD_SERVO_MSG_TYPE)
        left_hand_pub = roslibpy.Topic(ros_client, config.LEFT_HAND_CMD_TOPIC, config.HAND_MSG_TYPE)
        right_hand_pub = roslibpy.Topic(ros_client, config.RIGHT_HAND_CMD_TOPIC, config.HAND_MSG_TYPE)
        nav_pub = roslibpy.Topic(ros_client, config.NAV_CMD_TOPIC, config.NAV_MSG_TYPE) # NEW: Navigation Publisher
        print("[Connect Thread] All Publishers initialized.")

        ros_setup_done = False
        if subscriber_thread and subscriber_thread.is_alive():
            print("[Connect Thread] Warning: Old subscriber thread detected alive.")

        subscriber_thread = threading.Thread(target=start_ros_subscriber_thread_internal, daemon=True)
        subscriber_thread.start()
        print("[Connect Thread] Subscriber thread initiated.")

    except roslibpy.core.RosTimeoutError as e:
        error_msg = f"Error: Connection Timed Out - {str(e)}"
        print(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    except ConnectionRefusedError as e:
        error_msg = f"Error: Connection Refused by host - {str(e)}"
        print(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    except Exception as e:
        error_msg = f"Error: An unexpected error occurred in connection thread - {type(e).__name__}: {str(e)}"
        print(f"[Connect Thread] {error_msg}")
        ros_connection_status = error_msg
    finally:
        if not (ros_client and ros_client.is_connected):
            if local_ros_client:
                try:
                    if local_ros_client.is_connected: local_ros_client.terminate()
                except: pass
            ros_client = None
            ros_setup_done = False
            # Nullify publishers if connection failed before they were used or if client is gone
            left_arm_pub = right_arm_pub = head_servo_pub = None
            left_hand_pub = right_hand_pub = nav_pub = None # MODIFIED: Added nav_pub
        print("[Connect Thread] Connection attempt thread finished.")


def try_connect_ros():
    global ros_connection_thread, ros_connection_status
    ros_connection_status = f"Connecting to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT}..."
    print(ros_connection_status)
    if ros_connection_thread and ros_connection_thread.is_alive():
        print("Warning: Previous connection thread still alive before new start. Attempting to join.")
        ros_connection_thread.join(timeout=1.0)
    ros_connection_thread = threading.Thread(target=_ros_connect_thread_target, daemon=True)
    ros_connection_thread.start()


def safe_terminate_ros_client():
    global ros_client, ros_setup_done
    global left_arm_pub, right_arm_pub, head_servo_pub, left_hand_pub, right_hand_pub, nav_pub # MODIFIED: Added nav_pub
    global left_arm_sub, right_arm_sub, head_servo_sub, ros_connection_status

    ros_setup_done = False
    # Nullify publishers
    left_arm_pub = right_arm_pub = head_servo_pub = None
    left_hand_pub = right_hand_pub = nav_pub = None # MODIFIED: Added nav_pub

    try:
        if left_arm_sub and left_arm_sub.is_advertised: left_arm_sub.unadvertise()
    except Exception as e: print(f"Error unadvertising left_arm_sub: {e}")
    try:
        if right_arm_sub and right_arm_sub.is_advertised: right_arm_sub.unadvertise()
    except Exception as e: print(f"Error unadvertising right_arm_sub: {e}")
    try:
        if head_servo_sub and head_servo_sub.is_advertised: head_servo_sub.unadvertise()
    except Exception as e: print(f"Error unadvertising head_servo_sub: {e}")
    left_arm_sub = right_arm_sub = head_servo_sub = None
    
    if ros_client:
        client_to_terminate = ros_client
        ros_client = None
        try:
            is_connected_before_terminate = client_to_terminate.is_connected
            print(f"Attempting to terminate ROS client instance (was_connected: {is_connected_before_terminate})...")
            client_to_terminate.terminate()
            print("ROS client terminate() called on instance.")
        except AttributeError as e:
            print(f"AttributeError during ROS client termination: {e}. Client might have been None.")
        except Exception as e:
            print(f"Unexpected error during ROS client termination: {e}")
        finally:
            print("ROS client instance processed for termination.")
            ros_connection_status = "Disconnected"
    else:
        print("No active ros_client object to terminate.")
        ros_connection_status = "Disconnected"


def cleanup_ros_threads():
    global ros_connection_thread, subscriber_thread
    print("Initiating ROS threads cleanup...")
    safe_terminate_ros_client()
    if ros_connection_thread and ros_connection_thread.is_alive():
        print("Waiting for ROS connection thread to join...")
        ros_connection_thread.join(timeout=2.0)
        if ros_connection_thread.is_alive(): print("Warning: ROS connection thread did not join in time.")
    if subscriber_thread and subscriber_thread.is_alive():
        print("Waiting for ROS subscriber thread to join...")
        subscriber_thread.join(timeout=2.0)
        if subscriber_thread.is_alive(): print("Warning: ROS subscriber thread did not join in time.")
    print("ROS threads cleanup process finished.")