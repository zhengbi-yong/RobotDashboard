#!/usr/bin/env python3

import roslibpy
import roslibpy.actionlib
import time
import math
from twisted.internet import reactor
from loguru import logger
# --- 1. Configuration ---
ROS_BRIDGE_HOST = '192.168.0.105' # IMPORTANT: Change this to your robot's IP address
ROS_BRIDGE_PORT = 9090
MOVE_GROUP_ACTION_NAME = '/move_group'
MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
PLANNING_GROUP_LEFT_ARM = 'l_arm'
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_joint{i+1}' for i in range(7)]

# --- 2. Global Variables ---
ros_client = None
moveit_action_client = None
shutdown_timer = None

def _on_moveit_result(result):
    """Callback for handling the result from the MoveIt! action."""
    global shutdown_timer
    
    if shutdown_timer and shutdown_timer.active():
        logger.info("Action result received. Cancelling shutdown timer.")
        shutdown_timer.cancel()
        
    error_code = result['error_code']['val']
    if error_code == 1: # SUCCESS
        logger.info("MoveIt! goal completed successfully.")
    else:
        logger.error(f"MoveIt! goal failed with error code: {error_code}.")

    # --- FINAL FIX ---
    # Bypass the buggy roslibpy terminate() and use Twisted's standard stop method.
    logger.info("Requesting event loop shutdown...")
    reactor.callFromThread(reactor.stop) # This is the thread-safe way to stop the reactor.


def send_moveit_joint_goal(planning_group_name, joint_names, target_joint_angles_rad):
    """Constructs and sends a joint space goal to MoveIt!."""
    if not moveit_action_client or not ros_client.is_connected:
        logger.error("Error: MoveIt! Action Client is not available or ROS is not connected.")
        return

    logger.info(f"\n--- Sending MoveIt! Joint Goal ---")
    logger.info(f"Planning Group: {planning_group_name}")
    logger.info(f"Joints: {joint_names}")
    logger.info(f"Target (rad): {[f'{angle:.3f}' for angle in target_joint_angles_rad]}")
    logger.info("---------------------------------")

    joint_constraints = [{'joint_name': name, 'position': pos, 'tolerance_above': 0.01, 'tolerance_below': 0.01, 'weight': 1.0} for name, pos in zip(joint_names, target_joint_angles_rad)]
    goal_constraints = {'name': 'joint_space_goal_constraint', 'joint_constraints': joint_constraints}

    move_group_goal_message = {
        'request': {
            'group_name': planning_group_name,
            'goal_constraints': [goal_constraints],
            'num_planning_attempts': 5,
            'allowed_planning_time': 5.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.5,
            'max_acceleration_scaling_factor': 0.5
        },
        'planning_options': {'plan_only': False, 'replan': True, 'replan_attempts': 3}
    }
    
    goal = roslibpy.actionlib.Goal(moveit_action_client, move_group_goal_message)
    goal.on('result', _on_moveit_result)
    goal.send()
    logger.info(f"MoveIt! goal has been sent (Goal ID: {goal.goal_id}). Waiting for result...")

def timeout_shutdown():
    """This function is called by the timer if no result is received in time."""
    logger.warning("No result received from MoveIt! action in time. Shutting down.")
    # Also use the reactor to stop on timeout.
    reactor.callFromThread(reactor.stop)

def perform_actions_on_ready(client):
    """
    This is the main logic function, called after the connection is established.
    """
    global moveit_action_client, shutdown_timer
    logger.info("Connection to ROS Bridge successful.")
    logger.info("Initializing MoveIt! Action Client...")
    try:
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        logger.info("MoveIt! Action Client initialized.")

        # Using the safe goal that we know works.
        target_angles_deg = [15, -75, 0, 0, 0, 0, 0]
        logger.info(f"Target angles (deg): {target_angles_deg}")
        target_angles_rad = [math.radians(deg) for deg in target_angles_deg]

        send_moveit_joint_goal(
            planning_group_name=PLANNING_GROUP_LEFT_ARM,
            joint_names=LEFT_ARM_JOINT_NAMES_INTERNAL,
            target_joint_angles_rad=target_angles_rad
        )

        timeout_seconds = 20
        logger.info(f"Setting a {timeout_seconds}-second shutdown timer as a failsafe.")
        # call_later is still a roslibpy function, but it just schedules our reactor call.
        shutdown_timer = client.call_later(timeout_seconds, timeout_shutdown)

    except Exception as e:
        logger.error(f"An error occurred during on-ready setup: {e}")
        reactor.stop()

def main():
    """
    Main execution function. Sets up the ROS client and starts the event loop.
    """
    global ros_client
    logger.info("--- Robot Arm Control Script ---")
    logger.info(f"Connecting to ROS Bridge at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_client.on_ready(lambda: perform_actions_on_ready(ros_client))

    logger.info(f"Attempting to connect to {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
    try:
        ros_client.run()  # Starts the connection attempt
        logger.info("Waiting for connection...")
        start_time_conn = time.time()
        while not ros_client.is_connected:
            time.sleep(0.1)
            if time.time() - start_time_conn > 10:  # 10 seconds timeout
                raise TimeoutError(f"Connection to ROS Bridge timed out (over 10 seconds).")
        
        logger.info("Successfully connected to ROS Bridge server!")

    except TimeoutError as e:
        logger.error(f"Error: {e}")
        logger.info("Please check:")
        logger.info("1. Is the rosbridge server running on the target machine?")
        reactor.stop()
        return
    try:
        # run_forever() starts the Twisted reactor.
        ros_client.run_forever()
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt detected. Shutting down.")
    finally:
        # This will run after the reactor has been stopped by any of our shutdown calls.
        logger.info("Script finished.")

if __name__ == '__main__':
    main()