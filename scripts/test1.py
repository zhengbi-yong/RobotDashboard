#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslibpy
import roslibpy.actionlib
from twisted.internet import reactor

# --- Global variables ---
ros_client = None
action_client = None

def send_final_test_goal():
    """
    Sends one simple, safe goal to the left arm to test the full system.
    """
    if not action_client or not ros_client.is_connected:
        print("Error: Action Client is not available or ROS is not connected.")
        reactor.callFromThread(reactor.stop)
        return

    print("Sending final test goal: Left arm to ZERO position...")
    zero_joint_goal = [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    goal_msg = {
        'arm_name': 'left',
        'target_type': 'joint',
        'joint_goal': zero_joint_goal 
    }
    # goal_msg = {
    #     'arm_name': 'right',
    #     'target_type': 'joint',
    #     'joint_goal': zero_joint_goal 
    # }
    goal = roslibpy.actionlib.Goal(action_client, roslibpy.Message(goal_msg))
    
    def final_result_callback(result):
        print(f"\n========================================")
        print(f"FINAL TEST RESULT: {result}")
        if result.get('success'):
            print("\nSUCCESS! The robot has moved! Congratulations!")
        else:
            print("\nFAILURE! The robot did not move. Please check the roslaunch terminal for any new [ERROR] messages.")
        print(f"========================================")
        print("\nTest complete. Closing connection.")
        reactor.callFromThread(reactor.stop)

    goal.on('result', final_result_callback)
    goal.send()
    print("Goal sent. Waiting for the result...")

def on_ros_connect(client):
    global action_client
    print("Connection to ROS Bridge successful.")
    try:
        action_client = roslibpy.actionlib.ActionClient(
            client,
            'dual_arm_control',
            'arm_control_without_collision/DualArmControlAction'
        )
        print("Action Client initialized.")
        send_final_test_goal()
    except Exception as e:
        print(f"Error during setup: {e}")
        reactor.callFromThread(reactor.stop)

def main():
    global ros_client
    ros_client = roslibpy.Ros(host='192.168.0.105', port=9090)
    ros_client.on_ready(lambda: on_ros_connect(ros_client), run_in_thread=True)
    print(f"Attempting to connect to 192.168.0.105:9090 ...")
    try:
        ros_client.run_forever()
    except Exception as e:
        print(f"A fatal error occurred: {e}")
    finally:
        print("Script finished.")

if __name__ == '__main__':
    main()