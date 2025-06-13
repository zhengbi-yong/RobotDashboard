#!/usr/bin/env python
import roslibpy
import roslibpy.actionlib
import time
import numpy as np
import threading

# Configuration parameters
ROS_BRIDGE_HOST = '192.168.0.105'  # rosbridge server IP
ROS_BRIDGE_PORT = 9090
CONNECTION_TIMEOUT_SEC = 10  # Connection timeout (seconds)
LEFT_ACTION_NAME = '/left_move_min_jerk'
RIGHT_ACTION_NAME = '/right_move_min_jerk'
ACTION_TYPE = 'magic_dual_arm_control/MoveMinJerkAction'
ERROR_THRESHOLD = 0.02  # Error norm threshold (radians)
DURATION = 1  # Motion duration (seconds)
DT = 0.005  # Control period (seconds)
FOLLOW = True  # High follow mode
MAX_WAIT_TIME = 5  # Maximum wait time (seconds)


# Target positions for each arm
LEFT_END_POS = [
    [-0.03456844906806946, 0.07543634920120239, -1.729801011657715, 
     -1.1775609394073487, 1.482970820236206, 1.2370304893493653, 
     -0.016978849825263025],  # Target point 1
    [-1.6167857898692024, -0.9299463581465973,-0.8822464798621881,
     -1.3718462091410721,2.0761389999626476, 0.7061951533672465,
     -0.02312561342116271,],  # Target point 2
    [-0.03456844906806946, 0.07543634920120239, -1.729801011657715, 
     -1.1775609394073487, 1.482970820236206, 1.2370304893493653, 
     -0.016978849825263025],  # Target point 3
    [-1.6167857898692024, -0.9299463581465973,-0.8822464798621881,
     -1.3718462091410721,2.0761389999626476, 0.7061951533672465,
     -0.02312561342116271,],  # Target point 4
    [-0.03456844906806946, 0.07543634920120239, -1.729801011657715, 
     -1.1775609394073487, 1.482970820236206, 1.2370304893493653, 
     -0.016978849825263025],  # Target point 1
    [-1.6167857898692024, -0.9299463581465973,-0.8822464798621881,
     -1.3718462091410721,2.0761389999626476, 0.7061951533672465,
     -0.02312561342116271,],  # Target point 2
    [-0.03456844906806946, 0.07543634920120239, -1.729801011657715, 
     -1.1775609394073487, 1.482970820236206, 1.2370304893493653, 
     -0.016978849825263025],  # Target point 3
    [-1.6167857898692024, -0.9299463581465973,-0.8822464798621881,
     -1.3718462091410721,2.0761389999626476, 0.7061951533672465,
     -0.02312561342116271,],  # Target point 4
]
RIGHT_END_POS = [
    [-0.010976050141453744, 0.10696850199699402,-1.3478902839660645,
     -0.7843600728988648, -1.794400972366333, 1.4662013957977296,
     -0.00022685000468045474],  # Target point 1
    [1.3954605916848593,0.857253378822067, -2.1035755513029164,
     -1.6145517971887895,-1.9111878620492109, 0.6947457577878051,
     -0.00425860342271984,],  # Target point 2
    [-0.010976050141453744, 0.10696850199699402,-1.3478902839660645,
     -0.7843600728988648, -1.794400972366333, 1.4662013957977296,
     -0.00022685000468045474],  # Target point 3
    [1.3954605916848593,0.857253378822067, -2.1035755513029164,
     -1.6145517971887895,-1.9111878620492109, 0.6947457577878051,
     -0.00425860342271984,],  # Target point 4
    [-0.010976050141453744, 0.10696850199699402,-1.3478902839660645,
     -0.7843600728988648, -1.794400972366333, 1.4662013957977296,
     -0.00022685000468045474],  # Target point 1
    [1.3954605916848593,0.857253378822067, -2.1035755513029164,
     -1.6145517971887895,-1.9111878620492109, 0.6947457577878051,
     -0.00425860342271984,],  # Target point 2
    [-0.010976050141453744, 0.10696850199699402,-1.3478902839660645,
     -0.7843600728988648, -1.794400972366333, 1.4662013957977296,
     -0.00022685000468045474],  # Target point 3
    [1.3954605916848593,0.857253378822067, -2.1035755513029164,
     -1.6145517971887895,-1.9111878620492109, 0.6947457577878051,
     -0.00425860342271984,],  # Target point 4
]

# Global variables for joint positions
left_joint_pos = None
right_joint_pos = None

# Get joint state once for a specific arm
def get_joint_state_once(client, topic_name, arm_name):
    global left_joint_pos, right_joint_pos
    joint_pos = None
    subscriber = roslibpy.Topic(client, topic_name, 'sensor_msgs/JointState')
    
    def on_joint_state(msg):
        nonlocal joint_pos
        joint_pos = msg['position']
        subscriber.unsubscribe()

    subscriber.subscribe(on_joint_state)
    start_time = time.time()
    while joint_pos is None:
        time.sleep(0.1)
        if time.time() - start_time > 2.0:
            subscriber.unsubscribe()
            raise TimeoutError(f"Unable to receive joint state data from {topic_name} for {arm_name} arm")
    if arm_name == 'left':
        global left_joint_pos
        left_joint_pos = joint_pos
    else:
        global right_joint_pos
        right_joint_pos = joint_pos
    return joint_pos

# Monitor joint state and check error for a specific arm
def monitor_joint_state(client, topic_name, target_pos, timeout, arm_name, result_dict):
    joint_pos = None
    error_norm = float('inf')
    subscriber = roslibpy.Topic(client, topic_name, 'sensor_msgs/JointState')
    
    def on_joint_state(msg):
        nonlocal joint_pos, error_norm
        joint_pos = msg['position']
        if (isinstance(joint_pos, list) and len(joint_pos) == 7 and
                all(isinstance(x, (int, float)) for x in joint_pos)):
            error = np.array(target_pos) - np.array(joint_pos)
            error_norm = np.linalg.norm(error)
    
    subscriber.subscribe(on_joint_state)
    start_time = time.time()
    while error_norm > ERROR_THRESHOLD:
        time.sleep(0.01)
        if time.time() - start_time > timeout:
            subscriber.unsubscribe()
            result_dict[arm_name] = (None, None, f"Target not reached within {timeout} seconds for {arm_name} arm")
            return
        if joint_pos is None:
            continue
    subscriber.unsubscribe()
    result_dict[arm_name] = (joint_pos, error_norm, None)

# Main program
print(f"Connecting to rosbridge: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)

try:
    client.run()
    start_time = time.time()
    while not client.is_connected:
        time.sleep(0.1)
        if time.time() - start_time > CONNECTION_TIMEOUT_SEC:
            raise TimeoutError(f"Connection timed out (exceeded {CONNECTION_TIMEOUT_SEC} seconds)")
    print("Connection successful!")

    # Get initial joint states for both arms
    left_joint_pos = get_joint_state_once(client, '/get_left_arm_degree', 'left')
    right_joint_pos = get_joint_state_once(client, '/get_right_arm_degree', 'right')
    
    for pos, arm in [(left_joint_pos, 'left'), (right_joint_pos, 'right')]:
        if not isinstance(pos, list) or len(pos) != 7:
            raise ValueError(f"Initial joint angles for {arm} arm have incorrect format: {pos}")
        if not all(isinstance(x, (int, float)) for x in pos):
            raise ValueError(f"Initial joint angles for {arm} arm contain non-numeric elements: {pos}")

    # Create Action Clients for both arms
    left_action_client = roslibpy.actionlib.ActionClient(client, LEFT_ACTION_NAME, ACTION_TYPE)
    right_action_client = roslibpy.actionlib.ActionClient(client, RIGHT_ACTION_NAME, ACTION_TYPE)

    # Process target points for both arms
    max_targets = max(len(LEFT_END_POS), len(RIGHT_END_POS))
    for i in range(max_targets):
        left_end_pos = LEFT_END_POS[i % len(LEFT_END_POS)] if i < len(LEFT_END_POS) else None
        right_end_pos = RIGHT_END_POS[i % len(RIGHT_END_POS)] if i < len(RIGHT_END_POS) else None

        # Validate target positions
        goals = []
        if left_end_pos and isinstance(left_end_pos, list) and len(left_end_pos) == 7 and all(isinstance(x, (int, float)) for x in left_end_pos):
            goals.append(('left', left_end_pos, left_action_client, left_joint_pos))
        elif left_end_pos:
            print(f"Left arm target {i+1} has incorrect format or non-numeric elements: {left_end_pos}, skipping")
        
        if right_end_pos and isinstance(right_end_pos, list) and len(right_end_pos) == 7 and all(isinstance(x, (int, float)) for x in right_end_pos):
            goals.append(('right', right_end_pos, right_action_client, right_joint_pos))
        elif right_end_pos:
            print(f"Right arm target {i+1} has incorrect format or non-numeric elements: {right_end_pos}, skipping")

        if not goals:
            print(f"No valid targets for iteration {i+1}, skipping")
            continue

        # Send goals for both arms
        result_dict = {}
        monitor_threads = []
        for arm_name, end_pos, action_client, current_pos in goals:
            goal_message = {
                'start_pos': current_pos,
                'end_pos': end_pos,
                'duration': DURATION,
                'dt': DT,
                'follow': FOLLOW
            }
            print(f"Sending target {i+1}/{max_targets} for {arm_name} arm...")
            goal = roslibpy.actionlib.Goal(action_client, goal_message)
            goal.send()
            topic_name = f"/get_{arm_name}_arm_degree"
            thread = threading.Thread(target=monitor_joint_state, args=(client, topic_name, end_pos, MAX_WAIT_TIME, arm_name, result_dict))
            monitor_threads.append(thread)
            thread.start()

        # Wait for monitoring threads to complete
        for thread in monitor_threads:
            thread.join()

        # Process results
        for arm_name, end_pos, _, current_pos in goals:
            if arm_name in result_dict:
                joint_pos, error_norm, error_msg = result_dict[arm_name]
                if error_msg:
                    print(f"Error for {arm_name} arm: {error_msg}")
                else:
                    print(f"Target {i+1} completed for {arm_name} arm! Actual position: {joint_pos}")
                    print(f"Error norm: {error_norm:.6f} radians")
                    if arm_name == 'left':
                        left_joint_pos = joint_pos
                    else:
                        right_joint_pos = joint_pos
            else:
                print(f"No result received for {arm_name} arm")

    print("All targets completed!")

except TimeoutError as e:
    print(f"Error: {e}")
    print("Please check: 1. Is rosbridge server running? 2. Is IP/port correct? 3. Is firewall allowing port 9090? 4. Are joint state topics published?")
except ValueError as e:
    print(f"Data error: {e}")
except Exception as e:
    print(f"Unexpected error: {e}")
    import traceback
    traceback.print_exc()
finally:
    if client.is_connected:
        print("Disconnecting...")
        client.terminate()
        time.sleep(0.5)
    print("Script execution completed.")