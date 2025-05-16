# RobotDashboard/config.py

import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

# ROS Configuration
ROS_BRIDGE_HOST = '192.168.0.105' # CHANGE THIS IF NEEDED
ROS_BRIDGE_PORT = 9090

# --- ARM ---
LEFT_ARM_CMD_TOPIC = '/l_arm/rm_driver/MoveJ_Cmd'
RIGHT_ARM_CMD_TOPIC = '/r_arm/rm_driver/MoveJ_Cmd'
ARM_MSG_TYPE = 'dual_arm_msgs/MoveJ'
LEFT_ARM_STATE_TOPIC = '/l_arm/joint_states'
RIGHT_ARM_STATE_TOPIC = '/r_arm/joint_states'
ARM_STATE_MSG_TYPE = 'sensor_msgs/JointState'
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_j{i+1}' for i in range(7)]
RIGHT_ARM_JOINT_NAMES_INTERNAL = [f'r_j{i+1}' for i in range(7)]

# --- HEAD ---
HEAD_SERVO_CMD_TOPIC = '/servo_control/move'
HEAD_SERVO_MSG_TYPE = 'servo_ros/ServoMove'
HEAD_SERVO_STATE_TOPIC = '/servo_state'
HEAD_SERVO_STATE_MSG_TYPE = 'servo_ros/ServoAngle'
HEAD_SERVO_RANGES = {
    'head_tilt_servo': {'id': 1, 'min': 300, 'max': 600, 'neutral': 500},
    'head_pan_servo': {'id': 2, 'min': 0, 'max': 1000, 'neutral': 500}
}

# --- HAND --- NEW SECTION
LEFT_HAND_CMD_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
RIGHT_HAND_CMD_TOPIC = '/r_arm/rm_driver/Hand_SetAngle'
HAND_MSG_TYPE = 'dual_arm_msgs/Hand_Angle' # Assumed based on your info

# For UI and internal state tracking (6 DoFs per hand)
# These are placeholder names, adjust if you have specific finger/joint names
LEFT_HAND_DOF_NAMES = [f'l_hand_dof{i+1}' for i in range(6)]
RIGHT_HAND_DOF_NAMES = [f'r_hand_dof{i+1}' for i in range(6)]

# IMPORTANT: Define realistic min, max, and default for your hand DoFs
MIN_HAND_ANGLE = 0      # Placeholder
MAX_HAND_ANGLE = 1000   # Placeholder (int16 can go higher, but practically?)
DEFAULT_HAND_ANGLE = 500 # Placeholder
HAND_ANGLE_STEP = 10    # Placeholder
HAND_MARKS_STEP = 200   # Placeholder for slider marks

# Trajectory Directory
TRAJECTORY_DIR = os.path.join(_THIS_DIR, "trajectories")
os.makedirs(TRAJECTORY_DIR, exist_ok=True)

# UI Slider Defaults & Ranges
ARM_SLIDER_MIN = -170
ARM_SLIDER_MAX = 170
ARM_SLIDER_DEFAULT = 0
ARM_SLIDER_STEP = 1
ARM_SLIDER_MARKS_STEP = 50

PLAYBACK_SPEED_DEFAULT = 0.3
PLAYBACK_DELAY_DEFAULT = 2.0