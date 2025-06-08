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
LEFT_ARM_STATE_TOPIC = '/get_left_arm_degree'
RIGHT_ARM_STATE_TOPIC = '/get_right_arm_degree'
ARM_STATE_MSG_TYPE = 'sensor_msgs/JointState'
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_joint{i+1}' for i in range(7)]
RIGHT_ARM_JOINT_NAMES_INTERNAL = [f'r_joint{i+1}' for i in range(7)]

# --- MoveIt! Action Server Configuration (NEW) ---
MOVE_GROUP_ACTION_NAME = '/move_group'
MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction' # 注意：这需要你的ROS环境中有 moveit_msgs 包

# MoveIt! Planning Parameters
PLANNING_GROUP_LEFT_ARM = 'l_arm' # 在你的SRDF文件中定义的左臂规划组名称
PLANNING_GROUP_RIGHT_ARM = 'r_arm' # 在你的SRDF文件中定义的右臂规划组名称
PLANNER_ID = "RRTConnect" # 推荐的快速规划器，你可以改为你配置的任何规划器，例如 "PRMstarkConfigDefault"

NUM_PLANNING_ATTEMPTS = 5
ALLOWED_PLANNING_TIME = 5.0
VELOCITY_SCALING_FACTOR = 0.5 # 默认速度缩放因子
ACCELERATION_SCALING_FACTOR = 0.5 # 默认加速度缩放因子


# --- HEAD ---
HEAD_SERVO_CMD_TOPIC = '/servo_control/move'
HEAD_SERVO_MSG_TYPE = 'servo_ros/ServoMove'
HEAD_SERVO_STATE_TOPIC = '/servo_state'
HEAD_SERVO_STATE_MSG_TYPE = 'servo_ros/ServoAngle'
HEAD_SERVO_RANGES = {
    'head_tilt_servo': {'id': 1, 'min': 300, 'max': 600, 'neutral': 500},
    'head_pan_servo': {'id': 2, 'min': 0, 'max': 1000, 'neutral': 500}
}

# --- HAND ---
LEFT_HAND_CMD_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
RIGHT_HAND_CMD_TOPIC = '/r_arm/rm_driver/Hand_SetAngle'
HAND_MSG_TYPE = 'dual_arm_msgs/Hand_Angle' # Assumed based on your info

# For UI and internal state tracking (6 DoFs per hand)
LEFT_HAND_DOF_NAMES = [f'l_hand_dof{i+1}' for i in range(6)]
RIGHT_HAND_DOF_NAMES = [f'r_hand_dof{i+1}' for i in range(6)]

MIN_HAND_ANGLE = 0
MAX_HAND_ANGLE = 1000
DEFAULT_HAND_ANGLE = 1000
HAND_ANGLE_STEP = 10
HAND_MARKS_STEP = 200

# --- NAVIGATION --- NEW SECTION
NAV_CMD_TOPIC = '/navigation_marker'
NAV_MSG_TYPE = 'std_msgs/String'
PREDEFINED_NAV_POINTS = ['charging', 'path01', 'door'] + [f'seat{i:02d}' for i in range(1, 17)]


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

