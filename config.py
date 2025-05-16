# your_robot_dashboard/config.py

import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# ROS Configuration
ROS_BRIDGE_HOST = '192.168.0.105' # CHANGE THIS IF NEEDED
ROS_BRIDGE_PORT = 9090

# ROS Topics and Message Types
LEFT_ARM_CMD_TOPIC = '/l_arm/rm_driver/MoveJ_Cmd'
RIGHT_ARM_CMD_TOPIC = '/r_arm/rm_driver/MoveJ_Cmd'
ARM_MSG_TYPE = 'dual_arm_msgs/MoveJ'
HEAD_SERVO_CMD_TOPIC = '/servo_control/move'
HEAD_SERVO_MSG_TYPE = 'servo_ros/ServoMove'
LEFT_ARM_STATE_TOPIC = '/l_arm/joint_states'
RIGHT_ARM_STATE_TOPIC = '/r_arm/joint_states'
HEAD_SERVO_STATE_TOPIC = '/servo_state'
ARM_STATE_MSG_TYPE = 'sensor_msgs/JointState'
HEAD_SERVO_STATE_MSG_TYPE = 'servo_ros/ServoAngle'

# Joint Name Mappings and Servo Ranges
LEFT_ARM_JOINT_NAMES_INTERNAL = [f'l_j{i+1}' for i in range(7)]
RIGHT_ARM_JOINT_NAMES_INTERNAL = [f'r_j{i+1}' for i in range(7)]
HEAD_SERVO_RANGES = {
    'head_tilt_servo': {'id': 1, 'min': 300, 'max': 600, 'neutral': 500},
    'head_pan_servo': {'id': 2, 'min': 0, 'max': 1000, 'neutral': 500}
}

# Trajectory Directory - 修改这里
# 将 TRAJECTORY_DIR 定义为相对于当前 config.py 文件所在目录的路径
TRAJECTORY_DIR = os.path.join(_THIS_DIR, "trajectories")
os.makedirs(TRAJECTORY_DIR, exist_ok=True) # 确保这个目录存在

# UI Slider Defaults & Ranges (example, you can expand this)
ARM_SLIDER_MIN = -170
ARM_SLIDER_MAX = 170
ARM_SLIDER_DEFAULT = 0
ARM_SLIDER_STEP = 1
ARM_SLIDER_MARKS_STEP = 50

PLAYBACK_SPEED_DEFAULT = 0.3
PLAYBACK_DELAY_DEFAULT = 2.0