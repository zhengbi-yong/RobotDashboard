# RobotDashboard/config.py

import os

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

# ROS Configuration
ROS_BRIDGE_HOST = '192.168.0.105' # CHANGE THIS IF NEEDED
# ROS_BRIDGE_HOST = 'www.wanrenai.com' # CHANGE THIS IF NEEDED
# ROS_BRIDGE_HOST = '127.0.0.1' # CHANGE THIS IF NEEDED
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

# --- ARM CONTROL MODE CONFIGURATION (NEW) ---
ARM_CONTROL_MODES = {
    'moveit': 'MoveIt! 规划控制',
    'direct': '直接关节角控制'
}
DEFAULT_ARM_CONTROL_MODE = 'moveit'  # 默认使用MoveIt控制

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

# --- DIRECT JOINT CONTROL PARAMETERS ---
JOINT_CONTROL_SPEED = 0.3  # 直接关节控制时的速度参数（降低到0.3）
JOINT_CONTROL_TRAJECTORY_CONNECT = 1  # 轨迹连接标志：0=独立指令，1=连接轨迹
JOINT_ANGLE_UNIT = 'degrees'  # 关节角度单位：'degrees' 或 'radians'


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
PREDEFINED_NAV_POINTS = ['chongdian','chumen', 'quA7', 'gebigongsi', 'A7kaihui', 'A7beimian', 'gebibeimian', 'xiaohuiyishi', 'quqiantai', 'qiantaihuiyishi', 'zhanshiqu', 'qiantaimenkou', 'bolimenguodao', 'bolimenguodao1', 'qudengdianti', 'dianti1', 'dianti1li','dianti2', 'dianti2li','dianti3', 'dianti3li','dianti4', 'dianti4li','dianti5', 'dianti5li','dianti6', 'dianti6li', 'path01', 'door'] + [f'seat{i:02d}' for i in range(1, 17)]


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

PLANNING_FRAME = "base_link"  # 或者您机器人的主坐标系名称
END_EFFECTOR_LINKS = {
    PLANNING_GROUP_LEFT_ARM: "l_hand_base_link",
    PLANNING_GROUP_RIGHT_ARM: "r_hand_base_link"
}

# --- Joystick-style Navigation Control ---
# This is for sending continuous velocity commands, different from preset navigation goals.
NAV_JOY_CONTROL_TOPIC = '/navigation_joy_control'
# NOTE: The message type from your example is 'agv_ros/NavigationJoyControl'.
# Please ensure this matches the package name in your ROS environment.
NAV_JOY_CONTROL_MSG_TYPE = 'agv_ros/NavigationJoyControl' 

# --- Navigation Speed Constants ---
NAV_LINEAR_SPEED = 0.3      # Linear velocity in m/s
NAV_ANGULAR_SPEED = 0.5     # Angular velocity in rad/s

# --- ROBOT ACTION API CONFIGURATION (NEW) ---
ROBOT_ACTION_API_BASE_URL = 'http://117.147.181.50:5198'
ROBOT_ACTION_ENDPOINTS = {
    'set_manual_asr_result': '/agent_service/set_manual_asr_result/',
    'speak': '/agent_service/speak/',
    'force_command': '/agent_service/force_command/'
}

# 预定义动作列表
ROBOT_ACTIONS = [
    {'id': 'woshou', 'name': '握手', 'category': '交互'},
    {'id': 'huishou', 'name': '挥手', 'category': '交互'},
    {'id': 'zhaoshou', 'name': '招手', 'category': '交互'},
    {'id': 'huanying', 'name': '欢迎', 'category': '交互'},
    {'id': 'huanyingkeren', 'name': '欢迎客人', 'category': '交互'},
    {'id': 'yongbao', 'name': '拥抱', 'category': '交互'},
    {'id': 'pengquan', 'name': '碰拳', 'category': '交互'},
    
    {'id': 'tiaowu', 'name': '跳舞', 'category': '娱乐'},
    {'id': 'shoubiwu', 'name': '手臂舞', 'category': '娱乐'},
    {'id': 'shoushiwu', 'name': '手势舞蹈', 'category': '娱乐'},
    
    {'id': 'jingli', 'name': '敬礼', 'category': '礼仪'},
    {'id': 'zhaocaimao', 'name': '招财猫', 'category': '特色'},
    {'id': 'lanhuazhi', 'name': '兰花指', 'category': '特色'},
    {'id': 'zhizhuxia', 'name': '蜘蛛侠', 'category': '特色'},
    
    {'id': 'dibao', 'name': '递包', 'category': '服务'},
    {'id': 'baogaun', 'name': '保管', 'category': '服务'},
    {'id': 'nabao', 'name': '拿包', 'category': '服务'},
    {'id': 'jizhang', 'name': '记账', 'category': '服务'},
    
    {'id': 'fangshou', 'name': '放手', 'category': '手势'},
    {'id': 'songshou', 'name': '松手', 'category': '手势'},
    {'id': 'wojin', 'name': '握紧', 'category': '手势'},
    
    {'id': 'naotou', 'name': '挠头', 'category': '表情'},
    {'id': 'tiaoxin', 'name': '挑衅', 'category': '表情'},
    {'id': 'wulian', 'name': '捂脸', 'category': '表情'},
    {'id': 'zuoshouwulian', 'name': '左手捂脸', 'category': '表情'},
    {'id': 'sikao', 'name': '思考', 'category': '表情'},
    
    {'id': 'zhitian', 'name': '指天', 'category': '指示'},
    {'id': 'zhihui', 'name': '指挥', 'category': '指示'},
    {'id': 'zhitiankong', 'name': '指天空', 'category': '指示'},
    
    {'id': 'num0', 'name': '数字0', 'category': '数字'},
    {'id': 'num1', 'name': '数字1', 'category': '数字'},
    {'id': 'num2', 'name': '数字2', 'category': '数字'},
    {'id': 'num3', 'name': '数字3', 'category': '数字'},
    {'id': 'num4', 'name': '数字4', 'category': '数字'},
    
    {'id': 'caboli', 'name': '擦玻璃', 'category': '其他'},
    {'id': 'shitoujiandaobu', 'name': '石头剪刀布', 'category': '其他'},
]