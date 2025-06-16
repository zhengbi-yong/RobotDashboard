# RobotDashboard/callbacks/ros_message_callbacks.py

import json
from loguru import logger # IMPORT LOGURU
from .. import config # Relative import
import os
from datetime import datetime
import sys
# Note: These functions now accept 'current_joint_states_dict' as an argument
# to modify it directly, avoiding global state dependency within this file.

def left_arm_state_callback(message, current_joint_states_dict):
    logger.debug(f"[Left Arm Callback] Received message: {message}")
    if 'name' in message and 'position' in message and len(message['position']) == 7:
        for i in range(7):
            current_joint_states_dict[config.LEFT_ARM_JOINT_NAMES_INTERNAL[i]] = float(message['position'][i])

def right_arm_state_callback(message, current_joint_states_dict):
    logger.debug(f"[Right Arm Callback] Received message: {message}")
    if 'name' in message and 'position' in message and len(message['position']) == 7:
        for i in range(7):
            current_joint_states_dict[config.RIGHT_ARM_JOINT_NAMES_INTERNAL[i]] = float(message['position'][i])

def head_servo_state_callback(message, current_joint_states_dict):
    logger.debug(f"[Head Servo Callback] Received message: {json.dumps(message)}")

    servo_id_1 = message.get('servo_id_1')
    angle_1 = message.get('angle_1')
    if servo_id_1 is not None and angle_1 is not None:
        try:
            angle_1_float = float(angle_1)
            if servo_id_1 == config.HEAD_SERVO_RANGES['head_tilt_servo']['id']:
                current_joint_states_dict['head_tilt_servo'] = angle_1_float
            elif servo_id_1 == config.HEAD_SERVO_RANGES['head_pan_servo']['id']:
                 current_joint_states_dict['head_pan_servo'] = angle_1_float
        except ValueError:
            logger.warning(f"[Head Servo Callback] Could not convert angle_1 '{angle_1}' to float.")

    servo_id_2 = message.get('servo_id_2')
    angle_2 = message.get('angle_2')
    if servo_id_2 is not None and angle_2 is not None:
        try:
            angle_2_float = float(angle_2)
            if servo_id_2 == config.HEAD_SERVO_RANGES['head_pan_servo']['id']:
                current_joint_states_dict['head_pan_servo'] = angle_2_float
            elif servo_id_2 == config.HEAD_SERVO_RANGES['head_tilt_servo']['id']:
                current_joint_states_dict['head_tilt_servo'] = angle_2_float
        except ValueError:
            logger.warning(f"[Head Servo Callback] Could not convert angle_2 '{angle_2}' to float.")