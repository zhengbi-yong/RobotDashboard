#!/usr/bin/env python3
"""
机器人不执行指令诊断脚本
综合诊断可能的问题原因
"""

import sys
import os
import math
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger
import roslibpy

def diagnose_connection():
    """诊断连接状态"""
    logger.info("=== 连接状态诊断 ===")
    
    logger.info(f"ROS Client: {ros_handler.ros_client}")
    if ros_handler.ros_client:
        logger.info(f"Is Connected: {ros_handler.ros_client.is_connected}")
    
    logger.info(f"Left Arm Publisher: {ros_handler.left_arm_pub}")
    logger.info(f"Right Arm Publisher: {ros_handler.right_arm_pub}")
    logger.info(f"ROS Setup Done: {ros_handler.ros_setup_done}")
    
    return ros_handler.ros_client and ros_handler.ros_client.is_connected

def diagnose_message_format():
    """诊断消息格式"""
    logger.info("=== 消息格式诊断 ===")
    
    # 从配置中获取话题和消息类型
    from config import LEFT_ARM_CMD_TOPIC, RIGHT_ARM_CMD_TOPIC, ARM_MSG_TYPE
    
    logger.info(f"Left Arm Topic: {LEFT_ARM_CMD_TOPIC}")
    logger.info(f"Right Arm Topic: {RIGHT_ARM_CMD_TOPIC}")
    logger.info(f"Message Type: {ARM_MSG_TYPE}")
    
    # 构建测试消息
    test_message_deg = {
        'joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'speed': 0.3,
        'trajectory_connect': 1
    }
    
    test_message_rad = {
        'joint': [math.radians(x) for x in [0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        'speed': 0.3,
        'trajectory_connect': 1
    }
    
    logger.info(f"Test message (degrees): {test_message_deg}")
    logger.info(f"Test message (radians): {test_message_rad}")

def test_basic_movement():
    """测试基本运动"""
    logger.info("=== 基本运动测试 ===")
    
    if not diagnose_connection():
        logger.error("连接未建立，无法进行运动测试")
        return
    
    # 测试极小角度（安全）
    tiny_angles = [1.0, 0, 0, 0, 0, 0, 0]  # 只移动第一个关节1度
    
    logger.info(f"测试极小角度移动: {tiny_angles}")
    
    # 测试度数
    logger.info("尝试发送度数...")
    result1 = ros_handler.send_direct_joint_command('right', tiny_angles, use_radians=False)
    logger.info(f"度数结果: {result1}")
    time.sleep(5)
    
    # 测试弧度
    logger.info("尝试发送弧度...")
    result2 = ros_handler.send_direct_joint_command('right', tiny_angles, use_radians=True)
    logger.info(f"弧度结果: {result2}")
    time.sleep(5)
    
    # 归位
    logger.info("尝试归位...")
    zero_angles = [0, 0, 0, 0, 0, 0, 0]
    ros_handler.send_direct_joint_command('right', zero_angles, use_radians=False)
    time.sleep(2)
    ros_handler.send_direct_joint_command('right', zero_angles, use_radians=True)

def test_speed_variations():
    """测试不同速度"""
    logger.info("=== 速度变化测试 ===")
    
    if not diagnose_connection():
        return
    
    test_angles = [5.0, 0, 0, 0, 0, 0, 0]
    speeds = [0.05, 0.1, 0.2, 0.5, 0.8]
    
    for speed in speeds:
        logger.info(f"测试速度: {speed}")
        
        # 移动
        result = ros_handler.send_direct_joint_command('right', test_angles, speed=speed)
        logger.info(f"速度 {speed} 移动结果: {result}")
        time.sleep(3)
        
        # 归位
        ros_handler.send_direct_joint_command('right', [0, 0, 0, 0, 0, 0, 0], speed=speed)
        time.sleep(3)

def test_trajectory_modes():
    """测试轨迹模式"""
    logger.info("=== 轨迹模式测试 ===")
    
    if not diagnose_connection():
        return
    
    test_angles = [3.0, 0, 0, 0, 0, 0, 0]
    
    # 测试 trajectory_connect = 0
    logger.info("测试 trajectory_connect = 0")
    result1 = ros_handler.send_direct_joint_command('right', test_angles, trajectory_connect=0)
    logger.info(f"独立模式结果: {result1}")
    time.sleep(5)
    
    # 归位
    ros_handler.send_direct_joint_command('right', [0, 0, 0, 0, 0, 0, 0], trajectory_connect=0)
    time.sleep(3)
    
    # 测试 trajectory_connect = 1
    logger.info("测试 trajectory_connect = 1")
    result2 = ros_handler.send_direct_joint_command('right', test_angles, trajectory_connect=1)
    logger.info(f"连接模式结果: {result2}")
    time.sleep(5)
    
    # 归位
    ros_handler.send_direct_joint_command('right', [0, 0, 0, 0, 0, 0, 0], trajectory_connect=1)

def test_topic_publishing():
    """直接测试话题发布"""
    logger.info("=== 直接话题发布测试 ===")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS客户端未连接")
        return
    
    # 创建临时发布器
    topic_name = '/r_arm/rm_driver/MoveJ_Cmd'
    msg_type = 'dual_arm_msgs/MoveJ'
    
    logger.info(f"创建临时发布器: {topic_name}, {msg_type}")
    temp_pub = roslibpy.Topic(ros_handler.ros_client, topic_name, msg_type)
    
    # 测试不同的消息格式
    test_messages = [
        {
            'name': '度数_低速',
            'data': {
                'joint': [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed': 0.1,
                'trajectory_connect': 0
            }
        },
        {
            'name': '弧度_低速',
            'data': {
                'joint': [math.radians(2.0), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed': 0.1,
                'trajectory_connect': 0
            }
        },
        {
            'name': '度数_正常速度',
            'data': {
                'joint': [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed': 0.5,
                'trajectory_connect': 1
            }
        }
    ]
    
    for test in test_messages:
        logger.info(f"测试: {test['name']}")
        logger.info(f"消息: {test['data']}")
        
        try:
            message = roslibpy.Message(test['data'])
            temp_pub.publish(message)
            logger.info("消息发送成功")
        except Exception as e:
            logger.error(f"消息发送失败: {e}")
        
        time.sleep(8)  # 等待更长时间观察
        
        # 发送归位指令
        try:
            zero_msg = roslibpy.Message({
                'joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed': 0.5,
                'trajectory_connect': 0
            })
            temp_pub.publish(zero_msg)
            logger.info("归位指令发送")
        except Exception as e:
            logger.error(f"归位指令失败: {e}")
        
        time.sleep(5)

def main():
    """主诊断函数"""
    logger.info("启动机器人不执行指令综合诊断...")
    
    # 连接ROS
    ros_handler.try_connect_ros()
    time.sleep(3)
    
    # 运行诊断
    diagnose_connection()
    diagnose_message_format()
    
    # 用户确认
    response = input("是否继续进行运动测试？机器人会有小幅度运动 (y/n): ")
    if response.lower() == 'y':
        test_basic_movement()
        test_speed_variations()
        test_trajectory_modes()
        test_topic_publishing()
    
    logger.info("=== 诊断总结 ===")
    logger.info("请观察哪种测试使机器人实际移动:")
    logger.info("1. 如果度数测试有效，继续使用度数")
    logger.info("2. 如果弧度测试有效，需要启用弧度转换")
    logger.info("3. 如果低速测试有效，降低默认速度")
    logger.info("4. 如果特定trajectory_connect有效，调整配置")
    logger.info("5. 如果都无效，可能是机器人端配置问题")

if __name__ == "__main__":
    main()
