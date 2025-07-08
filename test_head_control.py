#!/usr/bin/env python3
"""
头部控制测试脚本
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger
import config

def test_head_control():
    """测试头部控制功能"""
    logger.info("=== 头部控制测试 ===")
    
    # 连接ROS
    logger.info("连接ROS...")
    ros_handler.try_connect_ros()
    time.sleep(3)
    
    # 检查连接状态
    logger.info(f"ROS客户端: {ros_handler.ros_client}")
    if ros_handler.ros_client:
        logger.info(f"是否连接: {ros_handler.ros_client.is_connected}")
    logger.info(f"头部发布器: {ros_handler.head_servo_pub}")
    logger.info(f"ROS设置完成: {ros_handler.ros_setup_done}")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    if not ros_handler.head_servo_pub:
        logger.error("头部发布器未初始化")
        return
    
    # 测试头部运动
    logger.info("测试头部运动...")
    
    # 测试序列
    test_sequences = [
        {"name": "中性位置", "tilt": 500, "pan": 500},
        {"name": "上看", "tilt": 400, "pan": 500},
        {"name": "下看", "tilt": 600, "pan": 500},
        {"name": "左看", "tilt": 500, "pan": 300},
        {"name": "右看", "tilt": 500, "pan": 700},
        {"name": "回到中性", "tilt": 500, "pan": 500},
    ]
    
    for i, seq in enumerate(test_sequences):
        logger.info(f"执行序列 {i+1}: {seq['name']} (俯仰: {seq['tilt']}, 左右: {seq['pan']})")
        
        try:
            # 发送俯仰指令
            tilt_msg = {
                'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'],
                'angle': seq['tilt']
            }
            ros_handler.head_servo_pub.publish(roslibpy.Message(tilt_msg))
            logger.info(f"发送俯仰指令: {tilt_msg}")
            
            time.sleep(0.1)
            
            # 发送左右指令
            pan_msg = {
                'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 
                'angle': seq['pan']
            }
            ros_handler.head_servo_pub.publish(roslibpy.Message(pan_msg))
            logger.info(f"发送左右指令: {pan_msg}")
            
            logger.info(f"序列 {i+1} 完成")
            time.sleep(2)  # 等待运动完成
            
        except Exception as e:
            logger.error(f"序列 {i+1} 执行失败: {e}")
    
    logger.info("=== 头部控制测试完成 ===")

def test_connection_only():
    """仅测试连接状态"""
    logger.info("=== 连接状态测试 ===")
    
    ros_handler.try_connect_ros()
    time.sleep(3)
    
    logger.info(f"ROS客户端存在: {ros_handler.ros_client is not None}")
    if ros_handler.ros_client:
        logger.info(f"ROS连接状态: {ros_handler.ros_client.is_connected}")
    logger.info(f"ROS连接字符串: {ros_handler.ros_connection_status}")
    logger.info(f"ROS设置完成: {ros_handler.ros_setup_done}")
    logger.info(f"头部发布器: {ros_handler.head_servo_pub is not None}")
    logger.info(f"左臂发布器: {ros_handler.left_arm_pub is not None}")
    logger.info(f"右臂发布器: {ros_handler.right_arm_pub is not None}")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='头部控制测试')
    parser.add_argument('--connection-only', action='store_true', help='仅测试连接状态')
    parser.add_argument('--head-test', action='store_true', help='测试头部运动')
    
    args = parser.parse_args()
    
    if args.connection_only:
        test_connection_only()
    elif args.head_test:
        test_head_control()
    else:
        logger.info("使用选项:")
        logger.info("  --connection-only  : 仅测试连接状态")
        logger.info("  --head-test       : 测试头部运动")
        test_connection_only()

if __name__ == "__main__":
    main()
