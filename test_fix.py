#!/usr/bin/env python3
"""
测试脚本：验证修复后的发布器初始化
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger

def test_publisher_fix():
    """测试发布器修复"""
    logger.info("=== 测试发布器修复 ===")
    
    # 检查初始状态
    logger.info(f"初始状态 - left_arm_pub: {ros_handler.left_arm_pub}")
    logger.info(f"初始状态 - right_arm_pub: {ros_handler.right_arm_pub}")
    logger.info(f"初始状态 - ros_client: {ros_handler.ros_client}")
    logger.info(f"初始状态 - ros_setup_done: {ros_handler.ros_setup_done}")
    
    # 尝试连接
    logger.info("尝试连接ROS...")
    ros_handler.try_connect_ros()
    
    # 等待一些时间让连接完成
    import time
    time.sleep(3)
    
    # 检查连接后状态
    logger.info(f"连接后 - ros_client: {ros_handler.ros_client}")
    logger.info(f"连接后 - ros_client.is_connected: {ros_handler.ros_client.is_connected if ros_handler.ros_client else 'None'}")
    logger.info(f"连接后 - left_arm_pub: {ros_handler.left_arm_pub}")
    logger.info(f"连接后 - right_arm_pub: {ros_handler.right_arm_pub}")
    
    # 测试直接控制函数
    if ros_handler.ros_client and ros_handler.ros_client.is_connected:
        logger.info("测试直接关节控制...")
        test_joints = [0, 0, 0, 0, 0, 0, 0]  # 7个关节角度
        
        # 测试右臂控制
        result = ros_handler.send_right_arm_direct_command(test_joints)
        logger.info(f"右臂直接控制结果: {result}")
        
        # 测试左臂控制
        result = ros_handler.send_left_arm_direct_command(test_joints)
        logger.info(f"左臂直接控制结果: {result}")
    else:
        logger.warning("ROS未连接，跳过控制测试")
    
    logger.info("=== 测试完成 ===")

if __name__ == "__main__":
    test_publisher_fix()
