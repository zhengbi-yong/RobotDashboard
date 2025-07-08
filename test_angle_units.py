#!/usr/bin/env python3
"""
机器人角度单位测试脚本
测试度数 vs 弧度的问题
"""

import sys
import os
import math
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger

def test_angle_units():
    """测试角度单位问题"""
    logger.info("=== 机器人角度单位测试 ===")
    
    # 等待ROS连接
    logger.info("等待ROS连接...")
    time.sleep(3)
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    # 小角度测试（安全）
    small_angle_deg = 5.0  # 5度
    small_angle_rad = math.radians(small_angle_deg)  # 约0.087弧度
    
    logger.info(f"测试小角度: {small_angle_deg}度 = {small_angle_rad:.4f}弧度")
    
    # 测试1: 发送度数（当前实现）
    logger.info("测试1: 发送度数值...")
    angles_deg = [small_angle_deg, 0, 0, 0, 0, 0, 0]
    result = ros_handler.send_right_arm_direct_command(angles_deg)
    logger.info(f"度数测试结果: {result}")
    logger.info("请观察机器人是否移动，等待10秒...")
    time.sleep(10)
    
    # 测试2: 发送弧度值（作为度数发送）
    logger.info("测试2: 发送弧度值（作为度数）...")
    angles_rad_as_deg = [small_angle_rad, 0, 0, 0, 0, 0, 0]
    result = ros_handler.send_right_arm_direct_command(angles_rad_as_deg)
    logger.info(f"弧度值测试结果: {result}")
    logger.info("请观察机器人是否移动，等待10秒...")
    time.sleep(10)
    
    # 归位
    logger.info("归位测试...")
    zero_angles = [0, 0, 0, 0, 0, 0, 0]
    result = ros_handler.send_right_arm_direct_command(zero_angles)
    logger.info(f"归位结果: {result}")
    
    logger.info("=== 角度单位测试完成 ===")
    logger.info("请根据机器人实际行为判断哪种单位是正确的")

def test_different_speeds():
    """测试不同的速度参数"""
    logger.info("=== 测试不同速度参数 ===")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    test_angles = [10, 0, 0, 0, 0, 0, 0]  # 小角度测试
    speeds = [0.1, 0.3, 0.5, 0.8]
    
    for speed in speeds:
        logger.info(f"测试速度: {speed}")
        result = ros_handler.send_direct_joint_command('right', test_angles, speed=speed)
        logger.info(f"速度 {speed} 测试结果: {result}")
        time.sleep(5)  # 等待执行
    
    # 归位
    zero_angles = [0, 0, 0, 0, 0, 0, 0]
    ros_handler.send_right_arm_direct_command(zero_angles)
    
    logger.info("=== 速度测试完成 ===")

def test_trajectory_connect_modes():
    """测试不同的trajectory_connect模式"""
    logger.info("=== 测试trajectory_connect模式 ===")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    test_angles = [5, 0, 0, 0, 0, 0, 0]
    
    # 测试 trajectory_connect = 0
    logger.info("测试 trajectory_connect = 0 (独立指令)")
    result = ros_handler.send_direct_joint_command('right', test_angles, trajectory_connect=0)
    logger.info(f"独立指令结果: {result}")
    time.sleep(3)
    
    # 测试 trajectory_connect = 1
    logger.info("测试 trajectory_connect = 1 (连接轨迹)")
    result = ros_handler.send_direct_joint_command('right', [0, 0, 0, 0, 0, 0, 0], trajectory_connect=1)
    logger.info(f"连接轨迹结果: {result}")
    time.sleep(3)
    
    logger.info("=== trajectory_connect模式测试完成 ===")

def main():
    """主测试函数"""
    logger.info("启动机器人角度单位调试测试...")
    
    # 连接ROS
    ros_handler.try_connect_ros()
    
    # 进行各种测试
    test_angle_units()
    test_different_speeds()
    test_trajectory_connect_modes()
    
    logger.info("所有调试测试完成！")
    logger.info("请根据机器人的实际反应来判断:")
    logger.info("1. 正确的角度单位（度数 vs 弧度）")
    logger.info("2. 合适的速度参数")
    logger.info("3. 合适的trajectory_connect设置")

if __name__ == "__main__":
    main()
