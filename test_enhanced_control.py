#!/usr/bin/env python3
"""
增强版测试脚本：验证直接关节控制的连续指令问题修复
"""

import sys
import os
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger

def test_continuous_commands():
    """测试连续关节控制指令"""
    logger.info("=== 测试连续关节控制指令 ===")
    
    # 等待ROS连接
    logger.info("等待ROS连接...")
    time.sleep(3)
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    # 定义测试角度序列
    test_sequences = [
        [0, 0, 0, 0, 0, 0, 0],      # 归位
        [10, 0, 0, 0, 0, 0, 0],     # 移动第1关节
        [10, 10, 0, 0, 0, 0, 0],    # 移动第2关节
        [10, 10, 10, 0, 0, 0, 0],   # 移动第3关节
        [0, 0, 0, 0, 0, 0, 0],      # 回到归位
    ]
    
    # 测试右臂
    logger.info("开始测试右臂连续指令...")
    for i, angles in enumerate(test_sequences):
        logger.info(f"发送第 {i+1} 个指令: {angles}")
        result = ros_handler.send_right_arm_direct_command(angles)
        logger.info(f"指令 {i+1} 结果: {result}")
        time.sleep(1)  # 等待1秒
    
    time.sleep(2)
    
    # 测试左臂
    logger.info("开始测试左臂连续指令...")
    for i, angles in enumerate(test_sequences):
        logger.info(f"发送第 {i+1} 个指令: {angles}")
        result = ros_handler.send_left_arm_direct_command(angles)
        logger.info(f"指令 {i+1} 结果: {result}")
        time.sleep(1)  # 等待1秒
    
    logger.info("=== 连续指令测试完成 ===")

def test_different_modes():
    """测试不同的控制模式"""
    logger.info("=== 测试不同控制模式 ===")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    test_angles = [5, 5, 5, 5, 5, 5, 5]
    
    # 测试独立模式
    logger.info("测试独立指令模式...")
    result = ros_handler.send_direct_joint_command_with_mode('right', test_angles, mode='independent')
    logger.info(f"独立模式结果: {result}")
    time.sleep(2)
    
    # 测试连续模式
    logger.info("测试连续轨迹模式...")
    result = ros_handler.send_direct_joint_command_with_mode('right', [0, 0, 0, 0, 0, 0, 0], mode='continuous')
    logger.info(f"连续模式结果: {result}")
    time.sleep(2)
    
    # 测试正常模式
    logger.info("测试正常模式...")
    result = ros_handler.send_direct_joint_command_with_mode('right', test_angles, mode='normal')
    logger.info(f"正常模式结果: {result}")
    
    logger.info("=== 模式测试完成 ===")

def test_rapid_commands():
    """测试快速连续指令（验证时间间隔限制）"""
    logger.info("=== 测试快速连续指令 ===")
    
    if not ros_handler.ros_client or not ros_handler.ros_client.is_connected:
        logger.error("ROS未连接，无法进行测试")
        return
    
    logger.info("发送快速连续指令...")
    start_time = time.time()
    
    for i in range(5):
        angles = [i * 2, 0, 0, 0, 0, 0, 0]  # 每次稍微改变角度
        logger.info(f"快速指令 {i+1}: {angles}")
        result = ros_handler.send_right_arm_direct_command(angles)
        logger.info(f"快速指令 {i+1} 结果: {result}")
        # 不添加延迟，测试内部时间间隔控制
    
    end_time = time.time()
    total_time = end_time - start_time
    logger.info(f"5个快速指令总耗时: {total_time:.3f}秒")
    
    logger.info("=== 快速指令测试完成 ===")

def main():
    """主测试函数"""
    logger.info("启动增强版直接控制测试...")
    
    # 连接ROS
    ros_handler.try_connect_ros()
    
    # 运行各种测试
    test_continuous_commands()
    test_different_modes()
    test_rapid_commands()
    
    logger.info("所有测试完成！")

if __name__ == "__main__":
    main()
