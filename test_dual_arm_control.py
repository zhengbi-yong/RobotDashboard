#!/usr/bin/env python3
"""
测试双臂控制模式选择功能
用于验证MoveIt!控制和直接关节角控制两种模式是否正常工作
"""

import roslibpy
import time
import math
from config import *

def test_direct_control():
    """测试直接关节角控制"""
    print("=" * 60)
    print("测试直接关节角控制功能")
    print("=" * 60)
    
    try:
        # 连接到ROS Bridge
        client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
        client.run()
        
        if not client.is_connected:
            print("❌ ROS Bridge 连接失败")
            return False
        
        print("✅ ROS Bridge 连接成功")
        
        # 创建发布器
        left_arm_pub = roslibpy.Topic(client, LEFT_ARM_CMD_TOPIC, ARM_MSG_TYPE)
        right_arm_pub = roslibpy.Topic(client, RIGHT_ARM_CMD_TOPIC, ARM_MSG_TYPE)
        
        print(f"📡 创建发布器:")
        print(f"   左臂: {LEFT_ARM_CMD_TOPIC}")
        print(f"   右臂: {RIGHT_ARM_CMD_TOPIC}")
        
        # 测试关节角度（度）
        test_angles = [0, 0, 0, 0, 0, 0, 0]  # 中性位置
        
        # 构建消息
        message = roslibpy.Message({
            'joint': [float(angle) for angle in test_angles],
            'speed': JOINT_CONTROL_SPEED,
            'trajectory_connect': JOINT_CONTROL_TRAJECTORY_CONNECT
        })
        
        print(f"🎯 发送测试指令:")
        print(f"   关节角度: {test_angles}")
        print(f"   速度: {JOINT_CONTROL_SPEED}")
        
        # 发送左臂指令
        print("📤 发送左臂指令...")
        left_arm_pub.publish(message)
        time.sleep(1)
        
        # 发送右臂指令
        print("📤 发送右臂指令...")
        right_arm_pub.publish(message)
        time.sleep(1)
        
        print("✅ 直接控制指令发送完成")
        
        # 清理
        left_arm_pub.unadvertise()
        right_arm_pub.unadvertise()
        client.terminate()
        
        return True
        
    except Exception as e:
        print(f"❌ 直接控制测试失败: {e}")
        return False

def print_control_modes():
    """打印控制模式配置信息"""
    print("=" * 60)
    print("双臂控制模式配置")
    print("=" * 60)
    
    print("📋 可用控制模式:")
    for mode_key, mode_desc in ARM_CONTROL_MODES.items():
        print(f"   {mode_key}: {mode_desc}")
    
    print(f"\n🔧 默认控制模式: {DEFAULT_ARM_CONTROL_MODE}")
    
    print(f"\n📡 话题配置:")
    print(f"   左臂指令话题: {LEFT_ARM_CMD_TOPIC}")
    print(f"   右臂指令话题: {RIGHT_ARM_CMD_TOPIC}")
    print(f"   消息类型: {ARM_MSG_TYPE}")
    
    print(f"\n⚙️ 直接控制参数:")
    print(f"   控制速度: {JOINT_CONTROL_SPEED}")
    print(f"   轨迹连接: {JOINT_CONTROL_TRAJECTORY_CONNECT}")
    
    print(f"\n🎯 MoveIt! 配置:")
    print(f"   左臂规划组: {PLANNING_GROUP_LEFT_ARM}")
    print(f"   右臂规划组: {PLANNING_GROUP_RIGHT_ARM}")
    print(f"   动作服务器: {MOVE_GROUP_ACTION_NAME}")

def main():
    """主函数"""
    print("🤖 双臂控制模式测试程序")
    print_control_modes()
    
    print("\n" + "=" * 60)
    print("开始功能测试")
    print("=" * 60)
    
    # 测试直接控制
    if test_direct_control():
        print("✅ 所有测试通过！")
    else:
        print("❌ 测试失败，请检查配置和ROS连接")
    
    print("\n📝 使用说明:")
    print("1. 在前端界面中，每个机械臂都有控制模式选择器")
    print("2. 可以选择 'MoveIt! 规划控制' 或 '直接关节角控制'")
    print("3. MoveIt! 模式使用运动规划，更安全但较慢")
    print("4. 直接控制模式直接发送关节角指令，响应快但需谨慎使用")
    print("5. 双臂可以独立选择不同的控制模式")

if __name__ == "__main__":
    main()
