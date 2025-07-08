#!/usr/bin/env python3
"""
ROS话题检查工具
用于诊断ROS Bridge连接和话题可用性问题
"""

import roslibpy
import time
import sys
from config import *

def check_ros_topics():
    """检查ROS话题的可用性"""
    print("🔍 ROS话题检查工具")
    print("=" * 60)
    
    try:
        # 连接到ROS Bridge
        print(f"📡 连接到ROS Bridge: {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
        client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
        client.run()
        
        if not client.is_connected:
            print("❌ ROS Bridge连接失败")
            return False
            
        print("✅ ROS Bridge连接成功")
        
        # 检查话题列表
        topics_to_check = [
            ("左臂状态", LEFT_ARM_STATE_TOPIC, ARM_STATE_MSG_TYPE),
            ("右臂状态", RIGHT_ARM_STATE_TOPIC, ARM_STATE_MSG_TYPE), 
            ("头部伺服状态", HEAD_SERVO_STATE_TOPIC, HEAD_SERVO_STATE_MSG_TYPE),
            ("左臂控制", LEFT_ARM_CMD_TOPIC, ARM_MSG_TYPE),
            ("右臂控制", RIGHT_ARM_CMD_TOPIC, ARM_MSG_TYPE),
            ("左手控制", LEFT_HAND_CMD_TOPIC, HAND_MSG_TYPE),
            ("右手控制", RIGHT_HAND_CMD_TOPIC, HAND_MSG_TYPE),
            ("导航控制", NAV_CMD_TOPIC, NAV_MSG_TYPE),
        ]
        
        print("\n📋 检查话题可用性:")
        print("-" * 60)
        
        available_topics = []
        unavailable_topics = []
        
        for name, topic, msg_type in topics_to_check:
            print(f"🔸 {name:12} | {topic:35} | {msg_type}")
            
            try:
                # 尝试创建话题对象
                test_topic = roslibpy.Topic(client, topic, msg_type)
                
                # 短暂等待以检查连接
                time.sleep(0.5)
                
                # 注意：is_subscribed只有在实际订阅后才有意义
                # 这里我们只是检查话题是否能创建
                available_topics.append((name, topic, msg_type))
                print(f"   ✅ 话题可创建")
                
                test_topic.unadvertise()  # 清理
                
            except Exception as e:
                unavailable_topics.append((name, topic, msg_type, str(e)))
                print(f"   ❌ 创建失败: {e}")
        
        # 汇总报告
        print("\n" + "=" * 60)
        print("📊 检查结果汇总:")
        print("=" * 60)
        
        print(f"✅ 可用话题 ({len(available_topics)}):")
        for name, topic, msg_type in available_topics:
            print(f"   • {name}: {topic}")
            
        if unavailable_topics:
            print(f"\n❌ 不可用话题 ({len(unavailable_topics)}):")
            for name, topic, msg_type, error in unavailable_topics:
                print(f"   • {name}: {topic}")
                print(f"     错误: {error}")
        
        # 测试简单的发布
        print(f"\n🧪 测试基本发布功能:")
        print("-" * 60)
        
        try:
            # 测试左臂控制话题
            test_pub = roslibpy.Topic(client, LEFT_ARM_CMD_TOPIC, ARM_MSG_TYPE)
            test_message = roslibpy.Message({
                'joint': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed': 0.1,
                'trajectory_connect': 0
            })
            
            print(f"📤 尝试发布到 {LEFT_ARM_CMD_TOPIC}")
            test_pub.publish(test_message)
            print("✅ 发布成功")
            
            test_pub.unadvertise()
            
        except Exception as e:
            print(f"❌ 发布测试失败: {e}")
        
        # 推荐配置
        print(f"\n💡 建议:")
        print("-" * 60)
        
        if len(unavailable_topics) > 0:
            print("⚠️  部分话题不可用，但这不会阻止基本控制功能")
            print("   状态监控可能不可用，但机械臂控制应该仍然工作")
            
        print("✨ 要启用控制功能，请确保:")
        print("   1. ROS机械臂驱动程序正在运行")
        print("   2. 机械臂控制器已启动")
        print("   3. MoveIt!节点正在运行（如果使用MoveIt!控制）")
        
        client.terminate()
        return True
        
    except Exception as e:
        print(f"❌ 检查过程中发生错误: {e}")
        return False

def check_moveit_action_server():
    """检查MoveIt! Action Server"""
    print("\n🎯 检查MoveIt! Action Server:")
    print("-" * 60)
    
    try:
        client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
        client.run()
        
        if not client.is_connected:
            print("❌ ROS Bridge连接失败")
            return False
            
        # 创建Action Client
        action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        
        print(f"🎮 Action Server: {MOVE_GROUP_ACTION_NAME}")
        print(f"📋 Action Type: {MOVE_GROUP_ACTION_TYPE}")
        
        # 等待一段时间看是否能连接
        time.sleep(2)
        
        print("✅ MoveIt! Action Client创建成功")
        print("   （具体可用性需要实际发送目标来测试）")
        
        client.terminate()
        return True
        
    except Exception as e:
        print(f"❌ MoveIt! Action Server检查失败: {e}")
        return False

def main():
    """主函数"""
    print("🤖 ROS机械臂控制系统诊断工具")
    print("=" * 60)
    print(f"目标地址: {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
    print("=" * 60)
    
    # 基本连接检查
    success = check_ros_topics()
    
    if success:
        # MoveIt检查
        check_moveit_action_server()
        
        print("\n" + "=" * 60)
        print("🎉 诊断完成！")
        print("💡 如果控制仍然不工作，请检查:")
        print("   1. 机械臂驱动程序是否运行")
        print("   2. ROS Bridge版本兼容性")
        print("   3. 网络连接稳定性")
        print("   4. 防火墙设置")
    
    return success

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n👋 用户中断，退出程序")
        sys.exit(0)
    except Exception as e:
        print(f"\n💥 程序异常退出: {e}")
        sys.exit(1)
