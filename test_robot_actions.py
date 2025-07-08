#!/usr/bin/env python3
"""
机器人动作功能测试脚本
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_comms import handler as ros_handler
from loguru import logger
import config

def test_robot_actions():
    """测试机器人动作功能"""
    logger.info("=== 机器人动作功能测试 ===")
    
    # 测试语音识别设置
    logger.info("测试语音识别设置...")
    result = ros_handler.send_asr_transcript("这是一个测试语音识别文本")
    logger.info(f"语音识别设置结果: {result}")
    
    # 测试直接说话
    logger.info("测试直接说话...")
    result = ros_handler.send_speak_command("你好，我是机器人小千")
    logger.info(f"直接说话结果: {result}")
    
    # 测试一些基本动作
    test_actions = ['woshou', 'huishou', 'zhaoshou']
    
    for action_id in test_actions:
        logger.info(f"测试动作: {action_id}")
        result = ros_handler.send_force_action_command(action_id)
        logger.info(f"动作 {action_id} 结果: {result}")
        
        import time
        time.sleep(2)  # 等待动作完成
    
    logger.info("=== 测试完成 ===")

def list_available_actions():
    """列出所有可用动作"""
    logger.info("=== 可用动作列表 ===")
    
    categories = {}
    for action in config.ROBOT_ACTIONS:
        category = action['category']
        if category not in categories:
            categories[category] = []
        categories[category].append(action)
    
    for category, actions in categories.items():
        logger.info(f"\n{category}动作:")
        for action in actions:
            logger.info(f"  {action['id']}: {action['name']}")

def test_specific_action(action_id):
    """测试特定动作"""
    logger.info(f"测试特定动作: {action_id}")
    result = ros_handler.send_force_action_command(action_id)
    logger.info(f"结果: {result}")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='机器人动作功能测试')
    parser.add_argument('--list', action='store_true', help='列出所有可用动作')
    parser.add_argument('--test', metavar='ACTION_ID', help='测试特定动作')
    parser.add_argument('--all', action='store_true', help='运行所有测试')
    
    args = parser.parse_args()
    
    if args.list:
        list_available_actions()
    elif args.test:
        test_specific_action(args.test)
    elif args.all:
        test_robot_actions()
    else:
        logger.info("使用 --help 查看可用选项")
        logger.info("快速使用:")
        logger.info("  python test_robot_actions.py --list     # 列出所有动作")
        logger.info("  python test_robot_actions.py --test woshou  # 测试握手动作")
        logger.info("  python test_robot_actions.py --all      # 运行所有测试")

if __name__ == "__main__":
    main()
