#!/usr/bin/env python3
"""
演示握手动作控制流程的测试脚本
"""

import sys
import os
import time
import requests
import json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
from loguru import logger

def demonstrate_handshake_flow():
    """演示握手动作的完整控制流程"""
    
    logger.info("=" * 60)
    logger.info("🤖 机器人握手动作控制流程演示")
    logger.info("=" * 60)
    
    # 步骤1：模拟用户点击握手按钮
    logger.info("📱 步骤1: 用户在界面点击'握手'按钮")
    logger.info("   - 按钮ID: action-woshou")
    logger.info("   - 显示文字: 握手")
    logger.info("   - 动作类别: 交互")
    
    # 步骤2：UI回调处理
    logger.info("\n🔄 步骤2: Dash UI回调函数处理")
    logger.info("   - 检测到按钮点击事件")
    logger.info("   - 提取动作ID: 'woshou'")
    logger.info("   - 查找动作名称: '握手'")
    
    # 步骤3：构建API请求
    logger.info("\n📡 步骤3: 构建HTTP API请求")
    
    # 从config.py读取配置
    base_url = config.ROBOT_ACTION_API_BASE_URL
    endpoint = config.ROBOT_ACTION_ENDPOINTS['force_command']
    full_url = f"{base_url}{endpoint}"
    
    request_data = {
        "force_command_str": "woshou",
        "is_force_input": True
    }
    
    logger.info(f"   - 目标URL: {full_url}")
    logger.info(f"   - 请求方法: POST")
    logger.info(f"   - 请求头: Content-Type: application/json")
    logger.info(f"   - 请求数据: {json.dumps(request_data, ensure_ascii=False, indent=6)}")
    
    # 步骤4：发送HTTP请求
    logger.info("\n🌐 步骤4: 发送HTTP请求到机器人服务器")
    
    try:
        headers = {'Content-Type': 'application/json'}
        
        logger.info(f"   - 正在连接到: {base_url}")
        logger.info("   - 发送握手动作指令...")
        
        # 实际发送请求
        response = requests.post(full_url, headers=headers, json=request_data, timeout=10)
        
        logger.info(f"   - 响应状态码: {response.status_code}")
        logger.info(f"   - 响应时间: {response.elapsed.total_seconds():.2f}秒")
        
        try:
            response_data = response.json()
            logger.info(f"   - 响应数据: {json.dumps(response_data, ensure_ascii=False, indent=6)}")
        except:
            logger.info(f"   - 响应文本: {response.text}")
        
        # 步骤5：处理响应结果
        logger.info("\n✅ 步骤5: 处理服务器响应")
        
        if response.status_code == 200:
            logger.success("   - ✅ 握手指令发送成功！")
            logger.info("   - 🤖 机器人开始执行握手动作")
            logger.info("   - 📱 前端显示: '已发送动作指令: 握手 (woshou)'")
        else:
            logger.error(f"   - ❌ 握手指令发送失败，状态码: {response.status_code}")
            logger.info("   - 📱 前端显示: '动作指令发送失败: 握手'")
            
    except requests.exceptions.ConnectionError:
        logger.error("   - ❌ 连接失败：无法连接到机器人服务器")
        logger.error(f"   - 检查服务器是否运行在 {base_url}")
        logger.info("   - 📱 前端显示: '动作指令发送失败: 握手'")
        
    except requests.exceptions.Timeout:
        logger.error("   - ⏰ 请求超时：服务器响应时间过长")
        logger.info("   - 📱 前端显示: '动作指令发送失败: 握手'")
        
    except Exception as e:
        logger.error(f"   - ❌ 未知错误: {e}")
        logger.info("   - 📱 前端显示: '动作指令发送失败: 握手'")
    
    # 步骤6：机器人端处理（理论流程）
    logger.info("\n🤖 步骤6: 机器人端处理（理论流程）")
    logger.info("   - 动作服务器接收HTTP请求")
    logger.info("   - 解析动作ID: 'woshou'")
    logger.info("   - 查找握手动作序列：")
    logger.info("     * 右臂抬起到合适角度")
    logger.info("     * 手部张开准备握手")
    logger.info("     * 前伸手臂")
    logger.info("     * 执行握手动作")
    logger.info("     * 恢复初始位置")
    logger.info("   - 将动作转换为关节角度指令")
    logger.info("   - 通过底层驱动控制电机运动")
    logger.info("   - 🎯 机器人完成握手动作！")
    
    logger.info("\n" + "=" * 60)
    logger.info("🎉 握手动作控制流程演示完成")
    logger.info("=" * 60)

def show_all_available_actions():
    """显示所有可用的动作列表"""
    logger.info("\n📋 所有可用动作列表：")
    
    categories = {}
    for action in config.ROBOT_ACTIONS:
        category = action['category']
        if category not in categories:
            categories[category] = []
        categories[category].append(action)
    
    for category, actions in categories.items():
        logger.info(f"\n  📂 {category}类动作：")
        for action in actions:
            logger.info(f"     - {action['name']} (ID: {action['id']})")

if __name__ == "__main__":
    # 演示握手流程
    demonstrate_handshake_flow()
    
    # 显示所有动作
    show_all_available_actions()
    
    logger.info(f"\n💡 提示：您可以修改此脚本来测试其他动作，只需更改动作ID即可。")
    logger.info(f"   例如：将 'woshou' 改为 'huishou' 来测试挥手动作。")
