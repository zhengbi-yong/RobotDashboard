#!/usr/bin/env python3
"""
说话指令API测试脚本
"""

import sys
import os
import time
import requests
import json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
from loguru import logger

def test_speak_api_direct():
    """直接测试说话API"""
    logger.info("=== 直接测试说话API ===")
    
    url = f"{config.ROBOT_ACTION_API_BASE_URL}{config.ROBOT_ACTION_ENDPOINTS['speak']}"
    data = {"text": "你好，这是一个测试消息"}
    headers = {'Content-Type': 'application/json'}
    
    logger.info(f"请求URL: {url}")
    logger.info(f"请求数据: {data}")
    logger.info(f"请求头: {headers}")
    
    try:
        logger.info("发送请求...")
        response = requests.post(url, headers=headers, json=data, timeout=10)
        
        logger.info(f"响应状态码: {response.status_code}")
        logger.info(f"响应头: {dict(response.headers)}")
        
        try:
            response_json = response.json()
            logger.info(f"响应JSON: {response_json}")
        except:
            logger.info(f"响应文本: {response.text}")
        
        if response.status_code == 200:
            logger.success("说话API测试成功！")
            return True
        else:
            logger.error(f"说话API测试失败，状态码: {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError as e:
        logger.error(f"连接错误: {e}")
        logger.error("可能原因: 1. 网络连接问题 2. 服务器地址错误 3. 服务器未启动")
        return False
    except requests.exceptions.Timeout as e:
        logger.error(f"请求超时: {e}")
        return False
    except requests.exceptions.RequestException as e:
        logger.error(f"请求异常: {e}")
        return False
    except Exception as e:
        logger.error(f"未知错误: {e}")
        return False

def test_speak_via_handler():
    """通过handler模块测试说话功能"""
    logger.info("=== 通过handler模块测试说话功能 ===")
    
    try:
        from ros_comms import handler as ros_handler
        
        success = ros_handler.send_speak_command("这是通过handler模块的测试消息")
        
        if success:
            logger.success("通过handler模块的说话测试成功！")
        else:
            logger.error("通过handler模块的说话测试失败！")
        
        return success
        
    except Exception as e:
        logger.error(f"handler模块测试异常: {e}")
        return False

def test_network_connectivity():
    """测试网络连通性"""
    logger.info("=== 测试网络连通性 ===")
    
    base_url = config.ROBOT_ACTION_API_BASE_URL
    logger.info(f"测试连接到: {base_url}")
    
    try:
        # 尝试连接到基础URL
        response = requests.get(base_url, timeout=5)
        logger.info(f"基础URL连接成功，状态码: {response.status_code}")
        return True
    except requests.exceptions.ConnectionError:
        logger.error(f"无法连接到 {base_url}")
        logger.error("请检查: 1. 网络连接 2. 服务器地址是否正确 3. 服务器是否运行")
        return False
    except Exception as e:
        logger.error(f"网络测试异常: {e}")
        return False

if __name__ == "__main__":
    logger.info("开始说话指令测试...")
    
    # 1. 测试网络连通性
    if not test_network_connectivity():
        logger.error("网络连通性测试失败，停止后续测试")
        exit(1)
    
    # 2. 直接测试API
    direct_success = test_speak_api_direct()
    
    # 3. 通过handler测试
    handler_success = test_speak_via_handler()
    
    # 总结
    logger.info("=== 测试总结 ===")
    logger.info(f"网络连通性: ✓")
    logger.info(f"直接API测试: {'✓' if direct_success else '✗'}")
    logger.info(f"Handler模块测试: {'✓' if handler_success else '✗'}")
    
    if direct_success and handler_success:
        logger.success("所有测试通过！说话功能正常。")
    elif direct_success:
        logger.warning("直接API测试成功，但Handler模块测试失败。检查Handler实现。")
    else:
        logger.error("API测试失败。请检查服务器配置和网络连接。")
