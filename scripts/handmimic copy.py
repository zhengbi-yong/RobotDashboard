# -*- coding: utf-8 -*-
import roslibpy
import time
import traceback
import asyncio
import websockets
import numpy as np

# --- 1. 配置参数 (请根据您的环境修改) ---

# WebSocket 服务器地址
WEBSOCKET_URI = "ws://101.126.68.169:5024"

# ROS Bridge 服务器配置
ROS_BRIDGE_HOST = '192.168.0.105'  # !!! 必须修改为你的 ROS Master/rosbridge 的 IP 地址 !!!
ROS_BRIDGE_PORT = 9090
CONNECTION_TIMEOUT_SEC = 10

# ROS 话题和服务配置 (来自您的原代码)
# !!! 请再次确认这些名称与您的机器人配置完全匹配 !!!
ANGLE_COMMAND_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
ANGLE_MESSAGE_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 2. 手势到机器人角度映射的核心参数 (!!! 重要：需要微调校准 !!!) ---
# MediaPipe landmark indices for fingertips and wrist
FINGER_TIPS = [4, 8, 12, 16, 20]  # [拇指, 食指, 中指, 无名指, 小指] 的指尖索引
WRIST = 0

# 距离校准: [最小距离 (手握紧), 最大距离 (手张开)]
# 这个值极大地影响模仿效果，您需要通过实验来确定最佳值。
# 方法：运行脚本并打印出 `distances` 数组，分别在完全握拳和完全张开手时记录下每个手指的距离值。
# 单位是米 (来自MediaPipe的世界坐标)
# 格式: { 'thumb': [min, max], 'index': [min, max], ... }
MIN_MAX_DISTANCES = {
    'thumb': [0.05, 0.15],   # 拇指到手腕的距离范围 (示例值)
    'index': [0.06, 0.18],   # 食指到手腕的距离范围 (示例值)
    'middle': [0.06, 0.20],  # 中指到手腕的距离范围 (示例值)
    'ring': [0.05, 0.19],    # 无名指到手腕的距离范围 (示例值)
    'pinky': [0.04, 0.17]    # 小指到手腕的距离范围 (示例值)
}
FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']

# --- 3. 辅助函数 ---

def parse_hand_data(data_string):
    """从WebSocket接收的原始字符串中解析出左手的世界坐标"""
    try:
        # 数据以 "*||*" 分隔世界坐标和归一化坐标
        world_data_part = data_string.split('*||*')[0]
        
        # 我们只关心左手 'Left' 的数据
        if not world_data_part.strip().startswith('Left'):
            return None

        lines = world_data_part.strip().split('\n')
        landmarks = {}
        # 从第二行开始解析 (第一行是 'Left')
        for line in lines[1:]:
            parts = line.split('|')
            if len(parts) == 4:
                idx = int(parts[0])
                # 将坐标转换为浮点数
                coords = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
                landmarks[idx] = coords
        
        # 必须包含所有21个关键点
        if len(landmarks) == 21:
            return landmarks
        else:
            print(f"警告: 解析到的关键点数量不足 ({len(landmarks)}/21)，跳过此帧。")
            return None

    except Exception as e:
        # print(f"数据解析出错: {e}\n原始数据: '{data_string[:100]}...'")
        return None

def calculate_robot_angles(landmarks):
    """根据手部关键点计算出机器人的6个目标角度 (0-1000)"""
    if not landmarks or WRIST not in landmarks:
        return None

    wrist_pos = landmarks[WRIST]
    
    # 计算每个指尖到手腕的距离
    distances = []
    for tip_idx in FINGER_TIPS:
        if tip_idx in landmarks:
            dist = np.linalg.norm(landmarks[tip_idx] - wrist_pos)
            distances.append(dist)
        else:
            return None # 如果缺少关键点则无法计算
    
    # 打印实时距离，用于校准 MIN_MAX_DISTANCES
    # print(f"实时距离 [拇,食,中,无,小]: [{distances[0]:.3f}, {distances[1]:.3f}, {distances[2]:.3f}, {distances[3]:.3f}, {distances[4]:.3f}]")

    # 将距离映射到 0-1 的弯曲比例 (0=握紧, 1=张开)
    curl_ratios = []
    for i, dist in enumerate(distances):
        finger_name = FINGER_NAMES[i]
        min_d, max_d = MIN_MAX_DISTANCES[finger_name]
        
        # 线性映射公式
        ratio = (dist - min_d) / (max_d - min_d)
        
        # 限制在 0.0 到 1.0 之间，防止超出校准范围
        ratio = max(0.0, min(1.0, ratio))
        curl_ratios.append(ratio)

    # 将弯曲比例 (0=握紧, 1=张开) 转换为机器人角度 (0=握紧, 1000=张开)
    # 注意：机器人指令是“张开度”，所以我们用 curl_ratio 直接乘以 1000
    # 如果指令是“握紧度”，则需要用 (1.0 - ratio)
    robot_angles = [int(ratio * 1000) for ratio in curl_ratios]

    # --- 处理第6个关节 ---
    # 您的机器人需要6个值，但我们只计算了5个手指。
    # 第6个值如何处理取决于您的机器人。常见的可能是：
    # 1. 拇指的旋转/内收：这需要更复杂的角度计算。
    # 2. 一个预留或不用的值：可以设为固定值，如500（半开）。
    # 3. 手掌的整体开合：可以取所有手指的平均值。
    # 这里我们采用最简单的方案：设为一个固定中间值 500。
    # !!! 您可以根据需要修改此逻辑 !!!
    sixth_joint_value = 500
    robot_angles.append(sixth_joint_value)
    
    return robot_angles


# --- 4. 主程序 ---

async def main():
    """主函数，处理ROS连接和WebSocket数据流"""
    
    # 初始化 ROS 客户端
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_publisher = None

    try:
        # --- 连接 ROS ---
        print(f"正在连接到 rosbridge服务器: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
        # 使用带超时的非阻塞连接
        ros_client.run()
        start_time = time.time()
        while not ros_client.is_connected:
            await asyncio.sleep(0.1)
            if time.time() - start_time > CONNECTION_TIMEOUT_SEC:
                raise TimeoutError(f"连接 ROS 超时 ({CONNECTION_TIMEOUT_SEC} 秒)")
        print("成功连接到 rosbridge 服务器！")

        # 创建 ROS 发布者
        ros_publisher = roslibpy.Topic(ros_client, ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        print(f"已创建发布者, 话题: '{ANGLE_COMMAND_TOPIC}'")

        # --- 连接 WebSocket ---
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            print(f"成功连接到手势识别 WebSocket: {WEBSOCKET_URI}")
            print("\n--- 开始接收手势数据并控制机器人 ---")
            
            # 持续接收消息
            async for message_str in websocket:
                # 1. 解析数据
                landmarks = parse_hand_data(message_str)
                if not landmarks:
                    continue # 如果数据无效或不是左手，则跳过

                # 2. 计算机器人角度
                target_angles = calculate_robot_angles(landmarks)
                if not target_angles:
                    continue # 如果计算失败，则跳过

                # 3. 创建并发布 ROS 消息
                ros_message = roslibpy.Message({'hand_angle': target_angles})
                ros_publisher.publish(ros_message)
                
                # 打印发送的指令，便于调试
                print(f"已发送指令: {target_angles}", end='\r')


    except TimeoutError as e:
        print(f"\n错误: {e}")
        print("请检查：\n1. rosbridge 是否正在运行 (roslaunch rosbridge_server rosbridge_websocket.launch)？")
        print(f"2. IP 地址 '{ROS_BRIDGE_HOST}' 是否正确且可以访问？")
        print("3. 防火墙是否允许端口 9090？")
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"\n错误: 与手势识别 WebSocket 的连接已断开: {e}")
        print("请检查后端推流程序是否仍在运行。")
    except Exception as e:
        print(f"\n发生意外错误: {e}")
        traceback.print_exc()
    finally:
        # --- 断开连接 ---
        if ros_client and ros_client.is_connected:
            print("\n正在断开与 rosbridge 服务器的连接...")
            ros_client.terminate()
            await asyncio.sleep(0.5) # 给一点时间来关闭
            print("连接已断开。")
        print("脚本执行完毕。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n检测到手动中断，正在退出程序...")