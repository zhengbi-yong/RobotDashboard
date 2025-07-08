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

# ROS 话题和服务配置
L_ANGLE_COMMAND_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'  # 左手话题
R_ANGLE_COMMAND_TOPIC = '/r_arm/rm_driver/Hand_SetAngle'  # 右手话题
ANGLE_MESSAGE_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 2. 核心参数 (!!! 重要：需要微调校准 !!!) ---

# --- 平滑滤波器参数 ---
SMOOTHING_FACTOR = 0.2

# --- MediaPipe 关键点索引 (左右手通用) ---
WRIST = 0
FINGER_TIPS = [4, 8, 12, 16, 20]  # [拇指, 食指, 中指, 无名指, 小指] 的指尖
THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP = FINGER_TIPS
THUMB_MCP = 2
THUMB_CMC = 1
INDEX_FINGER_MCP = 5

# --- 校准参数 (左右手共用) ---
MIN_MAX_DISTANCES = {
    'pinky': [0.04, 0.17], 'ring': [0.05, 0.19],
    'middle': [0.06, 0.20], 'index': [0.06, 0.18],
}
FINGER_NAMES = ['pinky', 'ring', 'middle', 'index']
MIN_MAX_THUMB_FLEXION_DIST = [0.03, 0.08]
MIN_MAX_THUMB_ADDUCTION_ANGLE = [25, 60]


# --- 3. 辅助函数 ---

def parse_hand_data(data_string):
    """从WebSocket接收的原始字符串中解析出手部类型('Left'或'Right')和世界坐标"""
    try:
        data_string = data_string.strip()
        hand_side = None
        if data_string.startswith('Left'): hand_side = 'Left'
        elif data_string.startswith('Right'): hand_side = 'Right'
        else: return None, None

        world_data_part = data_string.split('*||*')[0]
        lines = world_data_part.strip().split('\n')
        landmarks = {}
        for line in lines[1:]:
            parts = line.split('|')
            if len(parts) == 4:
                idx = int(parts[0])
                coords = np.array([float(p) for p in parts[1:]])
                landmarks[idx] = coords
        
        if len(landmarks) == 21: return hand_side, landmarks
        return None, None
    except Exception:
        return None, None

def calculate_robot_angles(landmarks):
    """根据手部关键点计算出机器人要求的6个目标角度 (0-1000)"""
    if not landmarks: return None
    required_indices = [WRIST, THUMB_TIP, THUMB_MCP, THUMB_CMC, INDEX_FINGER_MCP] + FINGER_TIPS[1:]
    if not all(idx in landmarks for idx in required_indices): return None

    wrist_pos = landmarks[WRIST]
    
    finger_curl_angles = {}
    for i, finger_name in enumerate(FINGER_NAMES):
        tip_idx = FINGER_TIPS[4 - i]
        dist = np.linalg.norm(landmarks[tip_idx] - wrist_pos)
        min_d, max_d = MIN_MAX_DISTANCES[finger_name]
        ratio = max(0.0, min(1.0, (dist - min_d) / (max_d - min_d)))
        finger_curl_angles[finger_name] = int(ratio * 1000)

    flex_dist = np.linalg.norm(landmarks[THUMB_TIP] - landmarks[THUMB_MCP])
    min_flex_d, max_flex_d = MIN_MAX_THUMB_FLEXION_DIST
    flex_ratio = max(0.0, min(1.0, (flex_dist - min_flex_d) / (max_flex_d - min_flex_d)))
    thumb_flexion_angle = int(flex_ratio * 1000)

    v_wrist_thumb = landmarks[THUMB_CMC] - wrist_pos
    v_wrist_index = landmarks[INDEX_FINGER_MCP] - wrist_pos
    dot_product = np.dot(v_wrist_thumb, v_wrist_index)
    norm_product = np.linalg.norm(v_wrist_thumb) * np.linalg.norm(v_wrist_index)
    thumb_adduction_angle = 500
    if norm_product > 1e-6:
        angle_rad = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        min_angle, max_angle = MIN_MAX_THUMB_ADDUCTION_ANGLE
        adduction_ratio = max(0.0, min(1.0, (angle_deg - min_angle) / (max_angle - min_angle)))
        thumb_adduction_angle = int(adduction_ratio * 1000)
    
    return [
        finger_curl_angles['pinky'], finger_curl_angles['ring'],
        finger_curl_angles['middle'], finger_curl_angles['index'],
        thumb_adduction_angle, thumb_flexion_angle
    ]


# --- 4. 主程序 ---

async def main():
    """主函数，处理ROS连接、双臂数据平滑和WebSocket数据流"""
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    publishers = {}
    smoothed_angles_per_hand = {}

    try:
        # --- 连接 ROS ---
        print(f"正在连接到 rosbridge服务器: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
        ros_client.run()
        
        # *** 修正 #1: 使用手动循环来等待连接，提高兼容性 ***
        start_time = time.time()
        while not ros_client.is_connected:
            await asyncio.sleep(0.1)
            if time.time() - start_time > CONNECTION_TIMEOUT_SEC:
                # 手动抛出 TimeoutError 异常
                raise TimeoutError(f"连接 ROS 超时 ({CONNECTION_TIMEOUT_SEC} 秒)")
        
        print("成功连接到 rosbridge 服务器！")

        # --- 创建并存储左右手两个发布者 ---
        publishers['Left'] = roslibpy.Topic(ros_client, L_ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        publishers['Right'] = roslibpy.Topic(ros_client, R_ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        print(f"已创建发布者, 左手: '{L_ANGLE_COMMAND_TOPIC}' | 右手: '{R_ANGLE_COMMAND_TOPIC}'")

        # --- 连接 WebSocket ---
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            print(f"成功连接到手势识别 WebSocket: {WEBSOCKET_URI}")
            print(f"--- 开始接收双手手势数据并控制机器人 (平滑系数: {SMOOTHING_FACTOR}) ---")
            
            async for message_str in websocket:
                # 1. 解析原始数据
                hand_side, landmarks = parse_hand_data(message_str)
                if not hand_side or not landmarks:
                    continue 

                # 2. 计算目标角度
                raw_target_angles = calculate_robot_angles(landmarks)
                if not raw_target_angles:
                    continue

                # 3. 平滑滤波并更新状态
                last_smoothed = smoothed_angles_per_hand.get(hand_side)
                current_angles_np = np.array(raw_target_angles, dtype=float)
                if last_smoothed is None:
                    current_smoothed = current_angles_np
                else:
                    current_smoothed = (SMOOTHING_FACTOR * current_angles_np) + \
                                       ((1.0 - SMOOTHING_FACTOR) * last_smoothed)
                smoothed_angles_per_hand[hand_side] = current_smoothed

                # 4. 同时发布所有手的指令
                output_log = ""
                for side, publisher in publishers.items():
                    if side in smoothed_angles_per_hand:
                        final_angles = [int(angle) for angle in smoothed_angles_per_hand[side]]
                        ros_message = roslibpy.Message({'hand_angle': final_angles})
                        publisher.publish(ros_message)
                        output_log += f"{side} Hand: {final_angles} | "
                
                print(f"发送指令 -> {output_log.rstrip(' | ')}      ", end='\r')


    except TimeoutError as e:
        # 这个异常现在由我们自己手动抛出
        print(f"\n错误: {e}")
        print("请检查：\n1. rosbridge 是否正在运行 (roslaunch rosbridge_server rosbridge_websocket.launch)？")
        print(f"2. IP 地址 '{ROS_BRIDGE_HOST}' 是否正确且可以访问？")

    # *** 修正 #2: 去掉 .exceptions，直接使用 websockets.ConnectionClosedError ***
    except websockets.ConnectionClosedError as e:
        print(f"\n错误: 与手势识别 WebSocket 的连接已断开: {e}")
        print("请检查后端推流程序是否仍在运行。")
    except Exception as e:
        print(f"\n发生意外错误: {e}")
        traceback.print_exc()
    finally:
        if ros_client and ros_client.is_connected:
            print("\n正在断开与 rosbridge 服务器的连接...")
            ros_client.terminate()
            await asyncio.sleep(0.5) 
            print("连接已断开。")
        print("脚本执行完毕。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n检测到手动中断，正在退出程序...")