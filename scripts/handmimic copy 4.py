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
# !!! 请再次确认这些名称与您的机器人配置完全匹配 !!!
L_ANGLE_COMMAND_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'  # 左手话题
R_ANGLE_COMMAND_TOPIC = '/r_arm/rm_driver/Hand_SetAngle'  # *** 新增：右手话题 ***
ANGLE_MESSAGE_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 2. 核心参数 (!!! 重要：需要微调校准 !!!) ---

# --- 平滑滤波器参数 ---
# 用于减少机器人抖动。值越低越平滑，但响应越慢。
SMOOTHING_FACTOR = 0.2

# --- MediaPipe 关键点索引 (左右手通用) ---
WRIST = 0
FINGER_TIPS = [4, 8, 12, 16, 20]  # [拇指, 食指, 中指, 无名指, 小指] 的指尖
THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP = FINGER_TIPS
THUMB_MCP = 2
THUMB_CMC = 1
INDEX_FINGER_MCP = 5

# --- 校准参数 (左右手暂时共用一套) ---
# 如果您发现左右手控制效果不一致，可以复制这些字典并创建针对右手 'R_' 的版本
# (1) 四指弯曲校准 (指尖到手腕距离)
MIN_MAX_DISTANCES = {
    'pinky': [0.04, 0.17],
    'ring': [0.05, 0.19],
    'middle': [0.06, 0.20],
    'index': [0.06, 0.18],
}
FINGER_NAMES = ['pinky', 'ring', 'middle', 'index']

# (2) 拇指 "展开伸直" 校准 (指尖(4)到根部(2)距离)
MIN_MAX_THUMB_FLEXION_DIST = [0.03, 0.08]

# (3) 拇指 "向里向外" 校准 (食指根(5)-手腕(0)-拇指根(1) 角度)
MIN_MAX_THUMB_ADDUCTION_ANGLE = [25, 60]


# --- 3. 辅助函数 ---

def parse_hand_data(data_string):
    """
    *** 修改: 从WebSocket接收的原始字符串中解析出手部类型('Left'或'Right')和世界坐标 ***
    返回: 一个元组 (hand_side, landmarks)，例如 ('Left', {landmark_dict})
          如果数据无效，则返回 (None, None)
    """
    try:
        data_string = data_string.strip()
        hand_side = None
        if data_string.startswith('Left'):
            hand_side = 'Left'
        elif data_string.startswith('Right'):
            hand_side = 'Right'
        else:
            return None, None # 如果不是手部数据，直接返回

        world_data_part = data_string.split('*||*')[0]
        lines = world_data_part.strip().split('\n')
        landmarks = {}
        # 从第二行开始解析 (第一行是 'Left' 或 'Right')
        for line in lines[1:]:
            parts = line.split('|')
            if len(parts) == 4:
                idx = int(parts[0])
                coords = np.array([float(p) for p in parts[1:]])
                landmarks[idx] = coords
        
        if len(landmarks) == 21:
            return hand_side, landmarks # 返回手部类型和关键点
        return None, None
    except Exception:
        return None, None

def calculate_robot_angles(landmarks):
    """
    根据手部关键点计算出机器人要求的6个目标角度 (0-1000)。
    此函数是通用的，无需修改。
    """
    if not landmarks:
        return None

    required_indices = [WRIST, THUMB_TIP, THUMB_MCP, THUMB_CMC, INDEX_FINGER_MCP] + FINGER_TIPS[1:]
    if not all(idx in landmarks for idx in required_indices):
        return None

    wrist_pos = landmarks[WRIST]
    
    # 四指弯曲度
    finger_curl_angles = {}
    for i, finger_name in enumerate(FINGER_NAMES):
        tip_idx = FINGER_TIPS[4 - i]
        dist = np.linalg.norm(landmarks[tip_idx] - wrist_pos)
        min_d, max_d = MIN_MAX_DISTANCES[finger_name]
        ratio = (dist - min_d) / (max_d - min_d)
        ratio = max(0.0, min(1.0, ratio))
        finger_curl_angles[finger_name] = int(ratio * 1000)

    # 拇指展开伸直
    flex_dist = np.linalg.norm(landmarks[THUMB_TIP] - landmarks[THUMB_MCP])
    min_flex_d, max_flex_d = MIN_MAX_THUMB_FLEXION_DIST
    flex_ratio = (flex_dist - min_flex_d) / (max_flex_d - min_flex_d)
    flex_ratio = max(0.0, min(1.0, flex_ratio))
    thumb_flexion_angle = int(flex_ratio * 1000)

    # 拇指向里向外
    v_wrist_thumb = landmarks[THUMB_CMC] - wrist_pos
    v_wrist_index = landmarks[INDEX_FINGER_MCP] - wrist_pos
    dot_product = np.dot(v_wrist_thumb, v_wrist_index)
    norm_product = np.linalg.norm(v_wrist_thumb) * np.linalg.norm(v_wrist_index)
    thumb_adduction_angle = 500
    if norm_product > 1e-6:
        angle_rad = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        min_angle, max_angle = MIN_MAX_THUMB_ADDUCTION_ANGLE
        adduction_ratio = (angle_deg - min_angle) / (max_angle - min_angle)
        adduction_ratio = max(0.0, min(1.0, adduction_ratio))
        thumb_adduction_angle = int(adduction_ratio * 1000)
    
    # 组装指令
    robot_angles = [
        finger_curl_angles['pinky'],
        finger_curl_angles['ring'],
        finger_curl_angles['middle'],
        finger_curl_angles['index'],
        thumb_adduction_angle,
        thumb_flexion_angle
    ]
    return robot_angles


# --- 4. 主程序 ---

async def main():
    """主函数，处理ROS连接、双臂数据平滑和WebSocket数据流"""
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    
    # *** 修改: 使用字典来管理左右手的发布者 ***
    publishers = {}
    
    # *** 修改: 使用字典来为每只手分别存储平滑后的角度 ***
    smoothed_angles_per_hand = {}

    try:
        # --- 连接 ROS ---
        print(f"正在连接到 rosbridge服务器: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
        ros_client.run()
        start_time = time.time()
        while not ros_client.is_connected:
            await asyncio.sleep(0.1)
            if time.time() - start_time > CONNECTION_TIMEOUT_SEC:
                raise TimeoutError(f"连接 ROS 超时 ({CONNECTION_TIMEOUT_SEC} 秒)")
        print("成功连接到 rosbridge 服务器！")

        # *** 修改: 创建并存储左右手两个发布者 ***
        publishers['Left'] = roslibpy.Topic(ros_client, L_ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        publishers['Right'] = roslibpy.Topic(ros_client, R_ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        print(f"已创建发布者, 左手话题: '{L_ANGLE_COMMAND_TOPIC}'")
        print(f"已创建发布者, 右手话题: '{R_ANGLE_COMMAND_TOPIC}'")

        # --- 连接 WebSocket ---
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            print(f"成功连接到手势识别 WebSocket: {WEBSOCKET_URI}")
            print(f"--- 开始接收双手手势数据并控制机器人 (平滑系数: {SMOOTHING_FACTOR}) ---")
            
            async for message_str in websocket:
                # 1. 解析原始数据，获取手部类型和关键点
                hand_side, landmarks = parse_hand_data(message_str)
                if not hand_side or not landmarks:
                    continue 

                # 2. 计算原始目标角度 (此函数可重用)
                raw_target_angles = calculate_robot_angles(landmarks)
                if not raw_target_angles:
                    continue

                # --- 3. ***核心改动: 对目标角度进行平滑滤波*** ---
                #    根据手部类型('Left'或'Right')获取对应的上一帧平滑数据
                last_smoothed = smoothed_angles_per_hand.get(hand_side)
                
                if last_smoothed is None:
                    # 如果是该手的第一帧数据，直接使用当前计算值作为初始值
                    current_smoothed = np.array(raw_target_angles, dtype=float)
                else:
                    # 使用EMA公式进行平滑
                    current_angles_np = np.array(raw_target_angles, dtype=float)
                    current_smoothed = (SMOOTHING_FACTOR * current_angles_np) + \
                                       ((1.0 - SMOOTHING_FACTOR) * last_smoothed)
                
                # *** 更新该手的平滑状态，以便下一帧使用 ***
                smoothed_angles_per_hand[hand_side] = current_smoothed
                
                # 将平滑后的浮点数结果转换为整数列表
                final_angles_to_send = [int(angle) for angle in current_smoothed]

                # 4. ***动态选择发布者并发布ROS消息***
                publisher = publishers.get(hand_side)
                if publisher:
                    ros_message = roslibpy.Message({'hand_angle': final_angles_to_send})
                    publisher.publish(ros_message)
                
                # 打印发送的指令，并留出一些空格以清除上一条可能更长的打印
                print(f"发送指令 -> {hand_side} 手: {final_angles_to_send}      ", end='\r')


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