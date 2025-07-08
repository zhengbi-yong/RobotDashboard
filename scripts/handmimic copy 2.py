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
ANGLE_COMMAND_TOPIC = '/l_arm/rm_driver/Hand_SetAngle'
ANGLE_MESSAGE_TYPE = 'dual_arm_msgs/Hand_Angle'

# --- 2. 手势到机器人角度映射的核心参数 (!!! 重要：需要微调校准 !!!) ---

# --- MediaPipe 关键点索引 ---
# 我们将使用这些索引来定位手的特定部位
WRIST = 0
FINGER_TIPS = [4, 8, 12, 16, 20]  # [拇指, 食指, 中指, 无名指, 小指] 的指尖
THUMB_TIP, INDEX_TIP, MIDDLE_TIP, RING_TIP, PINKY_TIP = FINGER_TIPS

THUMB_MCP = 2           # 拇指掌指关节 (可以理解为拇指根部)
THUMB_CMC = 1           # 拇指腕掌关节 (更靠近手腕的关节)
INDEX_FINGER_MCP = 5    # 食指根部

# --- 校准参数 ---
# 这是整个程序最关键的部分，您需要通过实验来找到最适合您自己的值。
# 方法：运行脚本，观察控制台打印的实时数据，记录下您在做特定动作时的数值范围。

# (1) 四个手指 (小、无、中、食) 的弯曲校准
#     测量每个指尖到手腕的距离。
#     格式: '手指名': [指尖到手腕的最小距离 (完全握拳), 最大距离 (完全伸直)]
MIN_MAX_DISTANCES = {
    'pinky': [0.04, 0.17],    # 小指 (示例值)
    'ring': [0.05, 0.19],     # 无名指 (示例值)
    'middle': [0.06, 0.20],   # 中指 (示例值)
    'index': [0.06, 0.18],    # 食指 (示例值)
}
# 与上面对应的手指名称列表 (用于循环)
FINGER_NAMES = ['pinky', 'ring', 'middle', 'index']

# (2) 拇指 "展开伸直" (第6个值) 的校准
#     测量拇指指尖(4)到拇指根部(2)的距离。
#     格式: [最小距离 (拇指完全弯曲), 最大距离 (拇指完全伸直)]
MIN_MAX_THUMB_FLEXION_DIST = [0.03, 0.08] # (示例值)

# (3) 拇指 "向里向外" (第5个值) 的校准
#     测量由 食指根(5)-手腕(0)-拇指根(1) 形成的角度。
#     格式: [最小角度 (拇指内收到最里), 最大角度 (拇指外展到最外)]，单位是度。
MIN_MAX_THUMB_ADDUCTION_ANGLE = [25, 60] # (示例值)


# --- 3. 辅助函数 ---

def parse_hand_data(data_string):
    """从WebSocket接收的原始字符串中解析出左手的世界坐标"""
    try:
        world_data_part = data_string.split('*||*')[0]
        if not world_data_part.strip().startswith('Left'):
            return None

        lines = world_data_part.strip().split('\n')
        landmarks = {}
        for line in lines[1:]:
            parts = line.split('|')
            if len(parts) == 4:
                idx = int(parts[0])
                coords = np.array([float(p) for p in parts[1:]])
                landmarks[idx] = coords
        
        if len(landmarks) == 21:
            return landmarks
        else:
            # print(f"警告: 解析到的关键点数量不足 ({len(landmarks)}/21)，跳过此帧。")
            return None
    except Exception:
        # print(f"数据解析出错: {e}\n原始数据: '{data_string[:100]}...'")
        return None

def calculate_robot_angles(landmarks):
    """
    根据手部关键点计算出机器人要求的6个目标角度 (0-1000)。
    输出顺序: [小拇指, 无名指, 中指, 食指, 拇指向里向外, 拇指展开伸直]
    """
    if not landmarks:
        return None

    # 检查所有需要的关键点是否存在，避免程序因缺少数据而出错
    required_indices = [WRIST, THUMB_TIP, THUMB_MCP, THUMB_CMC, INDEX_FINGER_MCP] + FINGER_TIPS[1:]
    if not all(idx in landmarks for idx in required_indices):
        print("警告: 缺少计算所需的关键点，跳过此帧。")
        return None

    wrist_pos = landmarks[WRIST]
    
    # --- 任务1: 计算四个手指的弯曲度 (小指, 无名指, 中指, 食指) ---
    finger_curl_angles = {}
    for i, finger_name in enumerate(FINGER_NAMES):
        tip_idx = FINGER_TIPS[4 - i] # 按 [小,无,中,食] 的顺序获取指尖索引
        
        dist = np.linalg.norm(landmarks[tip_idx] - wrist_pos)
        
        min_d, max_d = MIN_MAX_DISTANCES[finger_name]
        ratio = (dist - min_d) / (max_d - min_d)
        ratio = max(0.0, min(1.0, ratio))  # 将比例限制在0到1之间
        
        # 机器人指令是“张开度”，所以我们用 ratio 直接乘以 1000
        finger_curl_angles[finger_name] = int(ratio * 1000)

    # --- 任务2: 计算 "拇指展开伸直" (第6个值) ---
    thumb_tip_pos = landmarks[THUMB_TIP]
    thumb_mcp_pos = landmarks[THUMB_MCP]
    flex_dist = np.linalg.norm(thumb_tip_pos - thumb_mcp_pos)
    
    min_flex_d, max_flex_d = MIN_MAX_THUMB_FLEXION_DIST
    flex_ratio = (flex_dist - min_flex_d) / (max_flex_d - min_flex_d)
    flex_ratio = max(0.0, min(1.0, flex_ratio))
    thumb_flexion_angle = int(flex_ratio * 1000)

    # --- 任务3: 计算 "拇指向里向外" (第5个值) ---
    index_mcp_pos = landmarks[INDEX_FINGER_MCP]
    thumb_cmc_pos = landmarks[THUMB_CMC]
    
    v_wrist_thumb = thumb_cmc_pos - wrist_pos
    v_wrist_index = index_mcp_pos - wrist_pos
    
    dot_product = np.dot(v_wrist_thumb, v_wrist_index)
    norm_product = np.linalg.norm(v_wrist_thumb) * np.linalg.norm(v_wrist_index)
    
    thumb_adduction_angle = 500 # 默认值
    if norm_product > 1e-6: # 避免除以零
        angle_rad = np.arccos(np.clip(dot_product / norm_product, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        
        min_angle, max_angle = MIN_MAX_THUMB_ADDUCTION_ANGLE
        adduction_ratio = (angle_deg - min_angle) / (max_angle - min_angle)
        adduction_ratio = max(0.0, min(1.0, adduction_ratio))
        thumb_adduction_angle = int(adduction_ratio * 1000)
    
    # --- 任务4: 按机器人要求的顺序组装最终指令 ---
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
    """主函数，处理ROS连接和WebSocket数据流"""
    
    ros_client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    ros_publisher = None

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

        ros_publisher = roslibpy.Topic(ros_client, ANGLE_COMMAND_TOPIC, ANGLE_MESSAGE_TYPE)
        print(f"已创建发布者, 话题: '{ANGLE_COMMAND_TOPIC}'")

        # --- 连接 WebSocket ---
        async with websockets.connect(WEBSOCKET_URI) as websocket:
            print(f"成功连接到手势识别 WebSocket: {WEBSOCKET_URI}")
            print("\n--- 开始接收手势数据并控制机器人 ---")
            
            async for message_str in websocket:
                landmarks = parse_hand_data(message_str)
                if not landmarks:
                    continue 

                target_angles = calculate_robot_angles(landmarks)
                if not target_angles:
                    continue

                ros_message = roslibpy.Message({'hand_angle': target_angles})
                ros_publisher.publish(ros_message)
                
                # 在控制台打印发送的指令，便于调试
                print(f"发送指令: {target_angles}", end='\r')


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