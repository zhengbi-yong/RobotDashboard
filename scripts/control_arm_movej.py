import roslibpy
import time
import os
import math # 导入 math 模块用于度数到弧度的转换
from loguru import logger
# --- 配置参数 ---
# ROS_BRIDGE_HOST = 'www.wanrenai.com'  # Replace with your ROS master/rosbridge IP
# ROS_BRIDGE_HOST = '101.126.13.137'  # Replace with your ROS master/rosbridge IP
ROS_BRIDGE_HOST = '192.168.0.105'  # Replace with your ROS master/rosbridge IP
ROS_BRIDGE_PORT = 9090

CONNECTION_TIMEOUT_SEC = 10     # 等待连接的超时时间 (秒)

# --- 手臂控制参数 ---
ARM_COMMAND_TOPIC = '/l_arm/rm_driver/MoveJ_Cmd' # Topic for left arm MoveJ commands (假设是左臂)
ARM_MESSAGE_TYPE = 'dual_arm_msgs/MoveJ'      # Message type for MoveJ

# --- 需要发送的指令内容 ---
# ***** 修改: 输入以度为单位 *****
# TARGET_JOINT_ANGLES_DEG = [-116.111000061,-15.570999146,-35.953998566,-100.468002319,90.685997009,-90.078002930,60.738998413] # 示例：第四个关节旋转90度
TARGET_JOINT_ANGLES_DEG = [0,0,0,0,0,0,0] # 示例：第四个关节旋转90度
# 确保这个列表的长度与你的手臂自由度匹配 (dual_arm_msgs/MoveJ 中的 joint 字段期望的长度)
# 如果你的 MoveJ 消息期望7个关节，这里应该是7个值。

TARGET_SPEED = 0.5                               # Movement speed (0.0 to 1.0 or specific units, check message def)
TRAJECTORY_CONNECT = 0                           # Trajectory connection flag (0: overwrite, 1: append)
# --- 配置结束 ---

logger.info(f"准备连接到 rosbridge 服务器: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)

try:
    client.run() # Starts the connection attempt

    logger.info("正在等待连接...")
    start_time_conn = time.time()
    while not client.is_connected:
        time.sleep(0.1)
        if time.time() - start_time_conn > CONNECTION_TIMEOUT_SEC:
            raise TimeoutError(f"连接到 rosbridge 服务器超时（超过 {CONNECTION_TIMEOUT_SEC} 秒）")

    logger.info("成功连接到 rosbridge 服务器！")

    publisher = roslibpy.Topic(client, ARM_COMMAND_TOPIC, ARM_MESSAGE_TYPE)
    logger.info(f"已创建发布者，话题: '{ARM_COMMAND_TOPIC}', 类型: '{ARM_MESSAGE_TYPE}'")

    # --- 单位转换: 从度转换为弧度 ---
    # 确保 TARGET_JOINT_ANGLES_DEG 中的元素数量与 MoveJ 消息中的 'joint' 字段期望的长度一致
    # 例如，如果你的 dual_arm_msgs/MoveJ 消息中的 joint 是 float64[7] joint，那么这里应该是7个角度
    # 如果是 float64[] joint，那么它会接受任意长度，但机器人控制器端可能会有期望的长度
    if len(TARGET_JOINT_ANGLES_DEG) != 7: # 假设 rm_driver/MoveJ_Cmd 期望7个关节值 (l_joint1 to l_joint7)
        logger.warning(f"警告: TARGET_JOINT_ANGLES_DEG 提供了 {len(TARGET_JOINT_ANGLES_DEG)} 个角度, "
                       f"但通常 MoveJ 可能期望特定数量的关节 (例如6或7)。请确认。")
        # 根据你的实际情况，你可能需要在这里添加错误处理或调整关节数量

    target_joint_angles_rad = [math.radians(deg) for deg in TARGET_JOINT_ANGLES_DEG]
    logger.info(f"输入的关节角度 (度): {TARGET_JOINT_ANGLES_DEG}")
    logger.info(f"转换后的关节角度 (弧度): {[round(rad, 4) for rad in target_joint_angles_rad]}")


    # --- 创建 MoveJ 消息 ---
    # 确保这里的字段名称与 dual_arm_msgs/MoveJ 消息定义完全匹配。
    # 在 ROS 环境中运行 `rosmsg show dual_arm_msgs/MoveJ` 来确认。
    # 假设 dual_arm_msgs/MoveJ 的定义是：
    #   float64[] joint   (或者 float64[6] joint / float64[7] joint)
    #   float64 speed
    #   uint8 trajectory_connect
    message_data = {
        'joint': target_joint_angles_rad, # 使用转换后的弧度值
        'speed': TARGET_SPEED,
        'trajectory_connect': TRAJECTORY_CONNECT
    }
    message = roslibpy.Message(message_data)
    logger.info(f"准备发送的 MoveJ 指令 (弧度): {message.data}")

    logger.info("正在发布 MoveJ 指令...")
    publisher.publish(message)
    logger.info(f"已发布 MoveJ 指令: joint_rad={message_data['joint']}, speed={TARGET_SPEED}, trajectory_connect={TRAJECTORY_CONNECT}")

    time.sleep(1.0)
    logger.info("指令发布完成。")

except TimeoutError as e:
    logger.error(f"错误：{e}")
    logger.info("请检查：")
    logger.info("1. rosbridge 服务器是否已在目标机器上运行？")
    logger.info("   (在 ROS 机器上运行: roslaunch rosbridge_server rosbridge_websocket.launch)")
    logger.info(f"2. IP 地址 '{ROS_BRIDGE_HOST}' 是否正确且可以从当前机器访问？")
    logger.info("3. 防火墙是否允许端口 9090 的连接？")
except roslibpy.core.RosTimeoutError as e:
    logger.error(f"ROS 操作超时错误: {e}")
except Exception as e:
    logger.error(f"发生意外错误: {e}")
    import traceback
    traceback.print_exc()
finally:
    if client.is_connected:
        logger.info("正在断开与 rosbridge 服务器的连接...")
        client.terminate()
        time.sleep(0.5) # 给后台线程一点时间关闭
        logger.info("连接已断开。")
    else:
        logger.info("客户端未连接或已断开，尝试终止...")
        client.terminate() # 即使未连接也调用terminate以清理
        time.sleep(0.5)
        logger.info("脚本尝试终止。")

logger.info("脚本执行完毕。")