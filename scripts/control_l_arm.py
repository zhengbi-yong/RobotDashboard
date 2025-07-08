# 左起始位置：-5.566 -90.048 67.265 -9.144 15.361 0 -10.915
# 1：93.181 -91.408 85.089 103.572 -14.646 64.805 -117.317
# 2：-63.491 -72.353 66.692 1.955 15.509 18.815 -10.914
# 3：93.181 -91.408 85.089 103.572 -14.646 64.805 -117.317
# 4：-5.566 -90.048 67.265 -9.144 15.361 0 -10.915

# 右起始位置：-4.162 87.643 -79.405 -13.297 -3.766 10.634 -5.867
# 1：-32.915 91.757 -72.893 -122.004 -2.485 -2.044 -5.868
# 2：79.501 69.598 -79.409 -13.668 -3.747 7.712 -5.869
# 3：-32.915 91.757 -72.893 -122.004 -2.485 -2.044 -5.868
# 4：-4.162 87.643 -79.405 -13.297 -3.766 10.634 -5.867


import roslibpy
import time
import os

# --- 配置参数 ---
ROS_BRIDGE_HOST = '192.168.0.105'  # Replace with your ROS master/rosbridge IP
ROS_BRIDGE_PORT = 9090
CONNECTION_TIMEOUT_SEC = 10     # 等待连接的超时时间 (秒)

# --- 手臂控制参数 ---
ARM_COMMAND_TOPIC = '/l_arm/rm_driver/MoveJ_Cmd' # Topic for right arm MoveJ commands
ARM_MESSAGE_TYPE = 'dual_arm_msgs/MoveJ'      # Message type for MoveJ
# --- 需要发送的指令内容 ---
TARGET_JOINT_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # Target joint angles (use floats for clarity)
TARGET_SPEED = 0.5                               # Movement speed
TRAJECTORY_CONNECT = 0                           # Trajectory connection flag (0: overwrite, 1: append)
# --- 配置结束 ---

print(f"准备连接到 rosbridge 服务器: ws://{ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}")
# 连接到 rosbridge 服务器
client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)

try:
    # 启动客户端连接 (非阻塞)
    # client.run() # run() will block until terminated if run in the main thread
    # Use run_forever() in a separate thread or manage the event loop if needed for subscriptions,
    # but for a single publish, run() followed by terminate() might be enough if the publish happens quickly.
    # A safer approach for ensuring connection is established:
    client.run() # Starts the connection attempt in the background

    # 等待连接成功建立，设置超时
    print("正在等待连接...")
    start_time_conn = time.time()
    while not client.is_connected:
        time.sleep(0.1) # 短暂休眠，避免CPU空转
        if time.time() - start_time_conn > CONNECTION_TIMEOUT_SEC:
            raise TimeoutError(f"连接到 rosbridge 服务器超时（超过 {CONNECTION_TIMEOUT_SEC} 秒）")

    # 如果循环正常结束，说明连接成功
    print("成功连接到 rosbridge 服务器！")

    # --- 连接成功后，创建发布者 ---
    publisher = roslibpy.Topic(client, ARM_COMMAND_TOPIC, ARM_MESSAGE_TYPE)
    print(f"已创建发布者，话题: '{ARM_COMMAND_TOPIC}', 类型: '{ARM_MESSAGE_TYPE}'")

    # --- 创建 MoveJ 消息 ---
    # 注意：确保这里的字段名称 ('joint', 'speed', 'trajectory_connect')
    # 与 dual_arm_msgs/MoveJ 消息定义完全匹配。
    # 你可以通过在 ROS 环境中运行 `rosmsg show dual_arm_msgs/MoveJ` 来确认。
    message = roslibpy.Message({
        'joint': TARGET_JOINT_ANGLES,
        'speed': TARGET_SPEED,
        'trajectory_connect': TRAJECTORY_CONNECT
    })
    print(f"准备发送的 MoveJ 指令: {message.data}")

    # --- 发布消息 (通常 MoveJ 只需要发布一次) ---
    print("正在发布 MoveJ 指令...")
    publisher.publish(message)
    print(f"已发布 MoveJ 指令: joint={TARGET_JOINT_ANGLES}, speed={TARGET_SPEED}, trajectory_connect={TRAJECTORY_CONNECT}")

    # 短暂等待，确保消息有时间通过网络发送出去，尤其是在脚本即将终止时
    time.sleep(1.0)
    print("指令发布完成。")

# 捕捉连接超时错误
except TimeoutError as e:
    print(f"错误：{e}")
    print("请检查：")
    print("1. rosbridge 服务器是否已在目标机器上运行？")
    print("   (在 ROS 机器上运行: roslaunch rosbridge_server rosbridge_websocket.launch)")
    print(f"2. IP 地址 '{ROS_BRIDGE_HOST}' 是否正确且可以从当前机器访问？")
    print("3. 防火墙是否允许端口 9090 的连接？")
# 捕捉其他可能的 roslibpy 或网络错误
except roslibpy.core.RosTimeoutError as e:
    print(f"ROS 操作超时错误: {e}")
except Exception as e:
    print(f"发生意外错误: {e}")
    import traceback
    traceback.print_exc() # 打印详细的错误堆栈信息
finally:
    # 无论是否成功，最后都尝试断开连接
    if client.is_connected:
        print("正在断开与 rosbridge 服务器的连接...")
        client.terminate()
        # 等待一小段时间确保后台线程完全关闭
        time.sleep(0.5)
        print("连接已断开。")
    else:
        # 如果连接从未成功或已断开，也尝试终止以清理可能存在的后台线程
        print("客户端未连接或已断开，尝试终止后台线程（如果存在）...")
        client.terminate()
        time.sleep(0.5)
        print("脚本尝试终止。")

print("脚本执行完毕。")