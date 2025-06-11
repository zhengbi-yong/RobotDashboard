import numpy as np
import pinocchio as pin
from numpy.linalg import norm, solve
from pathlib import Path
import roslibpy
import roslibpy.actionlib # 导入actionlib
import time
import os
import traceback
import math
import threading # 导入threading

# --- 用于MoveIt!阻塞调用的全局变量 ---
moveit_goal_completed = threading.Event()
moveit_result_success = False

def compute_zyx_transform(alpha_deg, beta_deg, gamma_deg, p):
    """
    计算 ZYX 顺序的旋转矩阵和齐次变换矩阵。
    """
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    gamma = np.deg2rad(gamma_deg)

    R_x = np.array([[1, 0, 0], [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)]])
    R_y = np.array([[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]])
    R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0], [np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])

    R = R_z @ R_y @ R_x
    p = np.array(p)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = p
    return R, H

def compute_single_arm_ik(model, data, q, oMdes, joint_id, eps=5e-3, it_max=5000, dt=0.2, damp=1e-5):
    """计算单臂逆运动学，以 l_base_link 或 r_base_link 作为基座标系。"""
    lower_limits = model.lowerPositionLimit
    upper_limits = model.upperPositionLimit
    i = 0
    prev_err = np.inf

    while i < it_max:
        pin.forwardKinematics(model, data, q)
        base_to_ee = data.oMi[joint_id].actInv(oMdes)
        err = pin.log(base_to_ee).vector
        
        if norm(err) < eps:
            return q, True, i, norm(err)
        
        J_ee = pin.computeJointJacobian(model, data, q, joint_id)
        J = -np.dot(pin.Jlog6(base_to_ee.inverse()), J_ee)
        local_dt = dt * (1.5 if norm(err) > 0.01 else 0.5 if norm(err) < eps * 2 else 1.0)
        rank = np.linalg.matrix_rank(J)
        local_damp = damp if rank == 6 else 1e-3
        v = -J.T.dot(solve(J.dot(J.T) + local_damp * np.eye(6), err))
        q_new = pin.integrate(model, q, local_dt * v)
        
        for j in range(model.nq):
            q_new[j] = np.mod(q_new[j] + np.pi, 2 * np.pi) - np.pi
            if not np.isnan(lower_limits[j]) and not np.isnan(upper_limits[j]):
                q_new[j] = np.clip(q_new[j], lower_limits[j], upper_limits[j])
        
        q = q_new
        if norm(err) > prev_err:
            dt *= 0.5
        prev_err = norm(err)
        i += 1
    
    return q, False, i, norm(err)

def move_to_pose(model, data, q, joint_id, oMdes, arm_name):
    """通过多次尝试计算逆运动学。"""
    for attempt in range(5):
        q_attempt = q.copy() if attempt == 0 else pin.randomConfiguration(model)
        q_new, success, iteration, final_error = compute_single_arm_ik(model, data, q_attempt, oMdes, joint_id)
        if success:
            print(f"IK 收敛! 尝试次数: {attempt + 1}, 迭代次数: {iteration}, 最终误差: {final_error}")
            pin.forwardKinematics(model, data, q_new)
            print(f"{arm_name} 末端位置: {data.oMi[joint_id].translation}")
            return q_new, success
        else:
            print(f"尝试 {attempt + 1} 失败, 迭代次数: {iteration}, 最终误差: {final_error}")
    
    pin.forwardKinematics(model, data, q_new)
    print(f"{arm_name} 末端位置: {data.oMi[joint_id].translation}")
    return q_new, False

def control_hand_angle(publisher, hand_angles, hand_name):
    """发送灵巧手抓握指令。"""
    message = roslibpy.Message({'hand_angle': hand_angles})
    print(f"发布 {hand_name} 抓握指令: hand_angle={hand_angles}")
    publisher.publish(message)
    return True

def move_arm_with_moveit(action_client, arm_config, target_joint_angles):
    """
    通过MoveIt!发送关节目标，并等待其完成（阻塞式）。
    """
    global moveit_goal_completed, moveit_result_success

    if not action_client:
        print("[ERROR] MoveIt! Action Client 未初始化。")
        return False

    print(f"--- 正在通过MoveIt!发送关节目标 ---")
    print(f"      规划组: {arm_config['group']}")
    print(f"      目标关节角 (rad): {[f'{angle:.4f}' for angle in target_joint_angles]}")
    
    moveit_goal_completed.clear()
    moveit_result_success = False

    def _on_result(result):
        global moveit_result_success
        error_code = result['error_code']['val']
        if error_code == 1:  # SUCCESS
            print("[SUCCESS] MoveIt! 目标已成功完成。")
            moveit_result_success = True
        else:
            print(f"[FAILURE] MoveIt! 目标失败，错误代码: {error_code}。")
            moveit_result_success = False
        moveit_goal_completed.set()

    joint_constraints = [
        {'joint_name': name, 'position': pos, 'tolerance_above': 0.01, 'tolerance_below': 0.01, 'weight': 1.0}
        for name, pos in zip(arm_config['joints'], target_joint_angles)
    ]
    goal_constraints = {'name': f"{arm_config['group']}_goal", 'joint_constraints': joint_constraints}

    goal_message = {
        'request': {
            'group_name': arm_config['group'],
            'goal_constraints': [goal_constraints],
            'num_planning_attempts': 5,
            'allowed_planning_time': 10.0,
            'planner_id': "RRTConnect",
            'max_velocity_scaling_factor': 0.5,
            'max_acceleration_scaling_factor': 0.5
        },
        'planning_options': {'plan_only': False, 'replan': False, 'replan_attempts': 3}
    }
    
    goal = roslibpy.actionlib.Goal(action_client, goal_message)
    goal.on('result', _on_result)
    goal.send()
    
    print(f"      MoveIt! 目标已发送 (Goal ID: {goal.goal_id})。正在等待执行完成...")
    
    completed_in_time = moveit_goal_completed.wait(timeout=30.0)

    if not completed_in_time:
        print("[ERROR] MoveIt! 目标执行超时！")
        goal.cancel()
        return False
        
    return moveit_result_success

def main():
    os.environ['ROS_DOMAIN_ID'] = '0'

    # --- 配置参数 ---
    ROS_BRIDGE_HOST = '192.168.0.105'
    ROS_BRIDGE_PORT = 9090
    CONNECTION_TIMEOUT_SEC = 10

    # --- MoveIt! 配置 ---
    MOVE_GROUP_ACTION_NAME = '/move_group'
    MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
    MOVEIT_CONFIG = {
        'la': {'group': 'l_arm', 'joints': [f'l_joint{i+1}' for i in range(7)]},
        'ra': {'group': 'r_arm', 'joints': [f'r_joint{i+1}' for i in range(7)]}
    }
    
    # 灵巧手参数
    TARGET_HAND_OPEN_ANGLES = [1000,1000,1000,1000,1000,1000]
    TARGET_HAND_GRASP_ANGLES = [0, 0, 0, 0, 0, 1000]

    # 加载 URDF 模型
    l_urdf_filename = Path("./dual_75B_left_arm.urdf")
    r_urdf_filename = Path("./dual_75B_right_arm.urdf")
    l_model = pin.buildModelFromUrdf(str(l_urdf_filename))
    r_model = pin.buildModelFromUrdf(str(r_urdf_filename))
    print("左臂模型名称: " + l_model.name)
    print("右臂模型名称: " + r_model.name)
    l_data = l_model.createData()
    r_data = r_model.createData()

    l_q = pin.neutral(l_model)
    r_q = pin.neutral(r_model)
    print(f"左臂初始关节角: {l_q}")
    print(f"右臂初始关节角: {r_q}")

    joint_id = 7

    # ROS 初始化
    client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    moveit_action_client = None
    try:
        client.run()
        print(f"正在连接到 rosbridge 服务器 at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        start_time_conn = time.time()
        while not client.is_connected:
            time.sleep(0.1)
            if time.time() - start_time_conn > CONNECTION_TIMEOUT_SEC:
                raise TimeoutError(f"连接到 rosbridge 服务器超时（超过 {CONNECTION_TIMEOUT_SEC} 秒）")
        
        print("成功连接到 rosbridge 服务器")
        
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        print("MoveIt! Action Client 已初始化。")

    except TimeoutError as e:
        print(f"错误: {e}")
        print("请检查：\n1. rosbridge服务器是否已在目标机器上运行？\n2. IP地址和端口是否正确？\n3. 防火墙设置。")
        if client and client.is_connected: client.terminate()
        return
    except Exception as e:
        print(f"连接或初始化失败: {e}")
        traceback.print_exc()
        if client and client.is_connected: client.terminate()
        return

    l_hand_publisher = roslibpy.Topic(client, '/l_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')
    r_hand_publisher = roslibpy.Topic(client, '/r_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')

    # --- 【已修改的逻辑】 ---
    print("\n移动到初始关节配置 (Home)")
    l_home_q = np.zeros(7)
    r_home_q = np.zeros(7)
    
    print("移动左臂到Home...")
    if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['la'], l_home_q):
        print("警告: 左臂移动到Home失败，但脚本将继续执行。")

    print("移动右臂到Home...")
    if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['ra'], r_home_q):
        print("警告: 右臂移动到Home失败，但脚本将继续执行。")
    
    print("\n移动到预备姿态")
    l_pre_q = np.array([-0.0014, 0.3878, -0.0028, 1.2397, 0.0237, -0.0968, -0.0006])
    r_pre_q = np.array([-0.0002, -0.1898, -0.0002, -1.2850, 0.0008, -0.1903, 0.0])

    print("移动左臂到预备姿态...")
    if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['la'], l_pre_q):
        print("警告: 左臂移动到预备姿态失败，但脚本将继续执行。")
    
    print("移动右臂到预备姿态...")
    if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['ra'], r_pre_q):
        print("警告: 右臂移动到预备姿态失败，但脚本将继续执行。")

    l_q = r_pre_q.copy() # 假设初始姿态被成功设置
    r_q = r_pre_q.copy()

    # 进入主循环
    while True:
        try:
            print("\n请输入控制对象 (lh: 左手, rh: 右手, la: 左臂, ra: 右臂，输入 'q' 退出)：")
            control_choice = input().strip().lower()
            if control_choice == 'q': break
            if control_choice not in ['lh', 'rh', 'la', 'ra']:
                print("输入错误，请输入 'lh', 'rh', 'la', 'ra' 或 'q'")
                continue

            if control_choice in ['lh', 'rh']:
                # ... (手部控制逻辑保持不变)
                hand_name = '左手' if control_choice == 'lh' else '右手'
                publisher = l_hand_publisher if control_choice == 'lh' else r_hand_publisher
                print(f"\n请输入 {hand_name} 动作 (0: 张开, 1: 闭合, 'q' 退出)：")
                hand_action = input().strip()
                if hand_action == 'q': break
                
                angles = TARGET_HAND_OPEN_ANGLES if hand_action == '0' else TARGET_HAND_GRASP_ANGLES
                control_hand_angle(publisher, angles, hand_name)
                time.sleep(1.0)

            else:
                # ... (手臂控制逻辑保持不变)
                arm_name = 'left_arm' if control_choice == 'la' else 'right_arm'
                model = l_model if control_choice == 'la' else r_model
                data = l_data if control_choice == 'la' else r_data
                # 关键：确保q是当前机械臂的最新状态
                q = l_q if control_choice == 'la' else r_q
                
                p0 = [0.11275, 0.0444465, -0.029845] if control_choice == 'la' else [0.11275, -0.0444465, -0.029845]
                target_rotation, _ = compute_zyx_transform(-65 if control_choice == 'la' else 65, 0, 0, p0)

                print("\n请输入相机坐标系下的目标位置 (x y z)，以空格分隔（'q' 退出）：")
                user_input = input().strip()
                if user_input.lower() == 'q': break
                
                try:
                    x, y, z = map(float, user_input.split())
                    cameramat = np.array([x, y, z, 1])
                    cameramat[2] += 0.05
                except ValueError:
                    print("输入格式错误。")
                    continue

                pin.forwardKinematics(model, data, q)
                _, camera2base = compute_zyx_transform(0, 65 if control_choice == 'la' else -65, 90, p0)
                p = camera2base @ cameramat
                pend = [-0.03, 0, 0.12]
                pact = p[:3] - target_rotation @ pend
                oMdes = pin.SE3(target_rotation, pact)

                q_new, success = move_to_pose(model, data, q, joint_id, oMdes, arm_name)
                
                if success:
                    print(f"\nIK求解成功，将通过MoveIt!执行运动...")
                    if move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG[control_choice], q_new):
                        print("运动执行成功。")
                        if control_choice == 'la': l_q = q_new
                        else: r_q = q_new
                    else:
                        print("MoveIt! 未能成功执行计划的轨迹。机械臂状态可能未改变。")
                else:
                    print("逆运动学未收敛，跳过 ROS 指令发布。")
        
        except (KeyboardInterrupt, EOFError):
            break

    print("\n正在关闭连接...")
    l_hand_publisher.unadvertise()
    r_hand_publisher.unadvertise()
    client.terminate()
    print("脚本执行完成。")

if __name__ == "__main__":
    main()