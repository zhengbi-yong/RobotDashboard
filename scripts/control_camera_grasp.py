#!/usr/bin/env python3
import numpy as np
import pinocchio as pin
from numpy.linalg import norm, solve
from pathlib import Path
import roslibpy
import roslibpy.actionlib
import time
import os
import traceback
import threading

# --- 用于MoveIt!阻塞调用的全局变量 ---
moveit_goal_completed = threading.Event()
moveit_result_success = False

# --- 核心运动学与控制函数 (保持不变) ---

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
    """计算单臂逆运动学。"""
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
            print(f"IK 收敛! 尝试次数: {attempt + 1}, 迭代次数: {iteration}, 最终误差: {final_error:.4f}")
            pin.forwardKinematics(model, data, q_new)
            return q_new, success
        else:
            print(f"IK 尝试 {attempt + 1} 失败, 迭代次数: {iteration}, 最终误差: {final_error:.4f}")
    
    pin.forwardKinematics(model, data, q_new)
    return q_new, False

def control_hand_angle(publisher, hand_angles, hand_name):
    """发送灵巧手抓握指令。"""
    message = roslibpy.Message({'hand_angle': hand_angles})
    print(f"发布 {hand_name} 抓握指令: hand_angle={hand_angles}")
    publisher.publish(message)
    time.sleep(2.0)
    return True

def move_arm_with_moveit(action_client, arm_config, target_joint_angles):
    """通过MoveIt!发送关节目标，并等待其完成（阻塞式）。"""
    global moveit_goal_completed, moveit_result_success

    if not action_client:
        print("[ERROR] MoveIt! Action Client 未初始化。")
        return False

    print(f"--- 正在通过MoveIt!发送关节目标 [{arm_config['group']}] ---")
    
    moveit_goal_completed.clear()
    moveit_result_success = False

    def _on_result(result):
        global moveit_result_success
        error_code = result['error_code']['val']
        if error_code == 1:
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
    
    print(f"      MoveIt! 目标已发送。正在等待执行完成...")
    completed_in_time = moveit_goal_completed.wait(timeout=30.0)

    if not completed_in_time:
        print("[ERROR] MoveIt! 目标执行超时！")
        goal.cancel()
        return False
        
    return moveit_result_success

def main():
    """主执行函数：定义并执行一个自动化的抓取序列。"""
    os.environ['ROS_DOMAIN_ID'] = '0'

    # --- 1. 配置参数 ---
    ROS_BRIDGE_HOST = '192.168.0.105'
    ROS_BRIDGE_PORT = 9090
    CONNECTION_TIMEOUT_SEC = 10

    MOVE_GROUP_ACTION_NAME = '/move_group'
    MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
    MOVEIT_CONFIG = {
        'la': {'group': 'l_arm', 'joints': [f'l_joint{i+1}' for i in range(7)]},
        'ra': {'group': 'r_arm', 'joints': [f'r_joint{i+1}' for i in range(7)]}
    }
    
    GRIP_POSES = {
        "open":   [1000, 1000, 1000, 1000, 1000, 1000],
        "grasp":  [0, 0, 0, 0, 0, 1000],
    }

    # --- 2. 加载 URDF 模型 ---
    try:
        l_urdf_filename = Path("./dual_75B_left_arm.urdf")
        r_urdf_filename = Path("./dual_75B_right_arm.urdf")
        l_model = pin.buildModelFromUrdf(str(l_urdf_filename))
        r_model = pin.buildModelFromUrdf(str(r_urdf_filename))
        print("左臂模型名称: " + l_model.name)
        print("右臂模型名称: " + r_model.name)
        l_data = l_model.createData()
        r_data = r_model.createData()
    except Exception as e:
        print(f"错误: 无法加载URDF模型文件。")
        print(f"详细信息: {e}")
        return

    l_q = pin.neutral(l_model)
    r_q = pin.neutral(r_model)
    l_home_q = np.zeros(7)
    r_home_q = np.zeros(7)
    
    # --- 3. ROS 初始化 ---
    client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
    moveit_action_client = None
    try:
        client.run()
        print(f"正在连接到 rosbridge 服务器 at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        start_time_conn = time.time()
        while not client.is_connected:
            time.sleep(0.1)
            if time.time() - start_time_conn > CONNECTION_TIMEOUT_SEC:
                raise TimeoutError(f"连接到 rosbridge 服务器超时")
        
        print("成功连接到 rosbridge 服务器")
        
        moveit_action_client = roslibpy.actionlib.ActionClient(
            client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE
        )
        print("MoveIt! Action Client 已初始化。")

    except Exception as e:
        print(f"错误: 连接或初始化ROS客户端失败: {e}")
        traceback.print_exc()
        if client and client.is_connected: client.terminate()
        return

    l_hand_publisher = roslibpy.Topic(client, '/l_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')
    r_hand_publisher = roslibpy.Topic(client, '/r_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')


    # --- 4. 定义并执行抓取序列 ---
    try:
        # === 步骤 0: 移动到初始Home姿态 ===
        print("\n>>> 步骤 0: 移动双臂到Home姿态...")
        # --- 【已修改的逻辑】 ---
        if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['ra'], r_home_q):
            print("【警告】右臂移动到Home失败（根据MoveIt!报告），但脚本将按要求继续。")
        if not move_arm_with_moveit(moveit_action_client, MOVEIT_CONFIG['la'], l_home_q):
            print("【警告】左臂移动到Home失败（根据MoveIt!报告），但脚本将按要求继续。")
        
        r_q = r_home_q.copy()
        l_q = l_home_q.copy()
        print("双臂已就位（或已尝试就位）。")
        
        # === 抓取任务参数 (使用右臂) ===
        arm_to_use = 'ra'
        hand_to_use_pub = r_hand_publisher
        model, data, q = r_model, r_data, r_q
        joint_id = 7
        arm_config = MOVEIT_CONFIG[arm_to_use]
        hand_name = "右手"

        target_pos_in_camera = np.array([0.0, 0.0, 0.4])
        print(f"\n目标物体位置 (相机坐标系): {target_pos_in_camera}")

        p0_base = [0.11275, -0.0444465, -0.029845]
        _, camera2base = compute_zyx_transform(0, -65, 90, p0_base)
        
        target_rotation, _ = compute_zyx_transform(65, 0, 0, [0,0,0])
        pend = np.array([-0.03, 0, 0.12])


        # === 步骤 1: 打开手爪 ===
        print(f"\n>>> 步骤 1: 打开 {hand_name}...")
        control_hand_angle(hand_to_use_pub, GRIP_POSES["open"], hand_name)

        # === 步骤 2: 移动到抓取预备点 (物体上方) ===
        print("\n>>> 步骤 2: 移动到抓取预备点...")
        pre_grasp_offset = np.array([0, 0, -0.1])
        p_pre_grasp_cam = target_pos_in_camera + pre_grasp_offset
        p_pre_grasp_world = camera2base @ np.append(p_pre_grasp_cam, 1)
        p_pre_grasp_act = p_pre_grasp_world[:3] - target_rotation @ pend
        oMdes_pre_grasp = pin.SE3(target_rotation, p_pre_grasp_act)

        q_new, ik_success = move_to_pose(model, data, q, joint_id, oMdes_pre_grasp, arm_config['group'])
        if not ik_success: raise RuntimeError("预备点IK求解失败，终止任务。")
        
        # --- 【已修改的逻辑】 ---
        if not move_arm_with_moveit(moveit_action_client, arm_config, q_new):
            print("【警告】移动到预备点失败（根据MoveIt!报告），但脚本将按要求继续。")
        q = q_new.copy()


        # === 步骤 3: 移动到抓取点 (下降) ===
        print("\n>>> 步骤 3: 移动到抓取点...")
        p_grasp_cam = target_pos_in_camera.copy()
        p_grasp_cam[2] += 0.05 
        p_grasp_world = camera2base @ np.append(p_grasp_cam, 1)
        p_grasp_act = p_grasp_world[:3] - target_rotation @ pend
        oMdes_grasp = pin.SE3(target_rotation, p_grasp_act)

        q_new, ik_success = move_to_pose(model, data, q, joint_id, oMdes_grasp, arm_config['group'])
        if not ik_success: raise RuntimeError("抓取点IK求解失败，终止任务。")
        
        # --- 【已修改的逻辑】 ---
        if not move_arm_with_moveit(moveit_action_client, arm_config, q_new):
            print("【警告】移动到抓取点失败（根据MoveIt!报告），但脚本将按要求继续。")
        q = q_new.copy()


        # === 步骤 4: 闭合手爪 (抓取) ===
        print(f"\n>>> 步骤 4: 闭合 {hand_name} (执行抓取)...")
        control_hand_angle(hand_to_use_pub, GRIP_POSES["grasp"], hand_name)

        # === 步骤 5: 抬升物体 ===
        print("\n>>> 步骤 5: 抬升物体...")
        lift_offset = np.array([0, 0, -0.15])
        p_lift_cam = target_pos_in_camera + lift_offset
        p_lift_world = camera2base @ np.append(p_lift_cam, 1)
        p_lift_act = p_lift_world[:3] - target_rotation @ pend
        oMdes_lift = pin.SE3(target_rotation, p_lift_act)
        
        q_new, ik_success = move_to_pose(model, data, q, joint_id, oMdes_lift, arm_config['group'])
        if not ik_success: raise RuntimeError("抬升点IK求解失败，终止任务。")
        
        # --- 【已修改的逻辑】 ---
        if not move_arm_with_moveit(moveit_action_client, arm_config, q_new):
            print("【警告】抬升物体失败（根据MoveIt!报告），但脚本将按要求继续。")
        q = q_new.copy()

        # === 步骤 6: 返回Home姿态 ===
        print("\n>>> 步骤 6: 抓取完成，返回Home姿态...")
        if not move_arm_with_moveit(moveit_action_client, arm_config, r_home_q):
            print("【警告】返回Home姿态失败（根据MoveIt!报告），但任务主体已完成。")
        q = r_home_q.copy()
        
        print("\n[SUCCESS] 抓取任务序列全部执行完毕！")

    except (RuntimeError, KeyboardInterrupt, EOFError, TimeoutError) as e:
        print(f"\n任务被中断或发生错误: {e}")
    finally:
        # --- 5. 清理和关闭 ---
        print("\n正在关闭连接...")
        if 'l_hand_publisher' in locals() and l_hand_publisher: l_hand_publisher.unadvertise()
        if 'r_hand_publisher' in locals() and r_hand_publisher: r_hand_publisher.unadvertise()
        if client and client.is_connected: client.terminate()
        print("脚本执行完成。")

if __name__ == "__main__":
    main()