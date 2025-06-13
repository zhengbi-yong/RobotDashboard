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

# --- 全局变量 ---
moveit_action_completed = threading.Event()
moveit_action_success = False

# --- 核心运动学与控制函数 (保持不变) ---
def compute_zyx_transform(alpha_deg, beta_deg, gamma_deg, p):
    alpha = np.deg2rad(alpha_deg); beta = np.deg2rad(beta_deg); gamma = np.deg2rad(gamma_deg)
    R_x = np.array([[1, 0, 0], [0, np.cos(alpha), -np.sin(alpha)], [0, np.sin(alpha), np.cos(alpha)]])
    R_y = np.array([[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]])
    R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0], [np.sin(gamma), np.cos(gamma), 0], [0, 0, 1]])
    R = R_z @ R_y @ R_x; p = np.array(p); H = np.eye(4)
    H[:3, :3] = R; H[:3, 3] = p
    return R, H

def control_hand_angle(publisher, hand_angles, hand_name):
    message = roslibpy.Message({'hand_angle': hand_angles})
    print(f"发布 {hand_name} 抓握指令: hand_angle={hand_angles}")
    publisher.publish(message)
    time.sleep(2.0)
    return True

def compute_single_arm_ik(model, data, q, oMdes, joint_id, eps=5e-3, it_max=5000, dt=0.2, damp=1e-5):
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
        v = -J.T.dot(solve(J.dot(J.T) + (damp if np.linalg.matrix_rank(J)==6 else 1e-3) * np.eye(6), err))
        q_new = pin.integrate(model, q, dt * v)
        for j in range(model.nq):
            q_new[j] = np.mod(q_new[j] + np.pi, 2 * np.pi) - np.pi
            if not np.isnan(lower_limits[j]) and not np.isnan(upper_limits[j]):
                q_new[j] = np.clip(q_new[j], lower_limits[j], upper_limits[j])
        q = q_new
        if norm(err) > prev_err: dt *= 0.5
        prev_err = norm(err)
        i += 1
    return q, False, i, norm(err)

def move_arm_to_joint_goal(action_client, arm_config, target_joint_angles, start_joint_angles=None):
    global moveit_action_completed, moveit_action_success
    if not action_client: return False
    
    print(f"--- 正在发送 [关节目标] 到 [{arm_config['group']}] ---")

    moveit_action_completed.clear()
    moveit_action_success = False
    
    result_container = {'value': None}
    def _on_result(result):
        global moveit_action_success
        result_container['value'] = result
        if result and result.get('error_code', {}).get('val') == 1:
            moveit_action_success = True
        else:
            moveit_action_success = False
        moveit_action_completed.set()

    request_msg = {
        'group_name': arm_config['group'],
        'goal_constraints': [{
            'joint_constraints': [
                {'joint_name': name, 'position': pos, 'tolerance_above': 0.01, 'tolerance_below': 0.01, 'weight': 1.0}
                for name, pos in zip(arm_config['joints'], target_joint_angles)
            ]
        }],
        'num_planning_attempts': 10, 'allowed_planning_time': 10.0
    }
    if start_joint_angles is not None:
        request_msg['start_state'] = {'joint_state': {'name': arm_config['joints'], 'position': start_joint_angles.tolist()}}

    goal = roslibpy.actionlib.Goal(action_client, {'request': request_msg, 'planning_options': {'plan_only': False, 'replan': False}})
    goal.on('result', _on_result)
    goal.send()
    
    print("      目标已发送，正在等待执行...")
    completed_in_time = moveit_action_completed.wait(timeout=30.0)

    if moveit_action_success:
        print("      执行结果: 成功")
    else:
        if not completed_in_time:
            status_str = "失败(执行超时)"
            goal.cancel()
        else:
            final_result = result_container['value']
            error_code = final_result.get('error_code', {}).get('val', 'N/A') if final_result else 'N/A'
            status_str = f"失败(代码: {error_code})"
        print(f"      执行结果: {status_str}")
    
    return moveit_action_success

def execute_cartesian_trajectory(moveit_client, arm_config, model, data, q_current, joint_id, waypoints_se3):
    print(f"\n>>> 开始执行包含 {len(waypoints_se3)} 个路径点的笛卡尔轨迹...")
    q = q_current.copy()
    for i, oMdes_waypoint in enumerate(waypoints_se3):
        print(f"  -- 路径点 {i+1}/{len(waypoints_se3)} --")
        
        q_new, ik_success, _, _ = compute_single_arm_ik(model, data, q, oMdes_waypoint, joint_id)
        if not ik_success:
            print(f"【错误】路径点IK求解失败！无法继续执行轨迹。")
            return False, q
            
        move_success = move_arm_to_joint_goal(moveit_client, arm_config, q_new, start_joint_angles=q)
        if not move_success:
            print(f"【警告】移动到此路径点失败，但脚本将继续。")

        q = q_new.copy()
        time.sleep(0.1)
        
    print(">>> 笛卡尔轨迹执行完毕。")
    return True, q

def main():
    os.environ['ROS_DOMAIN_ID'] = '0'
    ROS_BRIDGE_HOST = '192.168.0.105'
    ROS_BRIDGE_PORT = 9090
    CONNECTION_TIMEOUT_SEC = 10
    MOVE_GROUP_ACTION_NAME = '/move_group'
    MOVE_GROUP_ACTION_TYPE = 'moveit_msgs/MoveGroupAction'
    MOVEIT_CONFIG = {
        'la': {'group': 'l_arm', 'joints': [f'l_joint{i+1}' for i in range(7)]},
        'ra': {'group': 'r_arm', 'joints': [f'r_joint{i+1}' for i in range(7)]}
    }
    GRIP_POSES = { "open": [1000]*6, "grasp": [0]*5 + [1000] }

    client = None
    try:
        # --- 模型加载 ---
        l_urdf_filename = Path("./dual_75B_left_arm.urdf"); r_urdf_filename = Path("./dual_75B_right_arm.urdf")
        l_model = pin.buildModelFromUrdf(str(l_urdf_filename)); r_model = pin.buildModelFromUrdf(str(r_urdf_filename))
        print(f"左臂模型: {l_model.name}, 右臂模型: {r_model.name}")
        
        # 【已修正】将丢失的 l_data 和 r_data 创建语句加回来
        l_data = l_model.createData()
        r_data = r_model.createData()

        # --- ROS 连接 ---
        client = roslibpy.Ros(host=ROS_BRIDGE_HOST, port=ROS_BRIDGE_PORT)
        client.run()
        print(f"正在连接到 rosbridge at {ROS_BRIDGE_HOST}:{ROS_BRIDGE_PORT}...")
        start_time_conn = time.time()
        while not client.is_connected:
            time.sleep(0.1)
            if time.time() - start_time_conn > CONNECTION_TIMEOUT_SEC:
                raise TimeoutError("连接 rosbridge 超时")
        print("连接成功。")
        
        moveit_action_client = roslibpy.actionlib.ActionClient(client, MOVE_GROUP_ACTION_NAME, MOVE_GROUP_ACTION_TYPE)
        print("MoveIt! Action Client 已初始化。")
        l_hand_publisher = roslibpy.Topic(client, '/l_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')
        r_hand_publisher = roslibpy.Topic(client, '/r_arm/rm_driver/Hand_SetAngle', 'dual_arm_msgs/Hand_Angle')
    except Exception as e:
        print(f"初始化失败: {e}"); traceback.print_exc()
        if client and client.is_connected: client.terminate()
        return

    try:
        l_home_q = np.zeros(7); r_home_q = np.zeros(7)
        
        print("\n>>> 步骤 0: 归位...")
        move_arm_to_joint_goal(moveit_action_client, MOVEIT_CONFIG['ra'], r_home_q)
        move_arm_to_joint_goal(moveit_action_client, MOVEIT_CONFIG['la'], l_home_q)
        r_q = r_home_q.copy(); l_q = l_home_q.copy()
        
        # --- 任务参数 ---
        arm_to_use, hand_to_use_pub = 'ra', r_hand_publisher
        model, data, q = r_model, r_data, r_q
        arm_config = MOVEIT_CONFIG[arm_to_use]; hand_name = "右手"
        joint_id = 7
        target_pos_in_camera = np.array([0.0, 0.0, 0.4])
        p0_base = [0.11275, -0.0444465, -0.029845]
        _, camera2base = compute_zyx_transform(0, -65, 90, p0_base)
        target_rotation, _ = compute_zyx_transform(65, 0, 0, [0,0,0])
        pend = np.array([-0.03, 0, 0.12])

        print(f"\n>>> 步骤 1: 打开 {hand_name}...")
        control_hand_angle(hand_to_use_pub, GRIP_POSES["open"], hand_name)

        p_pre_grasp_world = (camera2base @ np.append(target_pos_in_camera + np.array([0,0,-0.1]), 1))[:3]
        p_pre_grasp_act = p_pre_grasp_world - target_rotation @ pend
        oMdes_pre_grasp = pin.SE3(target_rotation, p_pre_grasp_act)

        p_grasp_world = (camera2base @ np.append(target_pos_in_camera + np.array([0,0,0.05]), 1))[:3]
        p_grasp_act = p_grasp_world - target_rotation @ pend
        oMdes_grasp = pin.SE3(target_rotation, p_grasp_act)

        print("\n>>> 步骤 2: 移动到抓取预备点...")
        q_pre_grasp, ik_success, _, _ = compute_single_arm_ik(model, data, q, oMdes_pre_grasp, joint_id)
        if not ik_success: raise RuntimeError("预备点IK求解失败")
        move_arm_to_joint_goal(moveit_action_client, arm_config, q_pre_grasp, start_joint_angles=q)
        q = q_pre_grasp.copy()
        
        descent_waypoints = [pin.SE3.Interpolate(oMdes_pre_grasp, oMdes_grasp, t) for t in np.linspace(0, 1, 10)]
        success, q = execute_cartesian_trajectory(moveit_action_client, arm_config, model, data, q, joint_id, descent_waypoints)
        if not success: raise RuntimeError("下降轨迹执行失败！")

        print(f"\n>>> 步骤 4: 闭合 {hand_name}...")
        control_hand_angle(hand_to_use_pub, GRIP_POSES["grasp"], hand_name)

        p_lift_world = (camera2base @ np.append(target_pos_in_camera + np.array([0,0,-0.15]), 1))[:3]
        p_lift_act = p_lift_world - target_rotation @ pend
        oMdes_lift = pin.SE3(target_rotation, p_lift_act)
        lift_waypoints = [pin.SE3.Interpolate(oMdes_grasp, oMdes_lift, t) for t in np.linspace(0, 1, 10)]
        success, q = execute_cartesian_trajectory(moveit_action_client, arm_config, model, data, q, joint_id, lift_waypoints)
        if not success: raise RuntimeError("抬升轨迹执行失败！")

        print("\n>>> 步骤 6: 返回Home...")
        move_arm_to_joint_goal(moveit_action_client, arm_config, r_home_q, start_joint_angles=q)
        
        print("\n[SUCCESS] 任务序列全部执行完毕！")

    except Exception as e:
        print(f"\n任务被中断或发生错误: {e}"); traceback.print_exc()
    finally:
        print("\n正在关闭连接...")
        if client and client.is_connected: client.terminate()
        print("脚本执行完成。")

if __name__ == "__main__":
    main()