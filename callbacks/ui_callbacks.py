# RobotDashboard/callbacks/ui_callbacks.py

import dash
from dash import html, no_update, Input, Output, State, callback_context
import dash_bootstrap_components as dbc
import roslibpy
import json
import time
import math
import traceback

from .. import config
from ..ros_comms import handler as ros_handler
from ..utils import trajectory_manager as traj_manager

_last_moveit_result_ui_timestamp = 0
_last_moveit_error_code_ui = None

def register_callbacks(app):
    @app.callback(
        Output("ros-connection-status-display", "children"),
        Output("connect-ros-button", "disabled"),
        Input("connect-ros-button", "n_clicks"),
        Input("interval-ros-status-update", "n_intervals")
    )
    def handle_ros_connection(n_clicks_connect, n_intervals_status):
        ctx = dash.callback_context
        triggered_id = ctx.triggered_id
        
        button_disabled_flag = False
        current_status_display = f"状态: {ros_handler.ros_connection_status}"

        if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
            button_disabled_flag = True
        elif ros_handler.ros_client and ros_handler.ros_client.is_connected:
            if ros_handler.ros_setup_done:
                current_status_display = f"状态: 已连接到 {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT} (设置完毕)"
            else:
                current_status_display = f"状态: 已连接到 {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT} (订阅器设置中...)"
            button_disabled_flag = True
        
        if triggered_id == "connect-ros-button" and n_clicks_connect is not None:
            if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
                print("连接尝试已在进行中。")
                return current_status_display, True

            if ros_handler.ros_client and ros_handler.ros_client.is_connected:
                print("ROS 客户端已连接。正在终止现有连接以重新连接。")
                ros_handler.safe_terminate_ros_client()
                time.sleep(1.0)

            ros_handler.try_connect_ros()
            current_status_display = f"状态: {ros_handler.ros_connection_status}"
            button_disabled_flag = True
            
        return current_status_display, button_disabled_flag

    @app.callback(
        Output("action-feedback-display", "children", allow_duplicate=True),
        Input("interval-ros-status-update", "n_intervals"),
        prevent_initial_call=True
    )
    def display_moveit_feedback(n_intervals):
        global _last_moveit_result_ui_timestamp, _last_moveit_error_code_ui

        current_moveit_result = ros_handler.get_latest_moveit_result()
        if current_moveit_result['timestamp'] > _last_moveit_result_ui_timestamp:
            _last_moveit_result_ui_timestamp = current_moveit_result['timestamp']
            _last_moveit_error_code_ui = current_moveit_result['error_code']

            if _last_moveit_error_code_ui == 1:
                return html.Div("MoveIt! 目标已成功规划并执行。", className="alert alert-success")
            elif _last_moveit_error_code_ui is not None:
                error_map = {
                    -1: "规划失败",
                    -2: "执行失败",
                    -3: "无效目标",
                }
                error_msg = error_map.get(_last_moveit_error_code_ui, f"未知错误代码: {_last_moveit_error_code_ui}")
                return html.Div(f"MoveIt! 目标失败: {error_msg} (代码: {_last_moveit_error_code_ui})", className="alert alert-danger")
        return no_update


    # --- 用下面这个完整的代码块替换您现有的 send_control_commands 函数 ---
    @app.callback(
        Output("action-feedback-display", "children", allow_duplicate=True),
        [Input("send-left-arm-button", "n_clicks"),
        Input("send-right-arm-button", "n_clicks"),
        Input("send-head-servo-button", "n_clicks"),
        Input("send-left-hand-button", "n_clicks"),
        Input("send-right-hand-button", "n_clicks"),
        Input("send-nav-command-button", "n_clicks"),
        Input("send-pose-goal-button", "n_clicks")],  # 7个Inputs
        [State(f"l_arm_slider_{i}", "value") for i in range(7)] +
        [State(f"r_arm_slider_{i}", "value") for i in range(7)] +
        [State("head-tilt-slider", "value"), State("head-pan-slider", "value")] +
        [State(f"l_hand_slider_{i}", "value") for i in range(len(config.LEFT_HAND_DOF_NAMES))] +
        [State(f"r_hand_slider_{i}", "value") for i in range(len(config.RIGHT_HAND_DOF_NAMES))] +
        [State("nav-point-dropdown", "value")] +
        [State("playback-speed-slider", "value")] +
        [State("pose-control-arm-select", "value"),
        State("pose-pos-x", "value"), State("pose-pos-y", "value"), State("pose-pos-z", "value"),
        State("pose-ori-w", "value"), State("pose-ori-x", "value"), State("pose-ori-y", "value"), State("pose-ori-z", "value")], # 38个States
        prevent_initial_call=True
    )
    def send_control_commands(n_l_arm, n_r_arm, n_head, n_l_hand, n_r_hand, n_nav, n_pose, # 7个n_clicks参数
                            # Arm Sliders (14)
                            l_arm_s0, l_arm_s1, l_arm_s2, l_arm_s3, l_arm_s4, l_arm_s5, l_arm_s6,
                            r_arm_s0, r_arm_s1, r_arm_s2, r_arm_s3, r_arm_s4, r_arm_s5, r_arm_s6,
                            # Head Sliders (2)
                            head_tilt_val, head_pan_val,
                            # Hand Sliders (假设左右各6个, 共12个)
                            l_hand_s0, l_hand_s1, l_hand_s2, l_hand_s3, l_hand_s4, l_hand_s5,
                            r_hand_s0, r_hand_s1, r_hand_s2, r_hand_s3, r_hand_s4, r_hand_s5,
                            # Other controls (2)
                            selected_nav_point,
                            playback_speed_for_moveit,
                            # New Pose Controls (8)
                            pose_arm_select,
                            pos_x, pos_y, pos_z,
                            ori_w, ori_x, ori_y, ori_z
                            ):
        # (函数体保持不变)
        ctx = callback_context
        button_id = ctx.triggered_id

        if not button_id:
            return no_update

        left_arm_angles_deg = [l_arm_s0, l_arm_s1, l_arm_s2, l_arm_s3, l_arm_s4, l_arm_s5, l_arm_s6]
        right_arm_angles_deg = [r_arm_s0, r_arm_s1, r_arm_s2, r_arm_s3, r_arm_s4, r_arm_s5, r_arm_s6]
        left_hand_angles_raw = [l_hand_s0, l_hand_s1, l_hand_s2, l_hand_s3, l_hand_s4, l_hand_s5]
        right_hand_angles_raw = [r_hand_s0, r_hand_s1, r_hand_s2, r_hand_s3, r_hand_s4, r_hand_s5]

        left_hand_angles_int = [int(a) for a in left_hand_angles_raw]
        right_hand_angles_int = [int(a) for a in right_hand_angles_raw]

        feedback_msg = "未知错误"
        try:
            if not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done) and button_id != "connect-ros-button":
                return html.Div("ROS 未连接或设置未完成。无法发送指令。", className="alert alert-danger")

            if button_id == "send-left-arm-button":
                if ros_handler.moveit_action_client:
                    target_joint_angles_rad = [math.radians(deg) for deg in left_arm_angles_deg]
                    ros_handler.send_moveit_joint_goal(
                        config.PLANNING_GROUP_LEFT_ARM,
                        config.LEFT_ARM_JOINT_NAMES_INTERNAL,
                        target_joint_angles_rad,
                        velocity_scaling_factor=playback_speed_for_moveit
                    )
                    feedback_msg = "左臂 MoveIt! 目标已发送！等待规划和执行结果..."
                    ros_handler.reset_latest_moveit_result()
                else: return html.Div("MoveIt! Action Client 不可用。", className="alert alert-warning")
            elif button_id == "send-right-arm-button":
                if ros_handler.moveit_action_client:
                    target_joint_angles_rad = [math.radians(deg) for deg in right_arm_angles_deg]
                    ros_handler.send_moveit_joint_goal(
                        config.PLANNING_GROUP_RIGHT_ARM,
                        config.RIGHT_ARM_JOINT_NAMES_INTERNAL,
                        target_joint_angles_rad,
                        velocity_scaling_factor=playback_speed_for_moveit
                    )
                    feedback_msg = "右臂 MoveIt! 目标已发送！等待规划和执行结果..."
                    ros_handler.reset_latest_moveit_result()
                else: return html.Div("MoveIt! Action Client 不可用。", className="alert alert-warning")
            
            elif button_id == "send-pose-goal-button":
                if any(v is None for v in [pos_x, pos_y, pos_z, ori_w, ori_x, ori_y, ori_z]):
                    return html.Div("错误: 请填写所有位置和姿态的输入框。", className="alert alert-danger")
                if not pose_arm_select:
                    return html.Div("错误: 请选择一个控制组 (左臂/右臂)。", className="alert alert-danger")
                try:
                    pose_data = {
                        "position": {"x": float(pos_x), "y": float(pos_y), "z": float(pos_z)},
                        "orientation": {"w": float(ori_w), "x": float(ori_x), "y": float(ori_y), "z": float(ori_z)}
                    }
                except (ValueError, TypeError):
                    return html.Div("错误: 位姿输入值必须是有效的数字。", className="alert alert-danger")
                ros_handler.send_moveit_pose_goal(
                    planning_group=pose_arm_select,
                    pose_data=pose_data,
                    velocity_scaling_factor=playback_speed_for_moveit
                )
                feedback_msg = f"位姿目标已发送至 {pose_arm_select}！等待规划和执行结果..."
                ros_handler.reset_latest_moveit_result()

            elif button_id == "send-head-servo-button":
                if ros_handler.head_servo_pub:
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_val)}))
                    time.sleep(0.05)
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_val)}))
                    ros_handler.latest_joint_states['head_tilt_servo'] = float(head_tilt_val)
                    ros_handler.latest_joint_states['head_pan_servo'] = float(head_pan_val)
                    feedback_msg = "头部伺服指令已发送!"
                else: return html.Div("头部伺服 Publisher 不可用。", className="alert alert-warning")

            elif button_id == "send-left-hand-button":
                if ros_handler.left_hand_pub:
                    msg_data = {'hand_angle': left_hand_angles_int}
                    ros_handler.left_hand_pub.publish(roslibpy.Message(msg_data))
                    for i, dof_name in enumerate(config.LEFT_HAND_DOF_NAMES):
                        ros_handler.latest_joint_states[dof_name] = float(left_hand_angles_int[i])
                    feedback_msg = "左手指令已发送!"
                else: return html.Div("左手 Publisher 不可用。", className="alert alert-warning")

            elif button_id == "send-right-hand-button":
                if ros_handler.right_hand_pub:
                    msg_data = {'hand_angle': right_hand_angles_int}
                    ros_handler.right_hand_pub.publish(roslibpy.Message(msg_data))
                    for i, dof_name in enumerate(config.RIGHT_HAND_DOF_NAMES):
                        ros_handler.latest_joint_states[dof_name] = float(right_hand_angles_int[i])
                    feedback_msg = "右手指令已发送!"
                else: return html.Div("右手 Publisher 不可用。", className="alert alert-warning")
            
            elif button_id == "send-nav-command-button":
                if not selected_nav_point:
                    return html.Div("请选择一个导航目标点。", className="alert alert-warning")
                if ros_handler.nav_pub:
                    msg_data = {'data': selected_nav_point}
                    ros_handler.nav_pub.publish(roslibpy.Message(msg_data))
                    feedback_msg = f"导航指令 '{selected_nav_point}' 已发送!"
                else: return html.Div("导航 Publisher 不可用。", className="alert alert-warning")
            
            return html.Div(feedback_msg, className="alert alert-success")

        except Exception as e:
            print(f"发送指令时出错 ({button_id}): {e}")
            traceback.print_exc()
            return html.Div(f"发送指令错误: {e}", className="alert alert-danger")

    # 这是修复后的正确代码，请用它替换上面的旧函数

    @app.callback(
        # 关键改动：Output从一个手臂组件变成了两个独立的组件
        Output("left-arm-states-display", "children"),
        Output("right-arm-states-display", "children"),
        Output("head-states-display", "children"),
        # Input保持不变
        Input("interval-joint-state-update", "n_intervals"),
        Input("refresh-states-button", "n_clicks")
    )
    def update_joint_state_display(n_intervals, n_refresh):
        # 从ros_handler获取最新的完整状态字典
        latest_states = ros_handler.latest_joint_states

        # --- 为左臂构建显示字符串 ---
        # 新布局的标签是 (rad)，所以我们直接显示弧度值
        left_arm_lines = []
        for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL:
            state = latest_states.get(name, 0.0)
            left_arm_lines.append(f"{name}: {state:.4f}") # 使用4位小数的弧度值
        left_arm_str = "\n".join(left_arm_lines)
        
        # --- 为右臂构建显示字符串 ---
        right_arm_lines = []
        for name in config.RIGHT_ARM_JOINT_NAMES_INTERNAL:
            state = latest_states.get(name, 0.0)
            right_arm_lines.append(f"{name}: {state:.4f}")
        right_arm_str = "\n".join(right_arm_lines)

        # --- 为头部构建显示字符串 ---
        head_lines = []
        for name in config.HEAD_SERVO_RANGES.keys():
            state = latest_states.get(name, "N/A")
            head_lines.append(f"{name}: {state}")
        head_str = "\n".join(head_lines)

        # 关键改动：返回三个值，与上面的三个Output一一对应
        return left_arm_str, right_arm_str, head_str


    @app.callback(
        [Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        [Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        [Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True)] +
        [Output(f"l_hand_slider_{i}", "value", allow_duplicate=True) for i in range(len(config.LEFT_HAND_DOF_NAMES))] +
        [Output(f"r_hand_slider_{i}", "value", allow_duplicate=True) for i in range(len(config.RIGHT_HAND_DOF_NAMES))] +
        [Output("action-feedback-display", "children", allow_duplicate=True)],
        Input("sync-sliders-to-state-button", "n_clicks"),
        prevent_initial_call=True
    )
    def sync_sliders_to_robot_state(n_clicks_sync):
        if not (ros_handler.ros_client and ros_handler.ros_client.is_connected):
            num_sliders = 14 + 2 + len(config.LEFT_HAND_DOF_NAMES) + len(config.RIGHT_HAND_DOF_NAMES)
            return [no_update] * num_sliders + [html.Div("ROS 未连接。", className="alert alert-warning")]

        l_arm_vals = [ros_handler.latest_joint_states.get(name, 0.0) * (180.0 / math.pi) for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
        r_arm_vals = [ros_handler.latest_joint_states.get(name, 0.0) * (180.0 / math.pi) for name in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
        head_tilt_val = ros_handler.latest_joint_states.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
        head_pan_val = ros_handler.latest_joint_states.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
        
        l_hand_vals = [ros_handler.latest_joint_states.get(name, config.DEFAULT_HAND_ANGLE) for name in config.LEFT_HAND_DOF_NAMES]
        r_hand_vals = [ros_handler.latest_joint_states.get(name, config.DEFAULT_HAND_ANGLE) for name in config.RIGHT_HAND_DOF_NAMES]

        all_slider_updates = l_arm_vals + r_arm_vals + [head_tilt_val, head_pan_val] + l_hand_vals + r_hand_vals
        feedback = html.Div("滑块已同步至当前机器人状态/指令值。", className="alert alert-info")
        return *all_slider_updates, feedback

    @app.callback(
        Output("trajectory-select-dropdown", "options"),
        [Input("refresh-trajectory-list-button", "n_clicks"),
         Input("save-trajectory-button", "n_clicks")]
    )
    def update_trajectory_dropdown(n_refresh, n_save):
        return traj_manager.get_trajectory_files_options()

    @app.callback(
        [Output("action-feedback-display", "children", allow_duplicate=True),
         Output("recorded-positions-count-display", "children", allow_duplicate=True),
         Output("recorded-positions-list-display", "children", allow_duplicate=True),
         Output("playback-state-store", "data", allow_duplicate=True)],
        [Input({'type': 'delete-trajectory-point-button', 'index': dash.ALL}, "n_clicks")],
        [State("playback-state-store", "data")],
        prevent_initial_call=True
    )
    def handle_delete_trajectory_point(delete_button_n_clicks, current_playback_state):
        ctx = callback_context
        if not ctx.triggered_id or not any(delete_button_n_clicks):
            return no_update, no_update, no_update, no_update

        if isinstance(ctx.triggered_id, dict) and ctx.triggered_id.get("type") == 'delete-trajectory-point-button':
            point_index_to_delete = ctx.triggered_id['index']
        else:
            print(f"Warning: handle_delete_trajectory_point triggered by unexpected id: {ctx.triggered_id}")
            return no_update, no_update, no_update, no_update

        feedback_msg, feedback_type = traj_manager.delete_position_from_trajectory(point_index_to_delete)
        active_trajectory = traj_manager.get_current_trajectory()
        new_playback_store_data = current_playback_state.copy()
        new_playback_store_data['trajectory_for_repeat'] = active_trajectory
        new_playback_store_data['current_index'] = 0

        count_display_text = f"活跃轨迹中包含 {len(active_trajectory)} 个位置点。" if active_trajectory else "活跃轨迹为空。"
        list_display_children = traj_manager.get_display_for_recorded_positions_list(active_trajectory)
        
        return html.Div(feedback_msg, className=f"alert alert-{feedback_type} mt-2"), count_display_text, list_display_children, new_playback_store_data

    @app.callback(
        [Output("action-feedback-display", "children", allow_duplicate=True),
         Output("recorded-positions-count-display", "children", allow_duplicate=True),
         Output("recorded-positions-list-display", "children", allow_duplicate=True),
         Output("playback-state-store", "data", allow_duplicate=True)],
        [Input("record-position-button", "n_clicks"),
         Input("save-trajectory-button", "n_clicks"),
         Input("load-selected-trajectory-button", "n_clicks")],
        [State("trajectory-filename-input", "value"),
         State("trajectory-select-dropdown", "value"),
         State("playback-state-store", "data"),
         *[State(f"l_hand_slider_{i}", "value") for i in range(len(config.LEFT_HAND_DOF_NAMES))],
         *[State(f"r_hand_slider_{i}", "value") for i in range(len(config.RIGHT_HAND_DOF_NAMES))]
        ],
        prevent_initial_call=True
    )
    def handle_record_save_load_trajectory(n_record, n_save, n_load,
                                           trajectory_filename, selected_trajectory_path,
                                           current_playback_state,
                                           *hand_slider_values
                                           ):
        ctx = callback_context
        button_id = ctx.triggered_id
        if not button_id:
             return no_update, no_update, no_update, no_update

        feedback_msg = ""
        feedback_type = "info"
        new_playback_store_data = current_playback_state.copy()
        active_trajectory_after_action = traj_manager.get_current_trajectory()


        num_left_hand_dofs = len(config.LEFT_HAND_DOF_NAMES)
        left_hand_ui_values = list(hand_slider_values[:num_left_hand_dofs])
        right_hand_ui_values = list(hand_slider_values[num_left_hand_dofs:])


        if button_id == "record-position-button":
            if not n_record: return no_update, no_update, no_update, no_update
            
            feedback_msg = traj_manager.record_current_position(
                ros_handler.latest_joint_states,
                left_hand_ui_values,            
                right_hand_ui_values            
            )
            active_trajectory_after_action = traj_manager.get_current_trajectory()
            feedback_type = "success"
        elif button_id == "save-trajectory-button":
            if not n_save: return no_update, no_update, no_update, no_update
            feedback_msg, feedback_type = traj_manager.save_trajectory(trajectory_filename)
            active_trajectory_after_action = traj_manager.get_current_trajectory()
        elif button_id == "load-selected-trajectory-button":
            if not n_load: return no_update, no_update, no_update, no_update
            feedback_msg, feedback_type, loaded_traj = traj_manager.load_trajectory(selected_trajectory_path)
            active_trajectory_after_action = loaded_traj
        else:
            return no_update, no_update, no_update, no_update

        new_playback_store_data['trajectory_for_repeat'] = active_trajectory_after_action
        new_playback_store_data['current_index'] = 0

        count_display_text = f"活跃轨迹中包含 {len(active_trajectory_after_action)} 个位置点。" if active_trajectory_after_action else "活跃轨迹为空。"
        list_display_children = traj_manager.get_display_for_recorded_positions_list(active_trajectory_after_action)
        
        return html.Div(feedback_msg, className=f"alert alert-{feedback_type} mt-2"), count_display_text, list_display_children, new_playback_store_data
    
    @app.callback(
        Output("action-feedback-display", "children", allow_duplicate=True),
        Input("replay-once-button", "n_clicks"),
        [State("playback-speed-slider", "value"), State("playback-delay-slider", "value")],
        prevent_initial_call=True
    )
    def handle_replay_once(n_clicks_replay, playback_speed, point_delay_sec):
        if not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
            return html.Div("ROS 未连接或未完成设置，无法回放。", className="alert alert-danger")
        
        current_trajectory = traj_manager.get_current_trajectory()
        if not current_trajectory:
            return html.Div("活跃轨迹中无位置点可回放。", className="alert alert-warning")

        print(f"开始单次回放 {len(current_trajectory)} 个位置点...")
        feedback_msgs = []
        try:
            for i, pos_data in enumerate(current_trajectory):
                if ros_handler.moveit_action_client:
                    left_targets_rad = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                    right_targets_rad = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                    
                    try:
                        ros_handler.send_moveit_joint_goal(
                            config.PLANNING_GROUP_LEFT_ARM,
                            config.LEFT_ARM_JOINT_NAMES_INTERNAL,
                            left_targets_rad,
                            velocity_scaling_factor=playback_speed
                        )
                        ros_handler.send_moveit_joint_goal(
                            config.PLANNING_GROUP_RIGHT_ARM,
                            config.RIGHT_ARM_JOINT_NAMES_INTERNAL,
                            right_targets_rad,
                            velocity_scaling_factor=playback_speed
                        )
                        feedback_msgs.append(f"点 {i+1}: 已发送 MoveIt! 手臂目标。")
                        ros_handler.reset_latest_moveit_result()
                    except Exception as e:
                        print(f"Error sending MoveIt! goal during replay (point {i+1}): {e}")
                        feedback_msgs.append(f"点 {i+1}: 发送 MoveIt! 目标失败: {e}")
                else:
                    feedback_msgs.append(f"点 {i+1}: MoveIt! Action Client 未就绪，无法回放手臂。")

                if ros_handler.head_servo_pub:
                    head_tilt_target = pos_data.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
                    head_pan_target = pos_data.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
                    try:
                        ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_target)}))
                        time.sleep(0.02)
                        ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_target)}))
                        feedback_msgs.append(f"点 {i+1}: 已发送头部伺服指令。")
                    except Exception as e:
                        feedback_msgs.append(f"点 {i+1}: 发送头部伺服指令失败: {e}")
                
                if ros_handler.left_hand_pub:
                    left_hand_targets_int = [int(pos_data.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.LEFT_HAND_DOF_NAMES]
                    try:
                        ros_handler.left_hand_pub.publish(roslibpy.Message({'hand_angle': left_hand_targets_int}))
                        feedback_msgs.append(f"点 {i+1}: 已发送左手指令。")
                    except Exception as e:
                        feedback_msgs.append(f"点 {i+1}: 发送左手指令失败: {e}")
                if ros_handler.right_hand_pub:
                    right_hand_targets_int = [int(pos_data.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.RIGHT_HAND_DOF_NAMES]
                    try:
                        ros_handler.right_hand_pub.publish(roslibpy.Message({'hand_angle': right_hand_targets_int}))
                        feedback_msgs.append(f"点 {i+1}: 已发送右手指令。")
                    except Exception as e:
                        feedback_msgs.append(f"点 {i+1}: 发送右手指令失败: {e}")

                time.sleep(max(0.1, point_delay_sec))
            
            print("单次回放完成。")
            return html.Div([html.P("单次回放完成。"), html.Ul([html.Li(msg) for msg in feedback_msgs])], className="alert alert-success")
        except Exception as e:
            print(f"回放过程中发生错误: {e}")
            return html.Div(f"回放错误: {e}", className="alert alert-danger")

    @app.callback(
        [Output("continuous-playback-interval", "disabled"),
         Output("continuous-playback-interval", "interval"),
         Output("start-continuous-replay-button", "disabled"),
         Output("stop-continuous-replay-button", "disabled"),
         Output("playback-state-store", "data", allow_duplicate=True),
         Output("action-feedback-display", "children", allow_duplicate=True)],
        [Input("start-continuous-replay-button", "n_clicks"), Input("stop-continuous-replay-button", "n_clicks")],
        [State("playback-delay-slider", "value"), State("playback-state-store", "data")],
        prevent_initial_call=True
    )
    def manage_continuous_replay_controls(n_start, n_stop, delay_sec_state, current_p_state_manage):
        ctx = callback_context; button_id = ctx.triggered_id
        new_p_state_manage = current_p_state_manage.copy()
        feedback_text, feedback_type = "", "info"
        interval_disabled = True; interval_ms = max(100, int(delay_sec_state * 1000))
        start_button_disabled = False; stop_button_disabled = True

        if button_id == "start-continuous-replay-button":
            if not n_start: return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, no_update
            if not new_p_state_manage.get('trajectory_for_repeat'):
                feedback_text, feedback_type = "无活跃轨迹可供连续回放。", "warning"
            elif not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
                feedback_text, feedback_type = "ROS未就绪，无法开始连续回放。", "danger"
            elif not ros_handler.moveit_action_client:
                feedback_text, feedback_type = "MoveIt! Action Client 未就绪，无法开始连续回放。", "danger"
            else:
                new_p_state_manage['is_repeating'] = True; new_p_state_manage['current_index'] = 0
                interval_disabled = False;
                start_button_disabled = True; stop_button_disabled = False
                feedback_text, feedback_type = "连续回放已开始。", "success"
                ros_handler.reset_latest_moveit_result()
        elif button_id == "stop-continuous-replay-button":
            if not n_stop: return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, no_update
            new_p_state_manage['is_repeating'] = False
            feedback_text, feedback_type = "连续回放已停止。", "info"
        
        interval_ms = max(100, int(delay_sec_state * 1000))
        return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, html.Div(feedback_text,className=f"alert alert-{feedback_type}")


    @app.callback(
        [Output("playback-state-store", "data", allow_duplicate=True),
         Output("action-feedback-display", "children", allow_duplicate=True),
         *[Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         *[Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True),
         *[Output(f"l_hand_slider_{i}", "value", allow_duplicate=True) for i in range(len(config.LEFT_HAND_DOF_NAMES))],
         *[Output(f"r_hand_slider_{i}", "value", allow_duplicate=True) for i in range(len(config.RIGHT_HAND_DOF_NAMES))]
        ],
        Input("continuous-playback-interval", "n_intervals"),
        [State("playback-state-store", "data"), State("playback-speed-slider", "value")],
        prevent_initial_call=True
    )
    def execute_continuous_playback_step(n_intervals_playback, current_p_state_exec, playback_speed_exec):
        num_arm_head_sliders = 14 + 2
        num_hand_sliders = len(config.LEFT_HAND_DOF_NAMES) + len(config.RIGHT_HAND_DOF_NAMES)
        total_sliders = num_arm_head_sliders + num_hand_sliders
        slider_no_updates = [no_update] * total_sliders

        if not current_p_state_exec.get('is_repeating') or \
           not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done) or \
           not ros_handler.moveit_action_client:
            return current_p_state_exec, no_update, *slider_no_updates

        trajectory_to_play = current_p_state_exec.get('trajectory_for_repeat', [])
        if not trajectory_to_play:
            return current_p_state_exec, html.Div("连续回放错误：轨迹为空。", className="alert alert-warning"), *slider_no_updates

        current_idx = current_p_state_exec.get('current_index', 0)
        if current_idx >= len(trajectory_to_play): current_idx = 0

        pos_data_exec = trajectory_to_play[current_idx]
        
        l_arm_s_vals = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
        r_arm_s_vals = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
        tilt_s_val = pos_data_exec.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
        pan_s_val = pos_data_exec.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
        l_hand_s_vals = [pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE) for dof in config.LEFT_HAND_DOF_NAMES]
        r_hand_s_vals = [pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE) for dof in config.RIGHT_HAND_DOF_NAMES]
        slider_updates_exec = l_arm_s_vals + r_arm_s_vals + [tilt_s_val, pan_s_val] + l_hand_s_vals + r_hand_s_vals

        feedback_message_exec = f"连续回放: 点 {current_idx + 1}/{len(trajectory_to_play)}"
        try:
            if ros_handler.moveit_action_client:
                left_targets_rad_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                right_targets_rad_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                
                ros_handler.send_moveit_joint_goal(
                    config.PLANNING_GROUP_LEFT_ARM,
                    config.LEFT_ARM_JOINT_NAMES_INTERNAL,
                    left_targets_rad_exec,
                    velocity_scaling_factor=playback_speed_exec
                )
                ros_handler.send_moveit_joint_goal(
                    config.PLANNING_GROUP_RIGHT_ARM,
                    config.RIGHT_ARM_JOINT_NAMES_INTERNAL,
                    right_targets_rad_exec,
                    velocity_scaling_factor=playback_speed_exec
                )
                ros_handler.reset_latest_moveit_result()
            else:
                feedback_message_exec += " (MoveIt! Action Client 未就绪，无法回放手臂。)"

            if ros_handler.head_servo_pub:
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(tilt_s_val)}))
                time.sleep(0.02)
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(pan_s_val)}))

            time.sleep(0.05)
            if ros_handler.left_hand_pub:
                left_hand_targets_int_exec = [int(pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.LEFT_HAND_DOF_NAMES]
                ros_handler.left_hand_pub.publish(roslibpy.Message({'hand_angle': left_hand_targets_int_exec}))
            if ros_handler.right_hand_pub:
                right_hand_targets_int_exec = [int(pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.RIGHT_HAND_DOF_NAMES]
                ros_handler.right_hand_pub.publish(roslibpy.Message({'hand_angle': right_hand_targets_int_exec}))

        except Exception as e:
            print(f"Error during continuous playback step {current_idx + 1}: {e}")
            feedback_message_exec = f"连续回放错误在点 {current_idx + 1}: {e}"
        
        new_p_state_exec = current_p_state_exec.copy()
        new_p_state_exec['current_index'] = (current_idx + 1) % len(trajectory_to_play)
        return (new_p_state_exec, html.Div(feedback_message_exec, className="alert alert-info"), *slider_updates_exec)