# RobotDashboard/callbacks/ui_callbacks.py

import dash
from dash import html, no_update, Input, Output, State, callback_context
import dash_bootstrap_components as dbc
import roslibpy
import json
import time
import math

from .. import config
from ..ros_comms import handler as ros_handler
from ..utils import trajectory_manager as traj_manager

def register_callbacks(app):
    # ... (handle_ros_connection callback remains the same) ...
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
        
        if triggered_id == "connect-ros-button" and n_clicks_connect is not None: # Check n_clicks
            if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
                print("连接尝试已在进行中。")
                return current_status_display, True

            if ros_handler.ros_client and ros_handler.ros_client.is_connected:
                print("ROS 客户端已连接。正在终止现有连接以重新连接。")
                ros_handler.safe_terminate_ros_client()
                if ros_handler.subscriber_thread and ros_handler.subscriber_thread.is_alive():
                    ros_handler.subscriber_thread.join(timeout=0.5)
                if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive(): # Should be false now
                    ros_handler.ros_connection_thread.join(timeout=0.5)
                time.sleep(1.0)

            ros_handler.try_connect_ros()
            current_status_display = f"状态: {ros_handler.ros_connection_status}" # Will show "Connecting..."
            button_disabled_flag = True
            
        return current_status_display, button_disabled_flag


    @app.callback(
        Output("action-feedback-display", "children", allow_duplicate=True),
        [Input("send-left-arm-button", "n_clicks"),
         Input("send-right-arm-button", "n_clicks"),
         Input("send-head-servo-button", "n_clicks"),
         Input("send-left-hand-button", "n_clicks"),   # NEW
         Input("send-right-hand-button", "n_clicks")], # NEW
        # Arm states (7 each)
        [State(f"l_arm_slider_{i}", "value") for i in range(7)] +
        [State(f"r_arm_slider_{i}", "value") for i in range(7)] +
        # Head states (2)
        [State("head-tilt-slider", "value"), State("head-pan-slider", "value")] +
        # Hand states (6 each) - NEW
        [State(f"l_hand_slider_{i}", "value") for i in range(len(config.LEFT_HAND_DOF_NAMES))] +
        [State(f"r_hand_slider_{i}", "value") for i in range(len(config.RIGHT_HAND_DOF_NAMES))],
        prevent_initial_call=True
    )
    def send_control_commands(n_l_arm, n_r_arm, n_head, n_l_hand, n_r_hand, *slider_values): # Added n_l_hand, n_r_hand
        ctx = callback_context
        button_id = ctx.triggered_id # Simpler way to get button id

        if not button_id: # If not triggered by a button click (e.g. initial load with prevent_initial_call)
            return no_update

        if not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
            return html.Div("ROS 未连接或设置未完成。无法发送指令。", className="alert alert-danger")

        # Unpack slider_values based on their order in State
        left_arm_angles_deg = slider_values[0:7]
        right_arm_angles_deg = slider_values[7:14]
        head_tilt_val = slider_values[14]
        head_pan_val = slider_values[15]
        # NEW: Unpack hand slider values
        left_hand_angles_raw = slider_values[16 : 16 + len(config.LEFT_HAND_DOF_NAMES)]
        right_hand_angles_raw = slider_values[16 + len(config.LEFT_HAND_DOF_NAMES) : 16 + len(config.LEFT_HAND_DOF_NAMES) + len(config.RIGHT_HAND_DOF_NAMES)]

        # Convert hand angles to int as per message spec
        left_hand_angles_int = [int(a) for a in left_hand_angles_raw]
        right_hand_angles_int = [int(a) for a in right_hand_angles_raw]

        feedback_msg = "未知错误"
        try:
            if button_id == "send-left-arm-button":
                if ros_handler.left_arm_pub:
                    msg_data = {'joint': [a * (math.pi / 180.0) for a in left_arm_angles_deg], 'speed': 0.3, 'trajectory_connect': 0}
                    ros_handler.left_arm_pub.publish(roslibpy.Message(msg_data))
                    feedback_msg = "左臂指令已发送!"
                else: return html.Div("左臂 Publisher 不可用。", className="alert alert-warning")
            # ... (right arm and head logic remains similar) ...
            elif button_id == "send-right-arm-button":
                if ros_handler.right_arm_pub:
                    msg_data = {'joint': [a * (math.pi / 180.0) for a in right_arm_angles_deg], 'speed': 0.3, 'trajectory_connect': 0}
                    ros_handler.right_arm_pub.publish(roslibpy.Message(msg_data))
                    feedback_msg = "右臂指令已发送!"
                else: return html.Div("右臂 Publisher 不可用。", className="alert alert-warning")

            elif button_id == "send-head-servo-button":
                if ros_handler.head_servo_pub:
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_val)}))
                    time.sleep(0.05)
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_val)}))
                    # Update latest_joint_states for head after sending command
                    ros_handler.latest_joint_states['head_tilt_servo'] = float(head_tilt_val)
                    ros_handler.latest_joint_states['head_pan_servo'] = float(head_pan_val)
                    feedback_msg = "头部伺服指令已发送!"
                else: return html.Div("头部伺服 Publisher 不可用。", className="alert alert-warning")

            # NEW: Handle hand commands
            elif button_id == "send-left-hand-button":
                if ros_handler.left_hand_pub:
                    msg_data = {'hand_angle': left_hand_angles_int}
                    ros_handler.left_hand_pub.publish(roslibpy.Message(msg_data))
                    # Update latest_joint_states for left hand after sending command
                    for i, dof_name in enumerate(config.LEFT_HAND_DOF_NAMES):
                        ros_handler.latest_joint_states[dof_name] = float(left_hand_angles_int[i])
                    feedback_msg = "左手指令已发送!"
                else: return html.Div("左手 Publisher 不可用。", className="alert alert-warning")

            elif button_id == "send-right-hand-button":
                if ros_handler.right_hand_pub:
                    msg_data = {'hand_angle': right_hand_angles_int}
                    ros_handler.right_hand_pub.publish(roslibpy.Message(msg_data))
                    # Update latest_joint_states for right hand after sending command
                    for i, dof_name in enumerate(config.RIGHT_HAND_DOF_NAMES):
                        ros_handler.latest_joint_states[dof_name] = float(right_hand_angles_int[i])
                    feedback_msg = "右手指令已发送!"
                else: return html.Div("右手 Publisher 不可用。", className="alert alert-warning")
            
            return html.Div(feedback_msg, className="alert alert-success")

        except Exception as e:
            print(f"发送指令时出错 ({button_id}): {e}")
            return html.Div(f"发送指令错误: {e}", className="alert alert-danger")

    # --- update_joint_state_display callback ---
    # This callback might need to be expanded if you add hand state topics and displays
    @app.callback(
        [Output("arm-states-display", "children"), Output("head-states-display", "children")],
        [Input("interval-joint-state-update", "n_intervals"), Input("refresh-states-button", "n_clicks")]
    )
    def update_joint_state_display(n_intervals, n_refresh):
        arm_states_to_display = {name: ros_handler.latest_joint_states.get(name, "N/A") for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL + config.RIGHT_ARM_JOINT_NAMES_INTERNAL}
        head_states_to_display = {name: ros_handler.latest_joint_states.get(name, "N/A") for name in config.HEAD_SERVO_RANGES.keys()}
        # If you add hand state display elements, populate their data here
        # hand_states_to_display = {name: ros_handler.latest_joint_states.get(name, "N/A") for name in config.LEFT_HAND_DOF_NAMES + config.RIGHT_HAND_DOF_NAMES}
        return json.dumps(arm_states_to_display, indent=2), json.dumps(head_states_to_display, indent=2)


    # --- sync_sliders_to_robot_state callback ---
    # This needs to be updated to include hand sliders if hand state feedback becomes available
    # For now, it only syncs arms and head based on actual feedback.
    @app.callback(
        # Arm slider outputs (7 each)
        [Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        [Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        # Head slider outputs (2)
        [Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True)] +
        # Hand slider outputs (6 each) - NEW (will use default if no feedback)
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
        
        # For hands, use the values from latest_joint_states which are updated on command
        # or default if never commanded / no feedback mechanism implemented yet
        l_hand_vals = [ros_handler.latest_joint_states.get(name, config.DEFAULT_HAND_ANGLE) for name in config.LEFT_HAND_DOF_NAMES]
        r_hand_vals = [ros_handler.latest_joint_states.get(name, config.DEFAULT_HAND_ANGLE) for name in config.RIGHT_HAND_DOF_NAMES]

        all_slider_updates = l_arm_vals + r_arm_vals + [head_tilt_val, head_pan_val] + l_hand_vals + r_hand_vals
        feedback = html.Div("滑块已同步至当前机器人状态/指令值。", className="alert alert-info")
        return *all_slider_updates, feedback

    # --- Trajectory related callbacks need to be updated for hands ---
    # handle_record_save_load_trajectory:
    #   - The record_current_position in trajectory_manager will now pick up hand values from latest_joint_states
    #     (because send_control_commands updates them there).
    #   - The display function get_display_for_recorded_positions_list will need to show hand values.

    # handle_replay_once & execute_continuous_playback_step:
    #   - Need to extract hand values from pos_data and publish hand commands.

    # --- update_trajectory_dropdown --- (no change needed for this one)
    @app.callback(
        Output("trajectory-select-dropdown", "options"),
        [Input("refresh-trajectory-list-button", "n_clicks"),
         Input("save-trajectory-button", "n_clicks")]
    )
    def update_trajectory_dropdown(n_refresh, n_save):
        return traj_manager.get_trajectory_files_options()

    # --- handle_delete_trajectory_point --- (no direct change for hands, but display will reflect it)
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
        list_display_children = traj_manager.get_display_for_recorded_positions_list(active_trajectory) # This will now show hands too
        
        return html.Div(feedback_msg, className=f"alert alert-{feedback_type} mt-2"), count_display_text, list_display_children, new_playback_store_data

    # --- handle_record_save_load_trajectory ---
    # No direct code change needed here IF record_current_position in traj_manager
    # correctly picks up hand values from latest_joint_states (which it should now).
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
         State("playback-state-store", "data")],
        prevent_initial_call=True
    )
    def handle_record_save_load_trajectory(n_record, n_save, n_load,
                                           trajectory_filename, selected_trajectory_path,
                                           current_playback_state):
        ctx = callback_context
        button_id = ctx.triggered_id
        if not button_id:
             return no_update, no_update, no_update, no_update

        feedback_msg = ""
        feedback_type = "info"
        new_playback_store_data = current_playback_state.copy()
        active_trajectory_after_action = []

        if button_id == "record-position-button":
            if not n_record: return no_update, no_update, no_update, no_update
            # latest_joint_states in ros_handler should now contain commanded hand values
            feedback_msg = traj_manager.record_current_position(ros_handler.latest_joint_states)
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
        list_display_children = traj_manager.get_display_for_recorded_positions_list(active_trajectory_after_action) # Will show hands
        
        return html.Div(feedback_msg, className=f"alert alert-{feedback_type} mt-2"), count_display_text, list_display_children, new_playback_store_data

    # --- Replay Callbacks ---
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
        try:
            for i, pos_data in enumerate(current_trajectory):
                # print(f"  回放点 {i+1}/{len(current_trajectory)}")
                # Arm commands
                if ros_handler.left_arm_pub:
                    left_targets = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                    ros_handler.left_arm_pub.publish(roslibpy.Message({'joint': left_targets, 'speed': playback_speed, 'trajectory_connect': 0}))
                if ros_handler.right_arm_pub:
                    right_targets = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                    ros_handler.right_arm_pub.publish(roslibpy.Message({'joint': right_targets, 'speed': playback_speed, 'trajectory_connect': 0}))
                time.sleep(0.05)
                # Head commands
                if ros_handler.head_servo_pub:
                    # ... (head command logic) ...
                    head_tilt_target = pos_data.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_target)}))
                    time.sleep(0.02)
                    head_pan_target = pos_data.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_target)}))
                
                # NEW: Hand commands
                time.sleep(0.05) # Small delay before hand commands
                if ros_handler.left_hand_pub:
                    left_hand_targets_int = [int(pos_data.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.LEFT_HAND_DOF_NAMES]
                    ros_handler.left_hand_pub.publish(roslibpy.Message({'hand_angle': left_hand_targets_int}))
                if ros_handler.right_hand_pub:
                    right_hand_targets_int = [int(pos_data.get(dof, config.DEFAULT_HAND_ANGLE)) for dof in config.RIGHT_HAND_DOF_NAMES]
                    ros_handler.right_hand_pub.publish(roslibpy.Message({'hand_angle': right_hand_targets_int}))

                time.sleep(max(0.1, point_delay_sec))
            print("单次回放完成。")
            return html.Div("单次回放完成。", className="alert alert-success")
        except Exception as e:
            print(f"回放过程中发生错误: {e}")
            return html.Div(f"回放错误: {e}", className="alert alert-danger")

    # manage_continuous_replay_controls (no direct changes for hands, but uses trajectory from store)
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
            else:
                new_p_state_manage['is_repeating'] = True; new_p_state_manage['current_index'] = 0
                interval_disabled = False;
                start_button_disabled = True; stop_button_disabled = False
                feedback_text, feedback_type = "连续回放已开始。", "success"
        elif button_id == "stop-continuous-replay-button":
            if not n_stop: return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, no_update
            new_p_state_manage['is_repeating'] = False
            feedback_text, feedback_type = "连续回放已停止。", "info"
        
        interval_ms = max(100, int(delay_sec_state * 1000)) # ensure interval is updated
        return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, html.Div(feedback_text,className=f"alert alert-{feedback_type}")


    # execute_continuous_playback_step
    @app.callback(
        [Output("playback-state-store", "data", allow_duplicate=True),
         Output("action-feedback-display", "children", allow_duplicate=True),
         # Arm slider outputs
         *[Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         *[Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         # Head slider outputs
         Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True),
         # Hand slider outputs - NEW
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

        if not current_p_state_exec.get('is_repeating') or not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
            return current_p_state_exec, no_update, *slider_no_updates

        trajectory_to_play = current_p_state_exec.get('trajectory_for_repeat', [])
        if not trajectory_to_play:
            return current_p_state_exec, html.Div("连续回放错误：轨迹为空。", className="alert alert-warning"), *slider_no_updates

        current_idx = current_p_state_exec.get('current_index', 0)
        if current_idx >= len(trajectory_to_play): current_idx = 0

        pos_data_exec = trajectory_to_play[current_idx]
        
        # Prepare slider updates (arms, head, hands)
        l_arm_s_vals = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
        r_arm_s_vals = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
        tilt_s_val = pos_data_exec.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
        pan_s_val = pos_data_exec.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
        l_hand_s_vals = [pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE) for dof in config.LEFT_HAND_DOF_NAMES]
        r_hand_s_vals = [pos_data_exec.get(dof, config.DEFAULT_HAND_ANGLE) for dof in config.RIGHT_HAND_DOF_NAMES]
        slider_updates_exec = l_arm_s_vals + r_arm_s_vals + [tilt_s_val, pan_s_val] + l_hand_s_vals + r_hand_s_vals

        feedback_message_exec = f"连续回放: 点 {current_idx + 1}/{len(trajectory_to_play)}"
        print(feedback_message_exec)
        try:
            # Arm commands
            if ros_handler.left_arm_pub:
                left_targets_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                ros_handler.left_arm_pub.publish(roslibpy.Message({'joint': left_targets_exec, 'speed': playback_speed_exec, 'trajectory_connect': 0}))
            if ros_handler.right_arm_pub:
                right_targets_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                ros_handler.right_arm_pub.publish(roslibpy.Message({'joint': right_targets_exec, 'speed': playback_speed_exec, 'trajectory_connect': 0}))
            time.sleep(0.05)
            # Head commands
            if ros_handler.head_servo_pub:
                # ... (head command logic as in replay_once) ...
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(tilt_s_val)}))
                time.sleep(0.02)
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(pan_s_val)}))

            # NEW: Hand commands for continuous playback
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