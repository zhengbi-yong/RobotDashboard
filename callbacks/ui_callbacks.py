# your_robot_dashboard/callbacks/ui_callbacks.py

import dash
from dash import html, no_update, Input, Output, State, callback_context # Ensure callback_context is imported
import roslibpy # For roslibpy.Message
import json
import time
import math # For pi
import dash_bootstrap_components as dbc # If not already imported

from .. import config
from ..ros_comms import handler as ros_handler  # Renamed for clarity
from ..utils import trajectory_manager as traj_manager # Renamed for clarity

def register_callbacks(app):
    @app.callback(
        Output("ros-connection-status-display", "children"),
        Output("connect-ros-button", "disabled"),
        Input("connect-ros-button", "n_clicks"),
        Input("interval-ros-status-update", "n_intervals")
    )
    def handle_ros_connection(n_clicks_connect, n_intervals_status):
        ctx = dash.callback_context
        triggered_id = ctx.triggered[0]['prop_id'].split('.')[0] if ctx.triggered else "interval-ros-status-update"

        button_disabled_flag = False
        current_status_display = f"Status: {ros_handler.ros_connection_status}"

        if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
            button_disabled_flag = True
        elif ros_handler.ros_client and ros_handler.ros_client.is_connected:
            if ros_handler.ros_setup_done:
                current_status_display = f"Status: Connected to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT} (Setup Complete)"
            else:
                current_status_display = f"Status: Connected to {config.ROS_BRIDGE_HOST}:{config.ROS_BRIDGE_PORT} (Subscriber setup pending...)"
            button_disabled_flag = True
        # else: # Already covered by initial current_status_display

        if triggered_id == "connect-ros-button":
            if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
                print("Connection attempt already in progress.")
                return current_status_display, True

            if ros_handler.ros_client and ros_handler.ros_client.is_connected:
                print("ROS client already connected. Terminating to reconnect.")
                ros_handler.safe_terminate_ros_client() # Use the handler's function
                # Give threads a moment to close after client termination
                if ros_handler.subscriber_thread and ros_handler.subscriber_thread.is_alive():
                    ros_handler.subscriber_thread.join(timeout=0.5)
                if ros_handler.ros_connection_thread and ros_handler.ros_connection_thread.is_alive():
                    ros_handler.ros_connection_thread.join(timeout=0.5)
                time.sleep(1.0) # Ensure full cleanup before restarting

            ros_handler.try_connect_ros() # Use the handler's function
            current_status_display = f"Status: {ros_handler.ros_connection_status}"
            button_disabled_flag = True

        return current_status_display, button_disabled_flag

    @app.callback(
        Output("action-feedback-display", "children", allow_duplicate=True),
        [Input("send-left-arm-button", "n_clicks"),
         Input("send-right-arm-button", "n_clicks"),
         Input("send-head-servo-button", "n_clicks")],
        [State(f"l_arm_slider_{i}", "value") for i in range(7)] + \
        [State(f"r_arm_slider_{i}", "value") for i in range(7)] + \
        [State("head-tilt-slider", "value"), State("head-pan-slider", "value")],
        prevent_initial_call=True
    )
    def send_control_commands(n_left, n_right, n_head, *slider_values):
        ctx = dash.callback_context
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]

        if not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
            return html.Div("ROS not connected or setup not complete.", className="alert alert-danger")

        left_arm_angles_deg = slider_values[0:7]
        right_arm_angles_deg = slider_values[7:14]
        head_tilt_val = slider_values[14]
        head_pan_val = slider_values[15]

        try:
            if button_id == "send-left-arm-button":
                if ros_handler.left_arm_pub:
                    msg_data = {'joint': [a * (math.pi / 180.0) for a in left_arm_angles_deg], 'speed': 0.3, 'trajectory_connect': 0}
                    ros_handler.left_arm_pub.publish(roslibpy.Message(msg_data))
                    return html.Div("Left arm command sent!", className="alert alert-success")
                else: return html.Div("Left arm publisher not available.", className="alert alert-warning")

            elif button_id == "send-right-arm-button":
                if ros_handler.right_arm_pub:
                    msg_data = {'joint': [a * (math.pi / 180.0) for a in right_arm_angles_deg], 'speed': 0.3, 'trajectory_connect': 0}
                    ros_handler.right_arm_pub.publish(roslibpy.Message(msg_data))
                    return html.Div("Right arm command sent!", className="alert alert-success")
                else: return html.Div("Right arm publisher not available.", className="alert alert-warning")

            elif button_id == "send-head-servo-button":
                if ros_handler.head_servo_pub:
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_val)}))
                    time.sleep(0.05)
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_val)}))
                    return html.Div("Head servo commands sent!", className="alert alert-success")
                else: return html.Div("Head servo publisher not available.", className="alert alert-warning")
        except Exception as e:
            print(f"Error publishing command for {button_id}: {e}")
            return html.Div(f"Error sending command: {e}", className="alert alert-danger")
        return no_update


    @app.callback(
        [Output("arm-states-display", "children"), Output("head-states-display", "children")],
        [Input("interval-joint-state-update", "n_intervals"), Input("refresh-states-button", "n_clicks")]
    )
    def update_joint_state_display(n_intervals, n_refresh):
        # Access latest_joint_states from ros_handler
        arm_states_to_display = {name: ros_handler.latest_joint_states.get(name, "N/A") for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL + config.RIGHT_ARM_JOINT_NAMES_INTERNAL}
        head_states_to_display = {name: ros_handler.latest_joint_states.get(name, "N/A") for name in config.HEAD_SERVO_RANGES.keys()}
        return json.dumps(arm_states_to_display, indent=2), json.dumps(head_states_to_display, indent=2)

    @app.callback(
        [Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        [Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)] +
        [Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True),
         Output("action-feedback-display", "children", allow_duplicate=True)],
        Input("sync-sliders-to-state-button", "n_clicks"),
        prevent_initial_call=True
    )
    def sync_sliders_to_robot_state(n_clicks_sync):
        if not (ros_handler.ros_client and ros_handler.ros_client.is_connected):
            return [no_update] * 14 + [no_update, no_update] + [html.Div("ROS not connected.", className="alert alert-warning")]

        l_slider_values = [ros_handler.latest_joint_states.get(name, 0.0) * (180.0 / math.pi) for name in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
        r_slider_values = [ros_handler.latest_joint_states.get(name, 0.0) * (180.0 / math.pi) for name in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
        head_tilt_slider_value = ros_handler.latest_joint_states.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
        head_pan_slider_value = ros_handler.latest_joint_states.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
        
        all_slider_updates = l_slider_values + r_slider_values + [head_tilt_slider_value, head_pan_slider_value]
        feedback = html.Div("Sliders synchronized to current robot state.", className="alert alert-info")
        return *all_slider_updates, feedback

    @app.callback(
        Output("trajectory-select-dropdown", "options"),
        [Input("refresh-trajectory-list-button", "n_clicks"),
         Input("save-trajectory-button", "n_clicks")] # Trigger on save to refresh list
    )
    def update_trajectory_dropdown(n_refresh, n_save):
        return traj_manager.get_trajectory_files_options()


    @app.callback(
        [Output("action-feedback-display", "children", allow_duplicate=True),
        Output("recorded-positions-count-display", "children", allow_duplicate=True),
        Output("recorded-positions-list-display", "children", allow_duplicate=True),
        Output("playback-state-store", "data", allow_duplicate=True)],
        [Input({'type': 'delete-trajectory-point-button', 'index': dash.ALL}, "n_clicks")], # 输入是删除按钮
        [State("playback-state-store", "data")],
        prevent_initial_call=True
    )
    def handle_delete_trajectory_point(delete_button_n_clicks, current_playback_state):
        ctx = callback_context # 或者 dash.callback_context

        # 关键的判断：确保这个回调确实是由一个删除按钮的点击触发的
        # 并且 n_clicks 列表不全是 None 或 0 (表示没有实际点击发生)
        if not ctx.triggered_id or not any(delete_button_n_clicks):
            # 如果没有任何删除按钮被点击 (例如，页面加载时或另一个回调触发了输出更新)
            # 那么这个回调不应该执行任何删除操作或返回删除相关的反馈
            return no_update, no_update, no_update, no_update

        # 从 ctx.triggered_id 获取触发此回调的按钮的实际 ID (包含 index)
        # ctx.triggered_id 是一个字典 {'type': 'delete-trajectory-point-button', 'index': X}
        # 如果不是由模式匹配的ID触发，triggered_id 可能只是一个字符串
        if isinstance(ctx.triggered_id, dict) and ctx.triggered_id.get("type") == 'delete-trajectory-point-button':
            point_index_to_delete = ctx.triggered_id['index']
        else:
            # 如果 triggered_id 不是预期的模式，则不应执行删除
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

    # Callback for record, save, load (ensure allow_duplicate=True is correctly set on its outputs if they are shared)
    # It might be cleaner to have a single callback update the trajectory display and store,
    # and other callbacks (record, load, delete) just update the traj_manager and trigger this central display callback.
    # For now, we duplicate the output updates.
    @app.callback(
        [Output("action-feedback-display", "children", allow_duplicate=True),
         Output("recorded-positions-count-display", "children", allow_duplicate=True), # Added allow_duplicate
         Output("recorded-positions-list-display", "children", allow_duplicate=True), # Added allow_duplicate
         Output("playback-state-store", "data", allow_duplicate=True)],
        [Input("record-position-button", "n_clicks"),
         Input("save-trajectory-button", "n_clicks"),
         Input("load-selected-trajectory-button", "n_clicks")],
        [State("trajectory-filename-input", "value"),
         State("trajectory-select-dropdown", "value"),
         State("playback-state-store", "data")], # Re-use existing state
        prevent_initial_call=True
    )
    def handle_record_save_load_trajectory(n_record, n_save, n_load,
                                           trajectory_filename, selected_trajectory_path,
                                           current_playback_state): # current_playback_state is already a param
        ctx = callback_context # Use dash.callback_context
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
        feedback_msg = ""
        feedback_type = "info"
        new_playback_store_data = current_playback_state.copy()
        
        active_trajectory_after_action = []

        if button_id == "record-position-button":
            feedback_msg = traj_manager.record_current_position(ros_handler.latest_joint_states)
            active_trajectory_after_action = traj_manager.get_current_trajectory()
            feedback_type = "success"
        elif button_id == "save-trajectory-button":
            feedback_msg, feedback_type = traj_manager.save_trajectory(trajectory_filename)
            active_trajectory_after_action = traj_manager.get_current_trajectory() # get current list
        elif button_id == "load-selected-trajectory-button":
            feedback_msg, feedback_type, loaded_traj = traj_manager.load_trajectory(selected_trajectory_path)
            active_trajectory_after_action = loaded_traj # use the returned list

        new_playback_store_data['trajectory_for_repeat'] = active_trajectory_after_action
        new_playback_store_data['current_index'] = 0

        count_display_text = f"活跃轨迹中包含 {len(active_trajectory_after_action)} 个位置点。" if active_trajectory_after_action else "活跃轨迹为空，请记录或加载位置点。"
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
        try:
            for i, pos_data in enumerate(current_trajectory):
                print(f"  回放点 {i+1}/{len(current_trajectory)}")
                if ros_handler.left_arm_pub:
                    left_targets = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                    ros_handler.left_arm_pub.publish(roslibpy.Message({'joint': left_targets, 'speed': playback_speed, 'trajectory_connect': 0}))
                if ros_handler.right_arm_pub:
                    right_targets = [pos_data.get(j, ros_handler.latest_joint_states.get(j, 0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                    ros_handler.right_arm_pub.publish(roslibpy.Message({'joint': right_targets, 'speed': playback_speed, 'trajectory_connect': 0}))
                time.sleep(0.05)
                if ros_handler.head_servo_pub:
                    head_tilt_target = pos_data.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(head_tilt_target)}))
                    time.sleep(0.02)
                    head_pan_target = pos_data.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
                    ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(head_pan_target)}))
                time.sleep(max(0.1, point_delay_sec))
            print("单次回放完成。")
            return html.Div("单次回放完成。", className="alert alert-success")
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
        ctx = dash.callback_context; button_id = ctx.triggered[0]['prop_id'].split('.')[0]
        new_p_state_manage = current_p_state_manage.copy()
        feedback_text, feedback_type = "", "info"
        interval_disabled = True; interval_ms = max(100, int(delay_sec_state * 1000))
        start_button_disabled = False; stop_button_disabled = True

        if button_id == "start-continuous-replay-button":
            if not new_p_state_manage.get('trajectory_for_repeat'): # Check the store
                feedback_text, feedback_type = "无活跃轨迹可供连续回放。", "warning"
            elif not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
                feedback_text, feedback_type = "ROS未就绪，无法开始连续回放。", "danger"
            else:
                new_p_state_manage['is_repeating'] = True; new_p_state_manage['current_index'] = 0
                interval_disabled = False;
                start_button_disabled = True; stop_button_disabled = False
                feedback_text, feedback_type = "连续回放已开始。", "success"
        elif button_id == "stop-continuous-replay-button":
            new_p_state_manage['is_repeating'] = False
            feedback_text, feedback_type = "连续回放已停止。", "info"
        
        # Ensure interval_ms is updated even if other conditions fail for start
        interval_ms = max(100, int(delay_sec_state * 1000))

        return interval_disabled, interval_ms, start_button_disabled, stop_button_disabled, new_p_state_manage, html.Div(feedback_text,className=f"alert alert-{feedback_type}")


    @app.callback(
        [Output("playback-state-store", "data", allow_duplicate=True),
         Output("action-feedback-display", "children", allow_duplicate=True),
         *[Output(f"l_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         *[Output(f"r_arm_slider_{i}", "value", allow_duplicate=True) for i in range(7)],
         Output("head-tilt-slider", "value", allow_duplicate=True),
         Output("head-pan-slider", "value", allow_duplicate=True)],
        Input("continuous-playback-interval", "n_intervals"),
        [State("playback-state-store", "data"), State("playback-speed-slider", "value")],
        prevent_initial_call=True
    )
    def execute_continuous_playback_step(n_intervals_playback, current_p_state_exec, playback_speed_exec):
        slider_no_updates = [no_update] * 16 # 7+7+1+1
        if not current_p_state_exec.get('is_repeating') or not (ros_handler.ros_client and ros_handler.ros_client.is_connected and ros_handler.ros_setup_done):
            return current_p_state_exec, no_update, *slider_no_updates

        trajectory_to_play = current_p_state_exec.get('trajectory_for_repeat', [])
        if not trajectory_to_play:
            return current_p_state_exec, html.Div("连续回放错误：轨迹为空。", className="alert alert-warning"), *slider_no_updates

        current_idx = current_p_state_exec.get('current_index', 0)
        if current_idx >= len(trajectory_to_play): current_idx = 0

        pos_data_exec = trajectory_to_play[current_idx]
        l_slider_vals_exec = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
        r_slider_vals_exec = [pos_data_exec.get(j, 0.0) * (180.0 / math.pi) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
        tilt_val_exec = pos_data_exec.get('head_tilt_servo', config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'])
        pan_val_exec = pos_data_exec.get('head_pan_servo', config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'])
        slider_updates_exec = l_slider_vals_exec + r_slider_vals_exec + [tilt_val_exec, pan_val_exec]

        feedback_message_exec = f"连续回放: 点 {current_idx + 1}/{len(trajectory_to_play)}"
        print(feedback_message_exec)
        try:
            if ros_handler.left_arm_pub:
                left_targets_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL]
                ros_handler.left_arm_pub.publish(roslibpy.Message({'joint': left_targets_exec, 'speed': playback_speed_exec, 'trajectory_connect': 0}))
            if ros_handler.right_arm_pub:
                right_targets_exec = [pos_data_exec.get(j, ros_handler.latest_joint_states.get(j,0.0)) for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL]
                ros_handler.right_arm_pub.publish(roslibpy.Message({'joint': right_targets_exec, 'speed': playback_speed_exec, 'trajectory_connect': 0}))
            time.sleep(0.05) # small delay between arm and head
            if ros_handler.head_servo_pub:
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_tilt_servo']['id'], 'angle': int(tilt_val_exec)}))
                time.sleep(0.02) # small delay between servo commands
                ros_handler.head_servo_pub.publish(roslibpy.Message({'servo_id': config.HEAD_SERVO_RANGES['head_pan_servo']['id'], 'angle': int(pan_val_exec)}))
        except Exception as e:
            print(f"Error during continuous playback step {current_idx + 1}: {e}")
            feedback_message_exec = f"连续回放错误在点 {current_idx + 1}: {e}"
        
        new_p_state_exec = current_p_state_exec.copy()
        new_p_state_exec['current_index'] = (current_idx + 1) % len(trajectory_to_play)
        return new_p_state_exec, html.Div(feedback_message_exec, className="alert alert-info"), *slider_updates_exec