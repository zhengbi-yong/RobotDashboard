# RobotDashboard/utils/trajectory_manager.py

import json
import os
import glob
from dash import html
import dash_bootstrap_components as dbc # <--- 确保导入 dbc
from .. import config # Relative import

# This list will store the active trajectory
recorded_positions = []

# ... (record_current_position, save_trajectory, load_trajectory, get_trajectory_files_options 函数保持不变) ...

def record_current_position(latest_joint_states_dict):
    global recorded_positions
    current_pos_to_record = {}
    all_joint_keys = (config.LEFT_ARM_JOINT_NAMES_INTERNAL +
                      config.RIGHT_ARM_JOINT_NAMES_INTERNAL +
                      list(config.HEAD_SERVO_RANGES.keys()))
    for key in all_joint_keys:
        current_pos_to_record[key] = latest_joint_states_dict.get(key, 0.0)
    recorded_positions.append(current_pos_to_record.copy())
    return f"已记录位置 {len(recorded_positions)} 到当前活跃轨迹。"

def save_trajectory(filename_str):
    global recorded_positions
    if not filename_str:
        return "请输入轨迹文件名。", "warning"
    if not recorded_positions:
        return "活跃轨迹中无位置点可保存。", "warning"

    sane_filename = "".join(c if c.isalnum() or c in ('_', '-') else '_' for c in filename_str.strip())
    if not sane_filename: sane_filename = "unnamed_trajectory"
    save_path = os.path.join(config.TRAJECTORY_DIR, f"{sane_filename}.json")
    try:
        with open(save_path, "w") as f: json.dump(recorded_positions, f, indent=4)
        return f"已保存 {len(recorded_positions)} 个位置点到文件: {os.path.basename(save_path)}.", "success"
    except Exception as e:
        return f"保存文件失败: {e}", "danger"

def load_trajectory(selected_trajectory_path):
    global recorded_positions
    if not selected_trajectory_path:
        return "未选择任何轨迹文件进行加载。", "warning", []
    if not os.path.exists(selected_trajectory_path):
        return f"错误: 选择的文件不存在 {os.path.basename(selected_trajectory_path)}", "danger", []
    try:
        with open(selected_trajectory_path, "r") as f: loaded_data = json.load(f)
        if not isinstance(loaded_data, list): raise ValueError("Trajectory file content is not a list.")
        recorded_positions = loaded_data
        return f"已从 {os.path.basename(selected_trajectory_path)} 加载 {len(recorded_positions)} 个位置点到活跃轨迹。", "success", recorded_positions
    except Exception as e:
        recorded_positions = []
        return f"加载或解析轨迹文件失败: {e}", "danger", []

def get_trajectory_files_options():
    files = glob.glob(os.path.join(config.TRAJECTORY_DIR, "*.json"))
    options = [{"label": os.path.basename(f).replace(".json",""), "value": f} for f in sorted(files)]
    if not options:
        options = [{"label": "No trajectories found", "value": "", "disabled": True}]
    return options

# NEW FUNCTION to delete a point (您应该已经添加了这个函数，如果没有，也需要添加)
def delete_position_from_trajectory(index_to_delete):
    global recorded_positions
    try:
        index_to_delete = int(index_to_delete)
        if 0 <= index_to_delete < len(recorded_positions):
            deleted_point_summary = f"点 {index_to_delete + 1}"
            recorded_positions.pop(index_to_delete)
            return f"已从活跃轨迹中删除 {deleted_point_summary}。", "success"
        else:
            return f"删除失败：索引 {index_to_delete} 无效。", "warning"
    except ValueError:
        return "删除失败：索引必须是数字。", "danger"
    except Exception as e:
        return f"删除点时发生错误: {e}", "danger"


# --- 这是需要替换的函数 ---
def get_display_for_recorded_positions_list(positions_list):
    if not positions_list:
        return "尚未记录或加载任何位置到活跃轨迹。"
    
    children = []
    for i, pos_data in enumerate(positions_list):
        l_summary_parts = [f"{pos_data.get(j, 0.0):.2f}" for j in config.LEFT_ARM_JOINT_NAMES_INTERNAL[:3]]
        r_summary_parts = [f"{pos_data.get(j, 0.0):.2f}" for j in config.RIGHT_ARM_JOINT_NAMES_INTERNAL[:3]]
        h_summary = f"Tilt:{pos_data.get('head_tilt_servo', 0):.0f}, Pan:{pos_data.get('head_pan_servo', 0):.0f}"
        summary_str = f"点 {i+1}: L({', '.join(l_summary_parts)}...), R({', '.join(r_summary_parts)}...), H({h_summary})"
        
        # 为每个点创建删除按钮
        delete_button = dbc.Button(
            "删除此点",
            id={'type': 'delete-trajectory-point-button', 'index': i}, # pattern-matching ID
            color="danger",
            size="sm",
            className="ml-2" # 按钮左边距
        )
        
        # 将点摘要和删除按钮放在一行中
        point_display_item = dbc.Row([
            dbc.Col(html.Summary(summary_str), width="auto"),
            dbc.Col(delete_button, width="auto")
        ], align="center", className="mb-1") # mb-1 是底边距

        children.append(html.Details([
            point_display_item, # 使用包含删除按钮的行
            html.Pre(json.dumps(pos_data, indent=2), style={'fontSize': '0.8em', 'marginLeft': '20px'}) # 详细信息缩进
        ], open=False, className="mb-1"))
    # print("Generated display children for trajectory list:", children) # 调试时可以取消注释
    return children
# --- 函数替换结束 ---

def get_current_trajectory():
    global recorded_positions
    return recorded_positions.copy()

def set_current_trajectory(new_trajectory_list):
    global recorded_positions
    recorded_positions = new_trajectory_list