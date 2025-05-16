# RobotDashboard/components/layout.py

from dash import dcc, html
import dash_bootstrap_components as dbc
from .. import config

def create_layout():
    # --- Left Arm Control Card ---
    left_arm_card_content = [
        dbc.CardHeader("左臂控制 (Left Arm Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"关节 {i+1} ({config.LEFT_ARM_JOINT_NAMES_INTERNAL[i]})"),
                dcc.Slider(id=f"l_arm_slider_{i}", min=config.ARM_SLIDER_MIN, max=config.ARM_SLIDER_MAX, value=config.ARM_SLIDER_DEFAULT, step=config.ARM_SLIDER_STEP, marks={j: str(j) for j in range(int(config.ARM_SLIDER_MIN), int(config.ARM_SLIDER_MAX)+1, config.ARM_SLIDER_MARKS_STEP)}, tooltip={"placement": "top", "always_visible": False})
            ], className="mb-3") for i in range(7)],
            html.Div(dbc.Button("发送左臂指令", id="send-left-arm-button", color="primary", className="mt-2 w-100"), className="d-grid")
        ])
    ]

    # --- Right Arm Control Card ---
    right_arm_card_content = [
        dbc.CardHeader("右臂控制 (Right Arm Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"关节 {i+1} ({config.RIGHT_ARM_JOINT_NAMES_INTERNAL[i]})"),
                dcc.Slider(id=f"r_arm_slider_{i}", min=config.ARM_SLIDER_MIN, max=config.ARM_SLIDER_MAX, value=config.ARM_SLIDER_DEFAULT, step=config.ARM_SLIDER_STEP, marks={j: str(j) for j in range(int(config.ARM_SLIDER_MIN), int(config.ARM_SLIDER_MAX)+1, config.ARM_SLIDER_MARKS_STEP)}, tooltip={"placement": "top", "always_visible": False})
            ], className="mb-3") for i in range(7)],
            html.Div(dbc.Button("发送右臂指令", id="send-right-arm-button", color="primary", className="mt-2 w-100"), className="d-grid")
        ])
    ]

    # --- Head Servo Control Card ---
    head_servo_card_content = [
        dbc.CardHeader("头部伺服控制 (Head Servo Control)"),
        dbc.CardBody([
            html.Div([
                html.Label(f"头部俯仰 (ID {config.HEAD_SERVO_RANGES['head_tilt_servo']['id']})"),
                dcc.Slider(id="head-tilt-slider", min=config.HEAD_SERVO_RANGES['head_tilt_servo']['min'], max=config.HEAD_SERVO_RANGES['head_tilt_servo']['max'], value=config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'], step=10, marks={j: str(j) for j in range(config.HEAD_SERVO_RANGES['head_tilt_servo']['min'], config.HEAD_SERVO_RANGES['head_tilt_servo']['max'] + 1, 100)}, tooltip={"placement": "top", "always_visible": False})
            ], className="mb-3"),
            html.Div([
                html.Label(f"头部左右 (ID {config.HEAD_SERVO_RANGES['head_pan_servo']['id']})"),
                dcc.Slider(id="head-pan-slider", min=config.HEAD_SERVO_RANGES['head_pan_servo']['min'], max=config.HEAD_SERVO_RANGES['head_pan_servo']['max'], value=config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'], step=10, marks={j: str(j) for j in range(config.HEAD_SERVO_RANGES['head_pan_servo']['min'], config.HEAD_SERVO_RANGES['head_pan_servo']['max'] + 1, 200)}, tooltip={"placement": "top", "always_visible": False})
            ], className="mb-3"),
            html.Div(dbc.Button("发送头部指令", id="send-head-servo-button", color="primary", className="mt-2 w-100"), className="d-grid")
        ])
    ]

    # --- Left Hand Control Card --- NEW
    left_hand_card_content = [
        dbc.CardHeader("左手控制 (Left Hand Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"{config.LEFT_HAND_DOF_NAMES[i]}"), # Using DOF names from config
                dcc.Slider(
                    id=f"l_hand_slider_{i}",
                    min=config.MIN_HAND_ANGLE, max=config.MAX_HAND_ANGLE, value=config.DEFAULT_HAND_ANGLE,
                    step=config.HAND_ANGLE_STEP,
                    marks={j: str(j) for j in range(config.MIN_HAND_ANGLE, config.MAX_HAND_ANGLE + 1, config.HAND_MARKS_STEP)},
                    tooltip={"placement": "top", "always_visible": False}
                )
            ], className="mb-3") for i in range(len(config.LEFT_HAND_DOF_NAMES))],
            html.Div(dbc.Button("发送左手指令", id="send-left-hand-button", color="info", className="mt-2 w-100"), className="d-grid")
        ])
    ]

    # --- Right Hand Control Card --- NEW
    right_hand_card_content = [
        dbc.CardHeader("右手控制 (Right Hand Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"{config.RIGHT_HAND_DOF_NAMES[i]}"), # Using DOF names from config
                dcc.Slider(
                    id=f"r_hand_slider_{i}",
                    min=config.MIN_HAND_ANGLE, max=config.MAX_HAND_ANGLE, value=config.DEFAULT_HAND_ANGLE,
                    step=config.HAND_ANGLE_STEP,
                    marks={j: str(j) for j in range(config.MIN_HAND_ANGLE, config.MAX_HAND_ANGLE + 1, config.HAND_MARKS_STEP)},
                    tooltip={"placement": "top", "always_visible": False}
                )
            ], className="mb-3") for i in range(len(config.RIGHT_HAND_DOF_NAMES))],
            html.Div(dbc.Button("发送右手指令", id="send-right-hand-button", color="info", className="mt-2 w-100"), className="d-grid")
        ])
    ]


    # --- ROS Connection and Status ---
    ros_connection_section = dbc.Card([
        dbc.CardHeader("ROS 连接状态 (ROS Connection)"),
        dbc.CardBody([
            dbc.Row([
                dbc.Col(dbc.Button("连接/重连 ROS", id="connect-ros-button", color="success", className="w-100"), width=12, md=4, className="mb-2 mb-md-0 d-grid"),
                dbc.Col(html.Div(id="ros-connection-status-display", children=f"状态: 未连接", className="align-self-center text-md-left text-center p-2 border rounded bg-light"), width=12, md=8)
            ], align="center")
        ])
    ], className="mb-4")

    # --- Current Joint States Card ---
    joint_states_card_content = [
        dbc.CardHeader("当前关节与传感器状态 (Current States)"), # Updated title
        dbc.CardBody([
            dbc.Row([
                dbc.Col(dbc.Button("手动刷新", id="refresh-states-button", color="info", outline=True, size="sm", className="me-2"), width="auto"),
                dbc.Col(dbc.Button("同步滑块至状态", id="sync-sliders-to-state-button", color="secondary", outline=True, size="sm"), width="auto")
            ], className="mb-3 justify-content-start"),
            dbc.Row([
                dbc.Col([html.H5("手臂关节 (rad):"), html.Pre(id="arm-states-display", className="bg-light p-2 border rounded", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '150px', 'overflowY': 'auto'})], md=6, className="mb-3 mb-md-0"), # Reduced maxHeight
                dbc.Col([html.H5("头部伺服 (raw):"), html.Pre(id="head-states-display", className="bg-light p-2 border rounded", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '70px', 'overflowY': 'auto'})], md=6) # Reduced maxHeight
            ]),
            # NEW: Placeholder for Hand States (if they become available)
            # dbc.Row([
            #     dbc.Col([html.H5("左手状态:"), html.Pre(id="left-hand-states-display", className="bg-light p-2 border rounded", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '100px', 'overflowY': 'auto'})], md=6, className="mt-3"),
            #     dbc.Col([html.H5("右手状态:"), html.Pre(id="right-hand-states-display", className="bg-light p-2 border rounded", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '100px', 'overflowY': 'auto'})], md=6, className="mt-3")
            # ], className="mt-3")
        ])
    ]

    # --- Trajectory Management Card ---
    trajectory_management_card_content = [
        dbc.CardHeader("轨迹管理 (Trajectory Management)"),
        dbc.CardBody([
            html.Div(id="action-feedback-display", className="mb-3"),
            dbc.Row([
                dbc.Col(dbc.Button("记录当前位置", id="record-position-button", color="success", className="w-100"), width=12, sm=6, md=4, className="mb-2 d-grid"),
            ], className="mb-3"),
            dbc.Form([
                dbc.Row([
                    dbc.Col(dcc.Input(id="trajectory-filename-input", type="text", placeholder="输入轨迹名称...", className="form-control"), width=12, sm=6, md=4, className="mb-2"),
                    dbc.Col(dbc.Button("保存活跃轨迹", id="save-trajectory-button", color="primary", className="w-100"), width=12, sm=6, md=4, className="mb-2 d-grid")
                ], className="mb-2 align-items-center"),
                dbc.Row([
                    dbc.Col(dcc.Dropdown(id="trajectory-select-dropdown", placeholder="选择已保存轨迹...", style={"width": "100%"}), width=12, sm=6, md=4, className="mb-2"),
                    dbc.Col(dbc.Button("刷新列表", id="refresh-trajectory-list-button", color="secondary", outline=True, className="w-100"), width=12, sm=3, md=2, className="mb-2 d-grid"),
                    dbc.Col(dbc.Button("加载选中轨迹", id="load-selected-trajectory-button", color="info", className="w-100"), width=12, sm=3, md=2, className="mb-2 d-grid")
                ], className="mb-3 align-items-center")
            ]),
            html.H5("活跃轨迹中的点 (Active Trajectory Points)", className="mt-3"),
            html.Div(id="recorded-positions-count-display", children="尚未记录或加载任何位置。", className="mb-2 text-muted"),
            html.Div(id="recorded-positions-list-display", style={'maxHeight': '250px', 'overflowY': 'auto', 'border': '1px solid #eee', 'padding': '10px', 'backgroundColor': '#f9f9f9', 'borderRadius': '5px'}) # Reduced maxHeight
        ])
    ]
    
    # --- Trajectory Playback Card ---
    trajectory_playback_card_content = [
        dbc.CardHeader("轨迹回放 (Trajectory Playback)"),
        dbc.CardBody([
            dbc.Row([
                 dbc.Col([html.Label("回放速度 (MoveJ speed):", className="form-label"), dcc.Slider(id="playback-speed-slider", min=0.1, max=1.0, value=config.PLAYBACK_SPEED_DEFAULT, step=0.05, marks={i/10: str(i/10) for i in range(1, 11, 1)}, tooltip={"placement": "top", "always_visible": False})], md=6, className="mb-3"),
                 dbc.Col([html.Label("点间延迟 (秒):", className="form-label"), dcc.Slider(id="playback-delay-slider", min=0.1, max=5.0, value=config.PLAYBACK_DELAY_DEFAULT, step=0.1, marks={round(i*0.5,1): str(round(i*0.5,1)) for i in range(1, 11)}, tooltip={"placement": "top", "always_visible": False})], md=6, className="mb-3")
            ]),
            dbc.Row([
                dbc.Col(dbc.Button("回放一次", id="replay-once-button", color="warning", className="me-2 w-100"), width=12, sm=4, className="mb-2 d-grid"),
                dbc.Col(dbc.Button("开始连续回放", id="start-continuous-replay-button", color="success", className="me-2 w-100"), width=12, sm=4, className="mb-2 d-grid"),
                dbc.Col(dbc.Button("停止连续回放", id="stop-continuous-replay-button", color="danger", disabled=True, className="w-100"), width=12, sm=4, className="mb-2 d-grid")
            ], className="justify-content-start")
        ])
    ]

    layout = dbc.Container([
        dcc.Store(id='playback-state-store', data={'is_repeating': False, 'current_index': 0, 'trajectory_for_repeat': []}),
        dcc.Interval(id='continuous-playback-interval', interval=2000, n_intervals=0, disabled=True),
        dcc.Interval(id='interval-ros-status-update', interval=1000, n_intervals=0),
        dcc.Interval(id='interval-joint-state-update', interval=500, n_intervals=0),

        dbc.Row(dbc.Col(html.H1("机器人控制与监控面板", className="text-center my-4"))),
        ros_connection_section,
        html.Hr(className="my-4"),

        # Main content area with three columns
        dbc.Row([
            dbc.Col([ # Column 1: Arms
                dbc.Card(left_arm_card_content, className="mb-4 shadow-sm"),
                dbc.Card(right_arm_card_content, className="mb-4 shadow-sm")
            ], md=6, lg=3), # Adjusted column widths
            dbc.Col([ # Column 2: Head and Hands
                dbc.Card(head_servo_card_content, className="mb-4 shadow-sm"),
                dbc.Card(left_hand_card_content, className="mb-4 shadow-sm"), # ADDED LEFT HAND
                dbc.Card(right_hand_card_content, className="mb-4 shadow-sm") # ADDED RIGHT HAND
            ], md=6, lg=3), # Adjusted column widths
            dbc.Col([ # Column 3: States and Trajectory Management
                dbc.Card(joint_states_card_content, className="mb-4 shadow-sm"),
                dbc.Card(trajectory_management_card_content, className="mb-4 shadow-sm")
            ], md=6, lg=3), # Adjusted column widths
             dbc.Col([ # Column 4: Trajectory Playback
                dbc.Card(trajectory_playback_card_content, className="mb-4 shadow-sm")
            ], md=6, lg=3)  # Adjusted column widths
        ]),
        
        html.Footer(
            dbc.Row(dbc.Col(html.P("请注意：浏览器刷新页面可能会导致已记录但未保存的位置数据丢失。", className="text-muted text-center small mt-5"))),
        )

    ], fluid=True, className="py-3 bg-light")
    return layout