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

    # --- Left Hand Control Card ---
    left_hand_card_content = [
        dbc.CardHeader("左手控制 (Left Hand Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"{config.LEFT_HAND_DOF_NAMES[i]}"),
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

    # --- Right Hand Control Card ---
    right_hand_card_content = [
        dbc.CardHeader("右手控制 (Right Hand Control)"),
        dbc.CardBody([
            *[html.Div([
                html.Label(f"{config.RIGHT_HAND_DOF_NAMES[i]}"),
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
    ])

    # --- Navigation Control Card ---
    nav_control_card_content = [
        dbc.CardHeader("导航控制 (Navigation Control)"),
        dbc.CardBody([
            html.Div([
                html.Label("选择导航目标点:", className="form-label"),
                dcc.Dropdown(
                    id="nav-point-dropdown",
                    options=[{'label': point, 'value': point} for point in config.PREDEFINED_NAV_POINTS],
                    placeholder="选择一个目标点...",
                    className="mb-3"
                )
            ]),
            html.Div(dbc.Button("发送导航指令", id="send-nav-command-button", color="primary", className="mt-2 w-100"), className="d-grid")
        ])
    ]

    # --- NEW: Individual State Display Cards ---
    left_arm_states_card_content = [
        dbc.CardHeader("左臂状态 (rad)"),
        dbc.CardBody(dbc.Alert(html.Pre(id="left-arm-states-display", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'margin': '0'}), color="secondary", className="mb-0 p-2"))
    ]
    right_arm_states_card_content = [
        dbc.CardHeader("右臂状态 (rad)"),
        dbc.CardBody(dbc.Alert(html.Pre(id="right-arm-states-display", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'margin': '0'}), color="secondary", className="mb-0 p-2"))
    ]
    head_states_card_content = [
        dbc.CardHeader("头部状态 (raw)"),
        dbc.CardBody(dbc.Alert(html.Pre(id="head-states-display", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'margin': '0'}), color="secondary", className="mb-0 p-2"))
    ]

    # --- NEW: Global Actions Card ---
    global_actions_card_content = [
        dbc.CardHeader("全局操作 (Global Actions)"),
        dbc.CardBody(
            dbc.Row([
                dbc.Col(dbc.Button("手动刷新所有状态", id="refresh-states-button", color="info", outline=True, className="w-100"), md=6, className="d-grid mb-2 mb-md-0"),
                dbc.Col(dbc.Button("同步滑块至状态", id="sync-sliders-to-state-button", color="secondary", outline=True, className="w-100"), md=6, className="d-grid")
            ])
        )
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
                    dbc.Col(dcc.Input(id="trajectory-filename-input", type="text", placeholder="输入轨迹名称...", className="form-control"), width=12, sm=12, md=4, className="mb-2"),
                    dbc.Col(dbc.Button("保存活跃轨迹", id="save-trajectory-button", color="primary", className="w-100"), width=12, sm=12, md=4, className="mb-2 d-grid")
                ], className="mb-2 align-items-center"),
                dbc.Row([
                    dbc.Col(dcc.Dropdown(id="trajectory-select-dropdown", placeholder="选择已保存轨迹...", style={"width": "100%"}), width=12, sm=12, md=4, className="mb-2"),
                    dbc.Col(dbc.Button("刷新列表", id="refresh-trajectory-list-button", color="secondary", outline=True, className="w-100"), width=12, sm=12, md=2, className="mb-2 d-grid"),
                    dbc.Col(dbc.Button("加载选中轨迹", id="load-selected-trajectory-button", color="info", className="w-100"), width=12, sm=12, md=2, className="mb-2 d-grid")
                ], className="mb-3 align-items-center")
            ]),
            html.H5("活跃轨迹中的点 (Active Trajectory Points)", className="mt-3"),
            html.Div(id="recorded-positions-count-display", children="尚未记录或加载任何位置。", className="mb-2 text-muted"),
            html.Div(id="recorded-positions-list-display", style={'maxHeight': '250px', 'overflowY': 'auto', 'border': '1px solid #eee', 'padding': '10px', 'backgroundColor': '#f9f9f9', 'borderRadius': '5px'})
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
    # --- NEW: Pose Control Card ---
    pose_control_card_content = [
        dbc.CardHeader("末端位姿控制 (End-Effector Pose Control)"),
        dbc.CardBody([
            dbc.Row([
                dbc.Col(html.Label("选择控制组:"), width=12),
                dbc.Col(
                    dbc.RadioItems(
                        id="pose-control-arm-select",
                        options=[
                            {'label': '左臂', 'value': config.PLANNING_GROUP_LEFT_ARM},
                            {'label': '右臂', 'value': config.PLANNING_GROUP_RIGHT_ARM},
                        ],
                        value=config.PLANNING_GROUP_LEFT_ARM, # 默认选中左臂
                        inline=True,
                        className="mb-3"
                    ),
                ),
            ]),
            html.Label("目标位置 (米):"),
            dbc.Row([
                dbc.Col(dbc.Input(id="pose-pos-x", placeholder="X", type="number"), className="mb-2"),
                dbc.Col(dbc.Input(id="pose-pos-y", placeholder="Y", type="number"), className="mb-2"),
                dbc.Col(dbc.Input(id="pose-pos-z", placeholder="Z", type="number"), className="mb-2"),
            ]),
            html.Label("目标姿态 (四元数):"),
            dbc.Row([
                dbc.Col(dbc.Input(id="pose-ori-w", placeholder="Qw", type="number", value=1.0)), # 默认值
                dbc.Col(dbc.Input(id="pose-ori-x", placeholder="Qx", type="number", value=0.0)),
                dbc.Col(dbc.Input(id="pose-ori-y", placeholder="Qy", type="number", value=0.0)),
                dbc.Col(dbc.Input(id="pose-ori-z", placeholder="Qz", type="number", value=0.0)),
            ], className="mb-3"),
            html.Div(
                dbc.Button("发送位姿目标", id="send-pose-goal-button", color="warning", className="w-100"),
                className="d-grid"
            )
        ])
    ]

    # --- Main Layout Definition ---
    layout = dbc.Container([
        # Hidden stores and intervals
        dcc.Store(id='playback-state-store', data={'is_repeating': False, 'current_index': 0, 'trajectory_for_repeat': []}),
        dcc.Interval(id='continuous-playback-interval', interval=2000, n_intervals=0, disabled=True),
        dcc.Interval(id='interval-ros-status-update', interval=1000, n_intervals=0),
        dcc.Interval(id='interval-joint-state-update', interval=500, n_intervals=0),

        # --- Header ---
        dbc.Row(dbc.Col(html.H1("机器人控制与监控面板", className="text-center my-4"))),
        
        # --- Top Section: Connection, Head ---
        dbc.Row([
            dbc.Col(ros_connection_section, width=12, lg=4), # Top-Left
            dbc.Col([ # Top-Center
                dbc.Card(head_servo_card_content, className="mb-2"),
                dbc.Card(head_states_card_content) # Head status directly below
            ], width=12, lg=4),
            dbc.Col(width=12, lg=4) # Empty spacer column
        ], align="start", justify="start", className="mb-4"),

        html.Hr(),

        # --- Main Workspace: Arms, Trajectory, and States ---
        dbc.Row([
            # -- Left Column: Left Arm Control and State --
            dbc.Col([
                dbc.Card(left_arm_card_content, className="mb-4"),
                dbc.Card(left_hand_card_content, className="mb-4"),
                dbc.Card(left_arm_states_card_content, className="mb-4")
            ], width=12, lg=3),

            # -- Center Column: Trajectory and Global Actions --
            dbc.Col([
                dbc.Card(global_actions_card_content, className="mb-4"),
                dbc.Card(pose_control_card_content, className="mb-4"),
                dbc.Card(trajectory_management_card_content, className="mb-4"),
                dbc.Card(trajectory_playback_card_content, className="mb-4"),
            ], width=12, lg=6),

            # -- Right Column: Right Arm Control and State --
            dbc.Col([
                dbc.Card(right_arm_card_content, className="mb-4"),
                dbc.Card(right_hand_card_content, className="mb-4"),
                dbc.Card(right_arm_states_card_content, className="mb-4")
            ], width=12, lg=3),
        ]),

        html.Hr(className="my-4"),

        # --- Bottom Section: Navigation ---
        dbc.Row(
            dbc.Col(dbc.Card(nav_control_card_content, className="mb-4"), width=12, lg=8),
            justify="center"
        ),

        # --- Footer ---
        html.Footer(
            dbc.Row(dbc.Col(html.P("请注意：浏览器刷新页面可能会导致已记录但未保存的位置数据丢失。", className="text-muted text-center small mt-5"))),
        )

    ], fluid=True, className="py-3 bg-light")
    return layout