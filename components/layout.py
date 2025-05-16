# your_robot_dashboard/components/layout.py

from dash import dcc, html
import dash_bootstrap_components as dbc
from .. import config # Relative import

def create_layout():
    layout = dbc.Container([
        dcc.Store(id='playback-state-store', data={'is_repeating': False, 'current_index': 0, 'trajectory_for_repeat': []}),
        dcc.Interval(id='continuous-playback-interval', interval=2000, n_intervals=0, disabled=True),
        html.H1("机器人关节控制与状态监控 (Dash)"),
        dbc.Row([
            dbc.Col([
                html.Button("Connect to ROS", id="connect-ros-button", n_clicks=0, className="btn btn-primary mb-2"),
                html.Div(id="ros-connection-status-display", children=f"Status: Disconnected") # Initial status
            ])
        ]),
        html.P("请注意：浏览器刷新页面可能会导致已记录但未保存的位置数据丢失。", className="alert alert-warning"),
        dcc.Interval(id='interval-ros-status-update', interval=1000, n_intervals=0),
        dcc.Interval(id='interval-joint-state-update', interval=500, n_intervals=0),
        dbc.Row([
            dbc.Col([
                html.H3("左臂控制"),
                *[html.Div([
                    html.Label(f"左臂关节 {i+1} ({config.LEFT_ARM_JOINT_NAMES_INTERNAL[i]})"),
                    dcc.Slider(id=f"l_arm_slider_{i}", min=config.ARM_SLIDER_MIN, max=config.ARM_SLIDER_MAX, value=config.ARM_SLIDER_DEFAULT, step=config.ARM_SLIDER_STEP, marks={j: str(j) for j in range(int(config.ARM_SLIDER_MIN), int(config.ARM_SLIDER_MAX)+1, config.ARM_SLIDER_MARKS_STEP)}, tooltip={"placement": "top", "always_visible": False})
                ], className="mb-2") for i in range(7)],
                html.Button("发送左臂控制指令", id="send-left-arm-button", n_clicks=0, className="btn btn-info mt-2")
            ], md=4),
            dbc.Col([
                html.H3("右臂控制"),
                *[html.Div([
                    html.Label(f"右臂关节 {i+1} ({config.RIGHT_ARM_JOINT_NAMES_INTERNAL[i]})"),
                    dcc.Slider(id=f"r_arm_slider_{i}", min=config.ARM_SLIDER_MIN, max=config.ARM_SLIDER_MAX, value=config.ARM_SLIDER_DEFAULT, step=config.ARM_SLIDER_STEP, marks={j: str(j) for j in range(int(config.ARM_SLIDER_MIN), int(config.ARM_SLIDER_MAX)+1, config.ARM_SLIDER_MARKS_STEP)}, tooltip={"placement": "top", "always_visible": False})
                ], className="mb-2") for i in range(7)],
                html.Button("发送右臂控制指令", id="send-right-arm-button", n_clicks=0, className="btn btn-info mt-2")
            ], md=4),
            dbc.Col([
                html.H3("头部伺服控制"),
                html.Div([
                    html.Label(f"头部俯仰 (ID {config.HEAD_SERVO_RANGES['head_tilt_servo']['id']})"),
                    dcc.Slider(id="head-tilt-slider", min=config.HEAD_SERVO_RANGES['head_tilt_servo']['min'], max=config.HEAD_SERVO_RANGES['head_tilt_servo']['max'], value=config.HEAD_SERVO_RANGES['head_tilt_servo']['neutral'], step=10, marks={j: str(j) for j in range(config.HEAD_SERVO_RANGES['head_tilt_servo']['min'], config.HEAD_SERVO_RANGES['head_tilt_servo']['max'] + 1, 100)}, tooltip={"placement": "top", "always_visible": False})
                ], className="mb-2"),
                html.Div([
                    html.Label(f"头部左右 (ID {config.HEAD_SERVO_RANGES['head_pan_servo']['id']})"),
                    dcc.Slider(id="head-pan-slider", min=config.HEAD_SERVO_RANGES['head_pan_servo']['min'], max=config.HEAD_SERVO_RANGES['head_pan_servo']['max'], value=config.HEAD_SERVO_RANGES['head_pan_servo']['neutral'], step=10, marks={j: str(j) for j in range(config.HEAD_SERVO_RANGES['head_pan_servo']['min'], config.HEAD_SERVO_RANGES['head_pan_servo']['max'] + 1, 200)}, tooltip={"placement": "top", "always_visible": False})
                ], className="mb-2"),
                html.Button("发送头部伺服控制指令", id="send-head-servo-button", n_clicks=0, className="btn btn-info mt-2")
            ], md=4),
        ]),
        html.Hr(),
        html.H3("当前关节状态"),
        dbc.Row([
            dbc.Col(html.Button("手动刷新状态显示", id="refresh-states-button", n_clicks=0, className="btn btn-sm btn-secondary mb-2"), width="auto"),
            dbc.Col(html.Button("同步滑块至当前状态", id="sync-sliders-to-state-button", n_clicks=0, className="btn btn-sm btn-outline-info mb-2"), width="auto")
        ], className="mb-3 justify-content-start"),
        dbc.Row([
            dbc.Col([html.H5("手臂关节 (rad):"), html.Pre(id="arm-states-display", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '200px', 'overflowY': 'auto', 'border': '1px solid #ccc', 'padding': '10px'})], md=6),
            dbc.Col([html.H5("头部伺服 (raw value):"), html.Pre(id="head-states-display", style={'whiteSpace': 'pre-wrap', 'wordBreak': 'break-all', 'maxHeight': '100px', 'overflowY': 'auto', 'border': '1px solid #ccc', 'padding': '10px'})], md=6)
        ]),
        html.Hr(),
        html.H3("位置记录、保存与加载 (活跃轨迹管理)"),
        html.Div(id="action-feedback-display", className="mb-2"),
        dbc.Row([
            dbc.Col(html.Button("记录当前位置到活跃轨迹", id="record-position-button", n_clicks=0, className="btn btn-success mr-2"), width="auto"),
        ], className="mb-2 justify-content-start"),
        dbc.Row([
            dbc.Col(dcc.Input(id="trajectory-filename-input", type="text", placeholder="输入轨迹名称 (例如 my_sequence)", className="mr-2", style={"width": "250px"}), width="auto"),
            dbc.Col(html.Button("保存活跃轨迹", id="save-trajectory-button", n_clicks=0, className="btn btn-primary"), width="auto")
        ], className="mb-2 justify-content-start"),
        dbc.Row([
            dbc.Col(dcc.Dropdown(id="trajectory-select-dropdown", placeholder="选择已保存的轨迹...", className="mr-2", style={"width": "250px"}), width="auto"),
            dbc.Col(html.Button("刷新轨迹列表", id="refresh-trajectory-list-button", n_clicks=0, className="btn btn-secondary mr-2"), width="auto"),
            dbc.Col(html.Button("加载选中轨迹到活跃区", id="load-selected-trajectory-button", n_clicks=0, className="btn btn-info"), width="auto"),
        ],className="mb-3 justify-content-start"),
        html.Hr(),
        html.H3("活跃轨迹回放"),
        dbc.Row([
             dbc.Col([html.Label("回放速度 (MoveJ speed param, 0.1 - 1.0):"), dcc.Slider(id="playback-speed-slider", min=0.1, max=1.0, value=config.PLAYBACK_SPEED_DEFAULT, step=0.05, marks={i/10: str(i/10) for i in range(1, 11, 1)}, tooltip={"placement": "top", "always_visible": False})], md=6),
             dbc.Col([html.Label("点间延迟 (秒, 0.1 - 5.0):"), dcc.Slider(id="playback-delay-slider", min=0.1, max=5.0, value=config.PLAYBACK_DELAY_DEFAULT, step=0.1, marks={round(i*0.5,1): str(round(i*0.5,1)) for i in range(1, 11)}, tooltip={"placement": "top", "always_visible": False})], md=6)
        ]),
        dbc.Row([
            dbc.Col(html.Button("回放活跃轨迹 (一次)", id="replay-once-button", n_clicks=0, className="btn btn-warning mr-2"), width="auto"),
            dbc.Col(html.Button("开始连续回放", id="start-continuous-replay-button", n_clicks=0, className="btn btn-success mr-2"), width="auto"),
            dbc.Col(html.Button("停止连续回放", id="stop-continuous-replay-button", n_clicks=0, className="btn btn-danger", disabled=True), width="auto")
        ], className="mb-2 justify-content-start"),
        html.H4("活跃轨迹中的位置点 (当前可操作)"),
        html.Div(id="recorded-positions-count-display", children="尚未记录或加载任何位置到活跃轨迹。"),
        html.Div(id="recorded-positions-list-display", style={'maxHeight': '300px', 'overflowY': 'auto', 'border': '1px solid #eee', 'padding': '10px', 'backgroundColor': '#f9f9f9'})
    ], fluid=True)
    return layout