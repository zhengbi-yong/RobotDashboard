# 机器人动作控制流程详解

## 当您点击"握手"按钮时的完整控制流程

### 1. 前端UI层面 (`components/layout.py`)

**按钮创建**：
```python
# 在create_action_buttons()函数中
dbc.Button(
    "握手",  # 按钮显示的文字
    id="action-woshou",  # 按钮的唯一ID
    color="primary",
    outline=True,
    size="sm",
    className="me-2 mb-2"
)
```

### 2. 回调处理层 (`callbacks/ui_callbacks.py`)

**按钮点击检测**：
```python
@app.callback(
    Output("robot-action-feedback", "children", allow_duplicate=True),
    [Input("action-woshou", "n_clicks")],  # 监听握手按钮的点击
    prevent_initial_call=True
)
def handle_action_buttons(*n_clicks_list):
    # 1. 检测哪个按钮被点击
    ctx = dash.callback_context
    triggered_id = ctx.triggered[0]['prop_id'].split('.')[0]
    
    # 2. 提取动作ID: "action-woshou" -> "woshou"
    action_id = triggered_id.replace("action-", "")
    
    # 3. 调用ROS处理器发送动作指令
    success = ros_handler.send_force_action_command("woshou")
    
    # 4. 返回UI反馈
    if success:
        return "已发送动作指令: 握手 (woshou)"
    else:
        return "动作指令发送失败: 握手"
```

### 3. ROS通信处理层 (`ros_comms/handler.py`)

**HTTP API请求发送**：
```python
def send_force_action_command(action_id):
    """发送强制动作指令"""
    data = {
        "force_command_str": "woshou",  # 动作ID
        "is_force_input": True          # 强制执行标志
    }
    return send_robot_action_request('force_command', data)

def send_robot_action_request(endpoint, data):
    """发送HTTP请求到机器人动作服务器"""
    # 1. 构建完整URL
    url = "http://117.147.181.50:5198/agent_service/force_command/"
    
    # 2. 设置请求头
    headers = {'Content-Type': 'application/json'}
    
    # 3. 发送POST请求
    response = requests.post(url, headers=headers, json=data, timeout=5)
    
    # 4. 检查响应状态
    return response.status_code == 200
```

### 4. 机器人服务端处理

**HTTP请求接收**：
- 机器人的动作服务器（运行在 `117.147.181.50:5198`）接收到请求
- 请求数据：`{"force_command_str": "woshou", "is_force_input": true}`

**动作解析与执行**：
1. 服务器解析 `"woshou"` 动作ID
2. 查找对应的握手动作序列
3. 将动作转换为具体的关节角度或轨迹指令
4. 通过机器人底层驱动执行动作

### 5. 完整的数据流向

```
用户点击"握手"按钮
        ↓
Dash UI回调触发 (ui_callbacks.py)
        ↓
提取动作ID: "woshou"
        ↓
调用 send_force_action_command("woshou")
        ↓
构建HTTP请求数据: {"force_command_str": "woshou", "is_force_input": true}
        ↓
发送POST请求到: http://117.147.181.50:5198/agent_service/force_command/
        ↓
机器人动作服务器接收请求
        ↓
解析"woshou"动作并执行对应的关节运动
        ↓
机器人执行握手动作
```

### 6. 配置信息 (`config.py`)

```python
# API服务器配置
ROBOT_ACTION_API_BASE_URL = 'http://117.147.181.50:5198'
ROBOT_ACTION_ENDPOINTS = {
    'force_command': '/agent_service/force_command/'
}

# 握手动作定义
ROBOT_ACTIONS = [
    {'id': 'woshou', 'name': '握手', 'category': '交互'},
    # ... 其他动作
]
```

### 7. 关键特点

1. **HTTP通信**：使用HTTP REST API与机器人通信，不依赖ROS话题
2. **动作抽象**：通过动作ID（如"woshou"）抽象复杂的关节运动
3. **异步执行**：前端发送指令后立即返回，机器人独立执行动作
4. **错误处理**：包含网络超时、连接失败等错误处理机制
5. **用户反馈**：实时显示指令发送状态和执行结果

### 8. 调试信息

如果您想查看详细的执行过程，可以查看日志输出：
- 前端会显示"已发送动作指令: 握手 (woshou)"
- 后台日志会记录HTTP请求的URL、数据和响应状态
- 如果失败，会显示具体的错误信息

这种设计的优势是将复杂的机器人控制逻辑封装在服务器端，前端只需要发送简单的动作指令即可，大大简化了客户端的复杂度。
