# 机器人动作控制功能说明

## 功能概述

已为机器人控制界面添加了完整的动作控制功能，包括语音控制和预设动作执行。

## 新增功能

### 1. 语音控制功能

#### 1.1 语音识别文本设置
- **功能**: 替换当前的语音识别结果
- **界面**: 文本输入框 + "设置语音识别" 按钮
- **API**: `POST /agent_service/set_manual_asr_result/`
- **参数**: 
  - `vad_state`: "end"
  - `transcript`: 用户输入的文本

#### 1.2 直接说话指令
- **功能**: 让机器人直接说出指定内容
- **界面**: 文本输入框 + "直接说话" 按钮  
- **API**: `POST /agent_service/speak/`
- **参数**:
  - `text`: 用户输入的文本

### 2. 预设动作控制

按类别组织的动作按钮，点击即可执行对应动作：

#### 交互动作
- 握手 (woshou)
- 挥手 (huishou)
- 招手 (zhaoshou)
- 欢迎 (huanying)
- 欢迎客人 (huanyingkeren)
- 拥抱 (yongbao)
- 碰拳 (pengquan)

#### 娱乐动作
- 跳舞 (tiaowu)
- 手臂舞 (shoubiwu)
- 手势舞蹈 (shoushiwu)

#### 礼仪动作
- 敬礼 (jingli)

#### 特色动作
- 招财猫 (zhaocaimao)
- 兰花指 (lanhuazhi)
- 蜘蛛侠 (zhizhuxia)

#### 服务动作
- 递包 (dibao)
- 保管 (baogaun)
- 拿包 (nabao)
- 记账 (jizhang)

#### 手势动作
- 放手 (fangshou)
- 松手 (songshou)
- 握紧 (wojin)

#### 表情动作
- 挠头 (naotou)
- 挑衅 (tiaoxin)
- 捂脸 (wulian)
- 左手捂脸 (zuoshouwulian)
- 思考 (sikao)

#### 指示动作
- 指天 (zhitian)
- 指挥 (zhihui)
- 指天空 (zhitiankong)

#### 数字动作
- 数字0 (num0)
- 数字1 (num1)
- 数字2 (num2)
- 数字3 (num3)
- 数字4 (num4)

#### 其他动作
- 擦玻璃 (caboli)
- 石头剪刀布 (shitoujiandaobu)

## 技术实现

### 1. 配置文件 (config.py)
```python
# API配置
ROBOT_ACTION_API_BASE_URL = 'http://117.147.181.50:5198'
ROBOT_ACTION_ENDPOINTS = {
    'set_manual_asr_result': '/agent_service/set_manual_asr_result/',
    'speak': '/agent_service/speak/',
    'force_command': '/agent_service/force_command/'
}

# 动作列表
ROBOT_ACTIONS = [
    {'id': 'woshou', 'name': '握手', 'category': '交互'},
    # ... 更多动作
]
```

### 2. HTTP客户端 (ros_comms/handler.py)
```python
def send_asr_transcript(transcript_text):
    """发送语音识别文本设置请求"""

def send_speak_command(text):
    """发送直接说话指令"""

def send_force_action_command(action_id):
    """发送强制动作指令"""
```

### 3. 用户界面 (components/layout.py)
- 按类别分组的动作按钮
- 语音控制输入框和按钮
- 实时反馈显示区域

### 4. 事件处理 (callbacks/ui_callbacks.py)
- 语音识别设置回调
- 直接说话回调
- 动作按钮点击回调

## 使用方法

### 启动应用
```bash
python app.py
```

### 语音控制
1. 在"输入要替换的语音识别文本"框中输入文本
2. 点击"设置语音识别"按钮

或者：

1. 在"输入要说的话"框中输入文本
2. 点击"直接说话"按钮

### 动作控制
1. 在"机器人动作控制"面板中找到所需动作
2. 点击对应的动作按钮
3. 观察反馈信息确认执行状态

## 测试工具

### 命令行测试
```bash
# 列出所有可用动作
python test_robot_actions.py --list

# 测试特定动作
python test_robot_actions.py --test woshou

# 运行所有测试
python test_robot_actions.py --all
```

### Web界面测试
1. 启动应用后访问 http://localhost:8050
2. 滚动到"机器人动作控制"部分
3. 测试各种功能

## 状态反馈

所有操作都有实时反馈：
- ✅ 成功：绿色提示框显示操作成功信息
- ❌ 失败：红色提示框显示错误信息
- 📝 详细日志：控制台输出详细的执行日志

## 网络要求

- 确保能够访问 `http://117.147.181.50:5198`
- 网络延迟应在合理范围内（< 5秒超时）

## 故障排除

### 常见问题

1. **请求超时**
   - 检查网络连接
   - 确认API服务器状态

2. **动作无响应**
   - 查看控制台日志
   - 确认动作ID是否正确

3. **界面显示问题**
   - 刷新页面
   - 检查浏览器控制台错误

### 调试方法

1. 查看应用日志
2. 使用测试脚本验证功能
3. 检查网络连通性

## 扩展方法

### 添加新动作
1. 在 `config.py` 的 `ROBOT_ACTIONS` 列表中添加新动作
2. 重启应用即可在界面中看到新按钮

### 修改API地址
1. 在 `config.py` 中修改 `ROBOT_ACTION_API_BASE_URL`
2. 重启应用

### 自定义动作分类
1. 在动作配置中使用新的 `category` 值
2. 界面会自动按类别分组显示

这个功能完全集成到现有的机器人控制界面中，提供了丰富的机器人动作控制能力。
