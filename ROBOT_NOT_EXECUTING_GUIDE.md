# 机器人不执行指令问题诊断和修复指南

## 问题现象
机器人能够接收到指令（通过 `rostopic echo` 可以看到消息），但是不执行动作。

## 可能原因分析

### 1. 角度单位问题 ⭐⭐⭐⭐⭐
**最可能的原因**：机器人驱动期望的角度单位与发送的不匹配。

- **度数 vs 弧度**: 
  - 如果机器人期望弧度，但我们发送度数，会导致角度过小（弧度值约为度数的1/57）
  - 如果机器人期望度数，但我们发送弧度，会导致角度过大（可能超出安全范围被忽略）

### 2. 速度参数问题 ⭐⭐⭐⭐
- 速度过快或过慢可能导致指令被忽略
- 建议尝试 0.1-0.3 的较低速度值

### 3. 轨迹连接模式问题 ⭐⭐⭐
- `trajectory_connect` 参数可能影响指令执行
- 尝试 0（独立指令）和 1（连接轨迹）

### 4. 机器人状态问题 ⭐⭐
- 机器人可能处于错误状态或需要使能
- 机器人安全系统可能阻止了运动

## 快速修复方案

### 方案1: 修改角度单位为弧度
```python
# 在 config.py 中修改:
JOINT_ANGLE_UNIT = 'radians'  # 改为 'radians'
```

### 方案2: 降低速度
```python
# 在 config.py 中修改:
JOINT_CONTROL_SPEED = 0.1  # 降低到 0.1
```

### 方案3: 修改轨迹连接模式
```python
# 在 config.py 中修改:
JOINT_CONTROL_TRAJECTORY_CONNECT = 0  # 改为独立指令模式
```

### 方案4: 组合修改（推荐先尝试）
```python
# 在 config.py 中同时修改:
JOINT_CONTROL_SPEED = 0.1
JOINT_CONTROL_TRAJECTORY_CONNECT = 0
JOINT_ANGLE_UNIT = 'radians'
```

## 诊断步骤

### 1. 运行诊断脚本
```bash
python test_robot_diagnosis.py
```

### 2. 手动测试不同配置
```python
# 在 Python 中测试
import ros_comms.handler as ros_handler

# 连接
ros_handler.try_connect_ros()

# 测试弧度模式
ros_handler.set_joint_angle_unit(use_radians=True)
ros_handler.send_right_arm_direct_command([1, 0, 0, 0, 0, 0, 0])

# 测试度数模式
ros_handler.set_joint_angle_unit(use_radians=False) 
ros_handler.send_right_arm_direct_command([1, 0, 0, 0, 0, 0, 0])
```

### 3. 检查机器人端状态
```bash
# 检查机器人话题状态
rostopic list | grep arm
rostopic info /r_arm/rm_driver/MoveJ_Cmd

# 检查机器人节点状态
rosnode list | grep arm
rosnode info <arm_node_name>
```

## 常见解决方案

### 解决方案A: 确认是弧度问题
如果测试发现是角度单位问题，确认机器人使用弧度：

1. 修改 `config.py`:
```python
JOINT_ANGLE_UNIT = 'radians'
```

2. 或者在代码中动态设置:
```python
ros_handler.set_joint_angle_unit(use_radians=True)
```

### 解决方案B: 确认是速度问题
如果速度过快，修改配置：

```python
JOINT_CONTROL_SPEED = 0.1  # 或更低
```

### 解决方案C: 机器人状态问题
检查机器人是否需要使能或复位：

1. 查看机器人手册确认启动流程
2. 检查是否有错误状态话题
3. 可能需要发送使能指令

### 解决方案D: 消息格式问题
确认消息类型和字段名称正确：

```bash
rosmsg show dual_arm_msgs/MoveJ
```

## 测试建议

### 安全测试
1. 始终使用小角度测试（1-5度）
2. 使用低速度（0.1-0.3）
3. 一次只测试一个关节

### 系统性测试
1. 先测试角度单位
2. 再测试速度参数
3. 最后测试轨迹模式

### 日志观察
关注日志中的：
- 连接状态
- 发布器状态
- 消息发送成功/失败
- 任何错误信息

## 如果问题仍然存在

1. **检查机器人端日志**：查看机器人控制器的日志输出
2. **确认话题名称**：确保话题名称与机器人实际监听的一致
3. **检查权限**：确认是否有权限控制机器人
4. **联系技术支持**：提供详细的测试结果和日志

## 快速验证脚本

运行以下测试来快速确定问题：

```bash
# 1. 基础诊断
python test_robot_diagnosis.py

# 2. 角度单位测试
python test_angle_units.py

# 3. 增强控制测试
python test_enhanced_control.py
```

根据这些测试的结果，调整配置文件中的相应参数。
