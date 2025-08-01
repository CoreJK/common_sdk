# armpi_common

幻尔六轴串联机械臂二次开发 SDK

## 项目简介

`armpi_common` 是一个用于控制幻尔六轴串联机械臂的 Python SDK。该 SDK 提供了完整的舵机通信协议实现，支持舵机的各种控制功能，包括位置控制、电机控制、参数配置等。

## 功能特性

- ✅ **完整的写指令支持** (14/14)
  - 舵机位置和电机模式控制
  - 角度和时间控制运动
  - 电压和温度限制设置
  - LED 控制和错误处理
  - 紧急停止功能

- 🔄 **读指令支持** (开发中)
  - 舵机状态读取
  - 位置、温度、电压监控
  - 配置参数读取

- 🛠️ **工具函数**
  - 校验和计算
  - 字节操作工具
  - 协议解析工具

## 安装

### 环境要求

- Python >= 3.8
- 支持串口通信的硬件设备

### 安装依赖

```bash
# 使用 PDM 安装依赖
pdm install

# 或者使用 pip
pip install pyserial-asyncio>=0.6
```

## 快速开始

### 基本使用

```python
from armpi_common.robot_arm_controller import RobotArmController

# 创建控制器实例
controller = RobotArmController(device='/dev/ttyUSB0', baudrate=115200)

# 设置舵机为位置控制模式
controller.set_joint_mode(1, 0, 0)

# 设置舵机角度和时间
controller.set_joint_angle_use_time(1, 300, 1000)  # 关节1，角度300，时间1000ms

# 启动运动
controller.set_joint_move_start(1)

# 紧急停止
controller.set_joint_emergency_stop(1)
```

### 电机控制模式

```python
# 设置为电机控制模式，正转速度500
controller.set_joint_mode(1, 1, 500)

# 设置为电机控制模式，反转速度-500
controller.set_joint_mode(1, 1, -500)
```

### 参数配置

```python
# 设置电压限制
controller.set_joint_vin_limit(1, 6000, 12000)  # 6V-12V

# 设置温度限制
controller.set_joint_temp_limit_range(1, 85)  # 85°C

# 设置角度限制
controller.set_joint_angle_limit(1, 0, 1000)  # 0-1000度

# 设置LED控制
controller.set_joint_led(1, 0)  # LED常亮
```

## API 文档

### RobotArmController

主要的机械臂控制器类。

#### 构造函数

```python
RobotArmController(device='/dev/ttyUSB0', baudrate=115200, timeout=0)
```

**参数：**
- `device`: 串口设备路径
- `baudrate`: 波特率，默认115200
- `timeout`: 超时时间

#### 核心方法

##### 运动控制

- `set_joint_angle_use_time(joint_id, angle, time)`: 设置关节角度和运动时间
- `set_joint_angle_with_time_after_start(joint_id, angle, delay_time)`: 设置延迟运动
- `set_joint_move_start(joint_id)`: 启动关节运动
- `set_joint_emergency_stop(joint_id)`: 紧急停止

##### 模式控制

- `set_joint_mode(joint_id, servo_mode, speed)`: 设置舵机工作模式
  - `servo_mode`: 0-位置控制模式，1-电机控制模式
  - `speed`: 转动速度，范围-1000~1000

##### 参数配置

- `set_joint_vin_limit(joint_id, vin_min, vin_max)`: 设置电压限制
- `set_joint_temp_limit_range(joint_id, temp_limit)`: 设置温度限制
- `set_joint_angle_limit(joint_id, angle_min, angle_max)`: 设置角度限制
- `set_joint_angle_offset_adjust(joint_id, angle_offset)`: 临时角度偏移调整
- `set_joint_angle_offset_write(joint_id, angle_offset)`: 永久角度偏移设置

##### 状态控制

- `set_joint_load_or_unload(joint_id, load_or_unload)`: 设置负载状态
- `set_joint_led(joint_id, led_ctrl)`: 设置LED控制
- `set_joint_led_error(joint_id, led_error)`: 设置LED错误报警

##### 系统配置

- `set_joint_id(joint_id, new_id)`: 设置舵机ID

## 通信协议

### 数据包格式

指令有两种，写指令和读指令。写指令：后面一般带有参数，将相应功能的参数写进舵机，来完成某种动作。读指令：后面一般不带参数，舵机接收到读指令后会立即返回相应数据，返回的指令值和发送给舵机的"读指令"值相同，并且带有参数。

| 帧头 | ID  | 数据长度（length） | 指令(cmd) | 参数(parm小端) | 校验和 |
| :--: | :--: | :--: | :--: | :--: | :--: |
| 0x55 0x55| 0x01 | 0x07 | 0x01 | 0x1f4 0x3e8| 0x16 |

### 校验和计算公式

```
Checksum = ~(ID + Length + cmd + Parm 1 + parm N)
```

若括号內的计算和超出 255 则取最低的一个字节，"～" 表示按位取反

### 支持的指令

完整的指令表请参考 `src/armpi_common/cmdTable.py`

## 开发状态

- ✅ 写指令接口：100% 完成 (14/14)
- 🔄 读指令接口：开发中 (0/14)
- ✅ 工具函数：完成
- ✅ 协议解析：完成

## 贡献

欢迎提交 Issue 和 Pull Request！

## 许可证

MIT License

## 作者

任伟明 (corejk@163.com)