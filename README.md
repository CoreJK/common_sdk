# armpi_common

幻尔六轴串联机械臂二次开发 SDK

## 项目简介

`armpi_common` 是一个用于控制幻尔六轴串联机械臂的 Python SDK。该 SDK 提供了完整的舵机通信协议实现，支持舵机的各种控制功能，包括位置控制、电机控制、参数配置等。

## 项目结构

```
common_sdk/
├── src/armpi_common/                    # 核心源代码包
│   ├── __init__.py                      # 包初始化文件
│   ├── robot_arm_controller.py          # 机械臂主控制器（核心类）
│   ├── armipi_module.py                 # 机械臂模型和角度转换
│   ├── cmdTable.py                      # 通信协议指令表
│   ├── utils.py                         # 工具函数集合
│   ├── _log.py                          # 日志配置模块
│   └── model/                           # 机械臂3D模型文件
│       ├── armpi_fpv.urdf               # URDF机械臂模型
│       └── meshes/                      # 3D网格文件
│           ├── base_link.STL            # 底座模型
│           ├── link1.STL ~ link5.STL    # 关节1-5模型
│           ├── camera_link.STL          # 相机模型
│           ├── gripper_base.STL         # 夹爪底座
│           └── l_*.STL, r_*.STL         # 夹爪左右部分
├── examples/                            # 示例代码
├── tests/                               # 测试代码
│   ├── test_case/                       # 测试用例
│   │   ├── test_robot_controller.py     # 控制器测试
│   │   └── robot_config.ini             # 测试配置
│   └── __init__.py                      # 测试包初始化
├── dist/                                # 构建产物
│   ├── armpi_common-0.1.0-py3-none-any.whl  # Python wheel包
│   └── armpi_common-0.1.0.tar.gz            # 源码包
├── pyproject.toml                       # 项目配置文件
├── requirements.txt                     # Python依赖列表
├── pdm.lock                            # PDM锁定文件
└── README.md                           # 项目说明文档
```

### 核心模块说明

- **`robot_arm_controller.py`**: 主要的机械臂控制器类，提供完整的API接口
- **`armipi_module.py`**: 机械臂模型定义，包含角度和脉冲转换功能
- **`cmdTable.py`**: 完整的舵机通信协议指令表
- **`utils.py`**: 通用工具函数，包括校验和计算、字节操作等
- **`_log.py`**: 统一的日志配置和管理

## 机械臂参数

### MDH 参数


```python
# 连杆长度(m)
# 底座的高度，这里把第一个坐标系和第二个坐标的原点重合到一起了
base_link = 0.064605

link1 = 0.10048
link2 = 0.094714

# 计算tool_link时取值为link3 + tool_link，因为把末端的坐标系原点和前一个重合到一起了
# 这里的tool_link指实际上的夹持器长度
link3 = 0.05071
tool_link = 0.1126
```

**Modified DH**
| i | α(i-1) | a(i-1) |       θ(i)      | d(i) |
|:--:|:--:|:--:|:--:|:--:|
| 1 |   0°   |   0    |  θ1(-120, 120)  |   0  |
| 2 |  -90°  |   0    |  θ2(-180, 0)    |   0  |
| 3 |   0°   | link1  |  θ3(-120, 120)  |   0  |
| 4 |   0°   | link2  |  θ4(-200, 20)   |   0  |
| 5 |  -90°  |   0    |  θ5(-120, 120)  |   0  |

### 舵机运动方向

**正视图**
![正视图](https://s2.loli.net/2025/08/11/E2PYLyXqMmUQNdx.png)

**俯视图**
![俯视图](https://s2.loli.net/2025/08/11/6KFafh1QgMui9XW.png)

**左视图**
![左视图](https://s2.loli.net/2025/08/11/1yQksXDVOMgJ8Kf.png)

**舵机运动方向与角度和脉冲值的关系**

| 电机编号 | 方向   | 负值（角度/脉冲） | 中位值（角度/脉冲） | 正值（角度/脉冲） | 方向   | 观察视角 | 备注     |
| -------- | ------ | ----------------- | ------------------- | ----------------- | ------ | -------- | -------- |
| ID-1     | 顺时针 | （-120°/0）       | （0°/500）          | (120°/1000)       | 逆时针 | 俯视     |          |
| ID-2     | 逆时针 | （-180°/1000）    | （-90°/500）        | (0°/0）           | 顺时针 | 正视     |          |
| ID-3     | 逆时针 | （-120°/0）       | (0°/500)            | (120°/1000)       | 顺时针 | 正视     |          |
| ID-4     | 逆时针 | （-200°/1000）    | (-90°/500)          | (20°/0)           | 顺时针 | 正视     |          |
| ID-5     | 逆时针 | （-120°/0）       | （0°/500）          | （120°/1000）     | 顺时针 | 左视     |          |
| ID-6     | 打开   | （0°/0）          | （45°/500）         | （90°/1000）      | 关闭   | 俯视     | 末端夹爪 |

## 功能特性

- ✅ **完整的写指令支持** (14/14)
  - 舵机位置和电机模式控制
  - 角度和时间控制运动
  - 电压和温度限制设置
  - LED 控制和错误处理
  - 紧急停止功能

- ✅ **读指令支持** (14/14)
  - 舵机状态读取
  - 位置、温度、电压监控
  - 配置参数读取

- 🛠️ **工具函数**
  - 校验和计算
  - 字节操作工具
  - 协议解析工具
  - 角度和脉冲转换

## 安装

### 环境要求

- Python >= 3.8
- 支持串口通信的硬件设备

### 安装依赖

```bash
# 使用 PDM 安装依赖
pdm install

# 创建虚拟环境后，再使用 pip 去创建虚拟环境，涉及到较多依赖
# 或者使用 pip
pip install -r requirements.txt
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

### 状态监控

```python
# 启用数据接收功能（读指令需要）
controller.enable_reception(True)

# 读取关节位置
position = controller.get_joint_position(1)
print(f"关节1位置: {position}")

# 读取关节温度
temp = controller.get_joint_temp(1)
print(f"关节1温度: {temp}")

# 读取关节电压
voltage = controller.get_joint_input_voltage(1)
print(f"关节1电压: {voltage}")

# 读取关节模式和速度
mode_speed = controller.get_joint_mode_and_speed(1)
print(f"关节1模式和速度: {mode_speed}")
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

##### 状态读取

- `get_joint_position(joint_id)`: 获取关节当前位置
- `get_joint_temp(joint_id)`: 获取关节温度
- `get_joint_input_voltage(joint_id)`: 获取关节输入电压
- `get_joint_mode_and_speed(joint_id)`: 获取关节模式和速度
- `get_joint_load_or_unload(joint_id)`: 获取关节负载状态

##### 参数读取

- `get_joint_move_and_time(joint_id)`: 获取最后一次角度参数和时间
- `get_joint_move_and_wait_time(joint_id)`: 获取最后一次角度参数和延迟启动时间
- `get_joint_angle_offset(joint_id)`: 获取角度偏移量
- `get_joint_angle_limit(joint_id)`: 获取角度限制
- `get_joint_vin_limit(joint_id)`: 获取电压限制
- `get_joint_temp_max_limit(joint_id)`: 获取温度限制

##### 配置读取

- `get_joint_id(joint_id)`: 获取舵机ID
- `get_joint_led_ctrl(joint_id)`: 获取LED控制状态
- `get_joint_led_error(joint_id)`: 获取LED错误配置

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
- ✅ 读指令接口：100% 完成 (14/14)
- ✅ 工具函数：完成
- ✅ 协议解析：完成

## 文档

- 🔧 [API 参考文档](README.md#api-文档)
- 💡 [示例代码](examples/)

## 贡献

欢迎提交 Issue 和 Pull Request！

## 许可证

MIT License