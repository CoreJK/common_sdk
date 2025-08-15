# Armpi-FPV 眼在手上标定指南

## 概述

眼在手上（Eye-in-Hand）标定是机器人视觉系统中的重要环节，用于确定相机与机器人末端执行器之间的精确空间变换关系。本指南将帮助您完成 Armpi-FPV 机器人的完整手眼标定流程。

## 系统架构

```
基座坐标系 (Base) 
    ↓ (机器人正解)
末端坐标系 (End-Effector)
    ↓ (手眼变换 - 待标定)
相机坐标系 (Camera)
    ↓ (相机标定已知)
标定板坐标系 (Calibration Board)
```

手眼标定的目标是求解末端坐标系到相机坐标系的变换矩阵。

## 硬件要求

### 必需设备
- Armpi-FPV 六轴机械臂
- USB 相机（安装在机械臂末端）
- 标定板（推荐规格见下文）
- 计算机（Linux 系统，支持 OpenCV）

### 标定板规格
- **推荐尺寸**: 9×6 棋盘格
- **方格大小**: 25mm × 25mm
- **材质**: 平整的硬质纸板或塑料板
- **打印质量**: 高精度打印，边缘清晰

### 相机安装要求
- 相机固定安装在机械臂末端
- 确保相机与末端执行器之间无相对运动
- 相机视野应能覆盖工作区域
- 推荐分辨率: 640×480 或更高

## 软件依赖

确保已安装以下 Python 包：

```bash
pip install opencv-python numpy scipy spatialmath-python roboticstoolbox-python
```

## 标定流程

### 第一步：环境准备

1. **硬件连接**
   ```bash
   # 检查机械臂连接
   ls /dev/ttyUSB*
   
   # 检查相机连接
   ls /dev/video*
   ```

2. **权限设置**
   ```bash
   # 添加用户到 dialout 组
   sudo usermod -a -G dialout $USER
   
   # 重新登录生效
   ```

3. **工作空间布置**
   - 将标定板放置在机器人工作空间内
   - 确保标定板表面平整
   - 标定板应在相机视野范围内
   - 准备充足的照明

### 第二步：相机标定

相机标定用于获取相机的内参矩阵和畸变系数。

```python
from armpi_common.robot_arm_controller import RobotArmController

# 连接机械臂
controller = RobotArmController(device="/dev/ttyUSB0")
controller.enable_reception(True)

# 执行相机标定
camera_result = controller.perform_camera_calibration(
    camera_source=0,        # 相机设备ID
    num_images=20,          # 采集图像数量
    board_size=(9, 6),      # 标定板尺寸
    square_size=0.025,      # 方格大小(米)
    save_directory="./camera_calibration"
)

if camera_result['status']:
    print("相机标定成功")
    print(f"重投影误差: {camera_result['calibration_error']:.4f} 像素")
else:
    print(f"相机标定失败: {camera_result['info']}")
```

**操作要点：**
- 在采集过程中需要手动变换标定板的位置和角度
- 确保标定板在所有图像中完整可见
- 标定板应覆盖相机视野的不同区域
- 包含不同深度的图像以获得更好的标定精度

### 第三步：手眼标定

手眼标定通过采集多个机器人位姿和对应的相机图像来求解变换关系。

```python
import numpy as np

# 使用相机标定结果
camera_matrix = np.array(camera_result['camera_matrix'])
distortion_coeffs = np.array(camera_result['distortion_coeffs'])

# 初始化手眼标定系统
init_result = controller.initialize_hand_eye_calibration(
    camera_matrix=camera_matrix,
    distortion_coeffs=distortion_coeffs,
    board_size=(9, 6),
    square_size=0.025,
    camera_source=0,
    save_directory="./hand_eye_calibration"
)

# 执行手眼标定
hand_eye_result = controller.perform_hand_eye_calibration(
    num_poses=15,                    # 采集位姿数量
    calibration_method='tsai',       # 标定算法
    workspace_center=[0.15, 0.0, 0.20],  # 工作空间中心
    workspace_radius=0.05,           # 工作空间半径
    show_preview=True                # 显示检测预览
)

if hand_eye_result['status']:
    print("手眼标定成功")
    print(f"标定误差: {hand_eye_result['calibration_error']:.6f}")
    print(f"标定文件: {hand_eye_result['calibration_file']}")
else:
    print(f"手眼标定失败: {hand_eye_result['info']}")
```

**重要提醒：**
- 标定过程中标定板必须保持固定不动
- 机器人会自动移动到不同位姿采集数据
- 确保在所有位姿下相机都能看到完整的标定板
- 如果某个位姿检测失败，系统会自动重试

### 第四步：标定验证

验证标定精度以确保标定质量。

```python
# 执行标定验证
validation_result = controller.validate_hand_eye_calibration(
    num_test_poses=5,  # 测试位姿数量
    camera_source=0
)

if validation_result['status']:
    print(f"验证成功率: {validation_result['success_rate']:.1%}")
    print(f"平均平移误差: {validation_result['mean_translation_error_mm']:.2f} mm")
    print(f"平均旋转误差: {validation_result['mean_rotation_error_deg']:.2f} °")
    
    # 评估标定质量
    trans_error = validation_result['mean_translation_error_mm']
    rot_error = validation_result['mean_rotation_error_deg']
    
    if trans_error < 2.0 and rot_error < 2.0:
        print("标定质量: 优秀")
    elif trans_error < 5.0 and rot_error < 5.0:
        print("标定质量: 良好")
    else:
        print("标定质量: 需要改进")
```

## 标定算法说明

本系统支持多种手眼标定算法：

### 1. Tsai 算法 (默认推荐)
- **特点**: 计算稳定，精度较高
- **适用**: 大多数应用场景
- **参数**: `calibration_method='tsai'`

### 2. Park 算法
- **特点**: 对噪声鲁棒性好
- **适用**: 传感器噪声较大的情况
- **参数**: `calibration_method='park'`

### 3. Horaud 算法
- **特点**: 基于四元数，计算效率高
- **适用**: 实时性要求高的应用
- **参数**: `calibration_method='horaud'`

### 4. Andreff 算法
- **特点**: 同时优化旋转和平移
- **适用**: 高精度要求的应用
- **参数**: `calibration_method='andreff'`

### 5. Daniilidis 算法
- **特点**: 基于对偶四元数
- **适用**: 理论研究和对比分析
- **参数**: `calibration_method='daniilidis'`

## 使用标定结果

### 加载标定文件

```python
# 加载已保存的手眼标定
load_result = controller.load_hand_eye_calibration(
    "./hand_eye_calibration/hand_eye_calibration.json"
)

if load_result['status']:
    print("标定加载成功")
else:
    print(f"标定加载失败: {load_result['info']}")
```

### 计算相机位姿

```python
# 获取当前机器人位姿对应的相机位姿
camera_pose_result = controller.get_camera_pose_in_base()

if camera_pose_result['status']:
    camera_pose = camera_pose_result['camera_pose']  # [x, y, z, rx, ry, rz]
    camera_matrix = camera_pose_result['camera_transform_matrix']  # 4x4变换矩阵
    
    print(f"相机位置: {camera_pose[:3]}")
    print(f"相机姿态: {camera_pose[3:]}")
else:
    print(f"计算失败: {camera_pose_result['info']}")
```

### 指定机器人位姿计算相机位姿

```python
# 为指定的机器人位姿计算相机位姿
target_robot_pose = [0.15, 0.0, 0.20, 0.0, 0.0, -3.14159]

camera_pose_result = controller.get_camera_pose_in_base(
    robot_pose=target_robot_pose
)
```

## 故障排除

### 常见问题及解决方案

#### 1. 相机无法打开
```python
# 检查相机连接
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print("相机连接正常")
    cap.release()
else:
    print("相机连接失败，请检查设备")
```

**解决方案:**
- 检查USB连接
- 确认相机驱动安装
- 尝试不同的设备ID (0, 1, 2...)

#### 2. 标定板检测失败
**可能原因:**
- 光照不足或过强
- 标定板模糊或变形
- 标定板尺寸设置错误
- 相机焦距不合适

**解决方案:**
- 调整照明条件
- 使用高质量标定板
- 确认标定板参数设置
- 调整相机到标定板的距离

#### 3. 机器人位姿不可达
**解决方案:**
- 调整工作空间参数
- 减小工作空间半径
- 检查机器人关节限制

#### 4. 标定精度不佳
**可能原因:**
- 标定样本数量不足
- 位姿分布不均匀
- 标定板检测精度低
- 机器人重复定位精度差

**改进方法:**
- 增加标定样本数量
- 优化位姿分布
- 提高标定板质量
- 检查机器人机械精度

### 调试技巧

#### 1. 启用详细日志
```python
from armpi_common._log import set_stream_level
set_stream_level("DEBUG")
```

#### 2. 保存中间结果
```python
# 手动保存标定数据
calibration_info = controller.get_hand_eye_calibration_info()
print("标定信息:", calibration_info)
```

#### 3. 分步验证
```python
# 单独验证相机标定
camera_info = camera_calibrator.get_calibration_info()
validation = camera_calibrator.validate_calibration()
print("相机标定验证:", validation)
```

## 精度评估标准

### 优秀标定 (生产级)
- 平移误差 < 2.0 mm
- 旋转误差 < 2.0°
- 重投影误差 < 0.5 像素

### 良好标定 (实验级)
- 平移误差 < 5.0 mm
- 旋转误差 < 5.0°
- 重投影误差 < 1.0 像素

### 可接受标定 (演示级)
- 平移误差 < 10.0 mm
- 旋转误差 < 10.0°
- 重投影误差 < 2.0 像素

## 应用示例

### 视觉引导抓取

```python
# 1. 获取当前相机位姿
camera_pose = controller.get_camera_pose_in_base()

# 2. 在相机图像中检测目标物体
# (这里需要您自己的物体检测算法)
target_pixel_coords = detect_object_in_image(image)

# 3. 将像素坐标转换为相机坐标系下的3D坐标
# (需要深度信息或平面假设)
target_camera_coords = pixel_to_camera_coords(target_pixel_coords, depth)

# 4. 转换到基座坐标系
camera_transform = np.array(camera_pose['camera_transform_matrix'])
target_base_coords = transform_point(target_camera_coords, camera_transform)

# 5. 规划机器人运动
controller.set_joint_move_with_coordinate(target_base_coords)
```

### 视觉伺服

```python
# 实时视觉反馈控制
while not target_reached:
    # 获取当前图像
    image = capture_image()
    
    # 检测目标在图像中的位置
    target_pixel = detect_target(image)
    
    # 计算误差
    error = target_pixel - image_center
    
    # 计算机器人运动增量
    robot_delta = visual_servo_control(error, camera_pose)
    
    # 执行运动
    controller.move_relative(robot_delta)
```

## 文件结构

标定完成后，将生成以下文件：

```
项目目录/
├── camera_calibration/
│   ├── camera_calibration.json      # 相机内参文件
│   ├── calibration_image_001.jpg    # 标定图像
│   └── ...
├── hand_eye_calibration/
│   ├── hand_eye_calibration.json    # 手眼标定结果
│   ├── collection_metadata.json     # 采集元数据
│   ├── sample_001.jpg              # 标定样本图像
│   └── ...
└── examples/
    └── eye_in_hand_calibration_example.py  # 完整示例
```

## 参考文献

1. Tsai, R. Y., & Lenz, R. K. (1989). A new technique for fully autonomous and efficient 3D robotics hand/eye calibration.
2. Park, F. C., & Martin, B. J. (1994). Robot sensor calibration: solving AX=XB on the Euclidean group.
3. Horaud, R., & Dornaika, F. (1995). Hand-eye calibration.
4. Andreff, N., Horaud, R., & Espiau, B. (2001). On-line hand-eye calibration.
5. Daniilidis, K. (1999). Hand-eye calibration using dual quaternions.

## 技术支持

如果您在使用过程中遇到问题，请参考：

1. **API 文档**: 查看各函数的详细说明
2. **示例代码**: 参考 `examples/` 目录中的示例
3. **调试日志**: 启用 DEBUG 级别日志查看详细信息
4. **社区支持**: 在项目仓库提交 Issue

---

**注意**: 本指南基于 Armpi-FPV 机器人系统，其他机器人系统可能需要相应调整。标定精度受多种因素影响，请根据实际应用需求选择合适的标定参数和质量标准。
