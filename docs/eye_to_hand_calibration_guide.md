# Armpi-FPV 眼在手外标定指南

## 概述

眼在手外（Eye-to-Hand）标定是机器人视觉系统中的另一种重要配置，相机固定在外部位置，标定板安装在机器人末端。本指南将帮助您完成 Armpi-FPV 机器人的眼在手外标定流程。

## 系统架构

### 眼在手外配置

```
固定相机坐标系 (Camera - Fixed)
    ↓ (眼在手外变换 - 待标定)
基座坐标系 (Base) 
    ↓ (机器人正解)
末端坐标系 (End-Effector)
    ↓ (标定板安装变换 - 已知)
标定板坐标系 (Calibration Board)
```

**眼在手外标定的目标**：求解固定相机坐标系到机械臂基座坐标系的变换矩阵。

### 与眼在手上标定的区别

| 特性 | 眼在手上 (Eye-in-Hand) | 眼在手外 (Eye-to-Hand) |
|------|----------------------|----------------------|
| 相机位置 | 固定在机械臂末端 | 固定在外部位置 |
| 标定板位置 | 固定在外部位置 | 安装在机械臂末端 |
| 相机运动 | 随机械臂运动 | 不运动 |
| 标定板运动 | 不运动 | 随机械臂运动 |
| 求解目标 | 末端到相机的变换 | 基座到相机的变换 |
| 数学模型 | AX = XB | AX = YB |

## 硬件要求

### 必需设备
- Armpi-FPV 六轴机械臂
- USB 相机（固定在外部位置）
- 标定板（安装在机械臂末端）
- 相机支架或固定装置
- 计算机（Linux 系统，支持 OpenCV）

### 标定板规格
- **推荐尺寸**: 9×6 棋盘格
- **方格大小**: 25mm × 25mm
- **材质**: 平整的硬质纸板或塑料板
- **打印质量**: 高精度打印，边缘清晰
- **安装要求**: 牢固安装在机械臂末端，无相对运动

### 相机安装要求
- 相机固定安装在外部位置（桌面支架、天花板等）
- 确保相机位置稳定，不会发生移动
- 相机视野应能覆盖机械臂工作区域
- 推荐分辨率: 640×480 或更高
- 相机到工作区域距离适中，确保标定板清晰可见

### 安装示意图

```
    [固定相机]
        ↓ 观测
    工作区域
        ↑
    [机械臂] ← 末端安装标定板
```

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

2. **相机安装**
   - 将相机固定在稳定的外部位置
   - 调整相机角度，确保能看到机械臂工作区域
   - 测试相机视野，确保标定板在末端时可见

3. **标定板安装**
   - 将标定板牢固安装在机械臂末端
   - 确保标定板与末端之间无相对运动
   - 记录标定板相对于末端的安装位置

4. **工作空间布置**
   - 清理机械臂工作区域
   - 确保良好的照明条件
   - 避免背景干扰

### 第二步：相机标定

眼在手外标定同样需要先获取相机的内参矩阵和畸变系数。

```python
from armpi_common.robot_arm_controller import RobotArmController

# 连接机械臂
controller = RobotArmController(device="/dev/ttyUSB0")
controller.enable_reception(True)

# 执行相机标定
camera_result = controller.perform_camera_calibration(
    camera_source=2,        # 相机设备ID
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
- 手动移动标定板到不同位置和角度（不要移动相机！）
- 确保标定板在所有图像中完整可见
- 标定板应覆盖相机视野的不同区域
- 包含不同深度的图像以获得更好的标定精度

### 第三步：眼在手外标定

眼在手外标定通过控制机械臂移动到不同位姿来采集数据。

```python
import numpy as np

# 使用相机标定结果
camera_matrix = np.array(camera_result['camera_matrix'])
distortion_coeffs = np.array(camera_result['distortion_coeffs'])

# 初始化眼在手外标定系统
init_result = controller.initialize_eye_to_hand_calibration(
    camera_matrix=camera_matrix,
    distortion_coeffs=distortion_coeffs,
    board_size=(9, 6),
    square_size=0.025,
    camera_source=2,
    camera_pose=[0.0, -0.3, 0.4, 0.0, 0.0, 0.0],  # 相机位置（可选）
    save_directory="./eye_to_hand_calibration"
)

# 执行眼在手外标定
eye_to_hand_result = controller.perform_eye_to_hand_calibration(
    num_poses=15,                    # 采集位姿数量
    calibration_method='tsai',       # 标定算法
    workspace_center=[0.15, 0.0, 0.20],  # 工作空间中心
    workspace_radius=0.05,           # 工作空间半径
    show_preview=True                # 显示检测预览
)

if eye_to_hand_result['status']:
    print("眼在手外标定成功")
    print(f"标定误差: {eye_to_hand_result['calibration_error']:.6f}")
    print(f"标定文件: {eye_to_hand_result['calibration_file']}")
else:
    print(f"眼在手外标定失败: {eye_to_hand_result['info']}")
```

**重要提醒：**
- 标定过程中相机必须保持固定不动
- 机器人会自动移动到不同位姿采集数据
- 确保在所有位姿下相机都能看到完整的标定板
- 如果某个位姿检测失败，系统会自动重试

### 第四步：标定验证

验证标定精度以确保标定质量。

```python
# 执行标定验证
validation_result = controller.validate_eye_to_hand_calibration(
    num_test_poses=5,  # 测试位姿数量
    camera_source=2
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

## 数学原理

### 眼在手外标定数学模型

眼在手外标定需要求解方程：**AX = YB**

其中：
- **A**: 机械臂基座到末端的变换矩阵序列
- **X**: 机械臂基座到相机的变换矩阵（待求解）
- **Y**: 机械臂末端到标定板的变换矩阵（已知，通常为固定安装）
- **B**: 相机到标定板的变换矩阵序列

### 求解过程

1. **数据采集**：在不同机器人位姿下采集相机图像
2. **位姿估计**：通过PnP算法估计标定板相对于相机的位姿
3. **方程变换**：将 AX = YB 转换为标准的 AX = XB 形式
4. **最优化求解**：使用最小二乘法或其他优化算法求解

### 变换关系

```
相机坐标系中的点 P_c
    ↓ (X: base -> camera)
基座坐标系中的点 P_b = X^(-1) * P_c

基座坐标系中的点 P_b
    ↓ (A: base -> end-effector)
末端坐标系中的点 P_e = A^(-1) * P_b

末端坐标系中的点 P_e
    ↓ (Y: end-effector -> board)
标定板坐标系中的点 P_board = Y^(-1) * P_e
```

## 使用标定结果

### 加载标定文件

```python
# 加载已保存的眼在手外标定
load_result = controller.load_eye_to_hand_calibration(
    "./eye_to_hand_calibration/eye_to_hand_calibration.json"
)

if load_result['status']:
    print("标定加载成功")
else:
    print(f"标定加载失败: {load_result['info']}")
```

### 获取相机位姿

```python
# 获取固定相机在基座坐标系中的位姿
camera_pose_result = controller.get_fixed_camera_pose()

if camera_pose_result['status']:
    camera_pose = camera_pose_result['camera_pose']  # [x, y, z, rx, ry, rz]
    camera_matrix = camera_pose_result['camera_transform_matrix']  # 4x4变换矩阵
    
    print(f"相机位置: {camera_pose[:3]}")
    print(f"相机姿态: {camera_pose[3:]}")
else:
    print(f"计算失败: {camera_pose_result['info']}")
```

### 计算标定板位姿

```python
# 根据机器人位姿计算标定板位姿
board_pose_result = controller.get_board_pose_from_robot_pose()

if board_pose_result['status']:
    board_pose = board_pose_result['board_pose']  # [x, y, z, rx, ry, rz]
    board_matrix = board_pose_result['board_transform_matrix']  # 4x4变换矩阵
    
    print(f"标定板位置: {board_pose[:3]}")
    print(f"标定板姿态: {board_pose[3:]}")
else:
    print(f"计算失败: {board_pose_result['info']}")
```

## 应用场景

### 1. 视觉引导抓取

```python
# 眼在手外配置的视觉引导抓取流程

# 1. 获取相机位姿
camera_pose = controller.get_fixed_camera_pose()

# 2. 在相机图像中检测目标物体
image = capture_image_from_fixed_camera()
target_pixel_coords = detect_object_in_image(image)

# 3. 将像素坐标转换为相机坐标系下的3D坐标
target_camera_coords = pixel_to_camera_coords(target_pixel_coords, depth)

# 4. 转换到基座坐标系
camera_to_base_transform = np.array(camera_pose['camera_transform_matrix'])
target_camera_homo = np.append(target_camera_coords, 1.0)
target_base_homo = camera_to_base_transform @ target_camera_homo
target_base_coords = target_base_homo[:3]

# 5. 规划机器人运动
controller.set_joint_move_with_coordinate(target_base_coords)
```

### 2. 物体跟踪

```python
# 眼在手外配置的物体跟踪
def track_object_with_fixed_camera():
    camera_pose = controller.get_fixed_camera_pose()
    camera_to_base = np.array(camera_pose['camera_transform_matrix'])
    
    while tracking:
        # 从固定相机获取图像
        image = capture_image()
        
        # 检测物体位置
        object_pixel = detect_object(image)
        
        # 转换到基座坐标系
        object_camera_3d = pixel_to_3d(object_pixel)
        object_base_3d = transform_point(object_camera_3d, camera_to_base)
        
        # 控制机器人跟踪
        controller.move_to_target(object_base_3d)
```

### 3. 质量检测

```python
# 眼在手外配置的产品质量检测
def quality_inspection():
    # 移动机器人到检测位置
    inspection_poses = generate_inspection_poses()
    
    for pose in inspection_poses:
        # 移动到检测位姿
        controller.set_joint_move_with_coordinate(pose)
        
        # 从固定相机获取图像
        image = capture_image()
        
        # 执行质量检测
        defects = detect_defects(image)
        
        # 记录检测结果
        record_inspection_result(pose, defects)
```

## 故障排除

### 常见问题及解决方案

#### 1. 相机检测不到标定板
**可能原因:**
- 标定板超出相机视野
- 标定板角度过大
- 照明条件不佳
- 标定板污损或模糊

**解决方案:**
- 调整机器人工作空间参数
- 修改相机位置或角度
- 改善照明条件
- 清洁或更换标定板

#### 2. 机器人位姿不可达
**解决方案:**
- 减小工作空间半径
- 调整工作空间中心位置
- 检查机器人关节限制
- 优化相机安装位置

#### 3. 标定精度不佳
**可能原因:**
- 标定样本数量不足
- 位姿分布不均匀
- 相机标定精度低
- 标定板安装不牢固

**改进方法:**
- 增加标定样本数量
- 优化位姿分布
- 重新进行相机标定
- 检查标定板安装

#### 4. 系统稳定性问题
**解决方案:**
- 确保相机固定牢靠
- 检查标定板安装稳定性
- 避免外部振动干扰
- 使用高质量的硬件设备

### 调试技巧

#### 1. 启用详细日志
```python
from armpi_common._log import set_stream_level
set_stream_level("DEBUG")
```

#### 2. 可视化标定板检测
```python
# 在数据采集时启用预览
eye_to_hand_result = controller.perform_eye_to_hand_calibration(
    show_preview=True  # 显示检测预览
)
```

#### 3. 单步验证
```python
# 逐步验证每个环节
# 1. 验证相机连接
cap = cv2.VideoCapture(camera_source)
ret, frame = cap.read()

# 2. 验证标定板检测
success, corners = calibrator.detect_chessboard(frame, show_corners=True)

# 3. 验证机器人运动
controller.set_joint_move_with_coordinate(test_pose)
```

## 精度评估标准

### 眼在手外标定精度评估

- **优秀标定** (生产级): 平移误差 < 3mm, 旋转误差 < 3°
- **良好标定** (实验级): 平移误差 < 6mm, 旋转误差 < 6°
- **可接受标定** (演示级): 平移误差 < 12mm, 旋转误差 < 12°

*注：眼在手外标定的精度要求通常比眼在手上标定稍低，因为固定相机的距离较远*

## 最佳实践

### 1. 硬件配置
- 使用高分辨率相机
- 确保充足稳定的照明
- 选择高质量的标定板
- 使用稳定的相机支架

### 2. 软件配置
- 适当调整工作空间参数
- 选择合适的标定算法
- 增加标定样本数量
- 定期验证标定精度

### 3. 维护建议
- 定期检查相机位置
- 清洁标定板表面
- 验证标定板安装
- 重新标定（如有必要）

## 与眼在手上标定的选择

### 眼在手外的优势
- 相机视野固定，工作范围大
- 相机不受机器人运动影响
- 适合大范围监控和检测
- 机器人结构简单

### 眼在手外的劣势
- 相机距离目标较远，精度相对较低
- 存在遮挡问题
- 相机安装位置要求较高
- 视角固定，灵活性有限

### 选择建议

**选择眼在手外的情况：**
- 需要大范围工作区域监控
- 对精度要求不是特别高
- 需要固定视角的应用
- 机器人负载能力有限

**选择眼在手上的情况：**
- 需要高精度操作
- 需要多角度视觉检测
- 工作空间相对较小
- 对灵活性要求较高

## 完整示例

运行完整的眼在手外标定示例：

```bash
# 运行完整的标定流程
python examples/eye_to_hand_calibration_example.py --mode full

# 运行快速演示（需要已有标定文件）
python examples/eye_to_hand_calibration_example.py --mode quick
```

## 参考文献

1. Shiu, Y. C., & Ahmad, S. (1989). Calibration of wrist-mounted robotic sensors by solving homogeneous transform equations of the form AX= XB.
2. Tsai, R. Y., & Lenz, R. K. (1989). A new technique for fully autonomous and efficient 3D robotics hand/eye calibration.
3. Daniilidis, K. (1999). Hand-eye calibration using dual quaternions.
4. Horaud, R., & Dornaika, F. (1995). Hand-eye calibration.

## 技术支持

如果您在使用过程中遇到问题，请参考：

1. **API 文档**: 查看各函数的详细说明
2. **示例代码**: 参考 `examples/` 目录中的示例
3. **调试日志**: 启用 DEBUG 级别日志查看详细信息
4. **社区支持**: 在项目仓库提交 Issue

---

**注意**: 眼在手外标定相比眼在手上标定在硬件安装和精度要求上有所不同，请根据实际应用场景选择合适的标定方式。标定质量很大程度上取决于硬件安装的稳定性和精度。
