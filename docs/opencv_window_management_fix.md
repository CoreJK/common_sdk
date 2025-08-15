# OpenCV窗口管理优化

## 问题描述

在手眼标定过程中，"Captured Sample"窗口会遮挡主预览窗口"Hand-Eye Calibration Preview"，影响用户操作和标定板检测的可视化反馈。

## 解决方案

### 1. 窗口布局管理函数

新增 `setup_opencv_windows()` 函数，统一管理窗口布局：

```python
def setup_opencv_windows():
    """设置OpenCV窗口布局，避免窗口重叠遮挡"""
    cv2.namedWindow("Hand-Eye Calibration Preview", cv2.WINDOW_NORMAL)
    cv2.moveWindow("Hand-Eye Calibration Preview", 50, 50)
    cv2.resizeWindow("Hand-Eye Calibration Preview", 600, 450)
```

### 2. 窗口位置优化

**主预览窗口:**
- 位置: 屏幕左上角 (50, 50)
- 尺寸: 600×450 像素
- 固定显示，持续提供实时反馈

**采集确认窗口:**
- 位置: 屏幕右侧 (700, 50)
- 尺寸: 400×300 像素
- 临时显示1.5秒后自动关闭
- 每次采集使用不同窗口名避免累积

### 3. 改进特性

#### 窗口标识优化
- 采集窗口使用动态命名: `"Captured Sample {sample_count}"`
- 避免多个窗口重叠累积

#### 视觉反馈增强
```python
cv2.putText(preview_frame, f"Sample {sample_count} Captured!", 
           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
cv2.putText(preview_frame, "Position confirmed - pose saved", 
           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
```

#### 用户提示
- 在主窗口显示操作提示: "Press 'q' to exit preview"
- 启动时显示窗口布局说明

### 4. 应用场景

#### 手动标定模式
- 主窗口: 实时显示相机画面和标定板检测状态
- 确认窗口: 显示成功采集的位姿样本

#### 自动标定模式
- 统一的窗口布局管理
- 减少窗口遮挡对自动流程的影响

## 技术细节

### 窗口属性设置
```python
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # 可调整大小
cv2.moveWindow(window_name, x, y)                # 设置位置
cv2.resizeWindow(window_name, width, height)     # 设置尺寸
```

### 窗口生命周期管理
```python
cv2.imshow(window_name, image)                   # 显示图像
cv2.waitKey(display_time)                        # 控制显示时间
cv2.destroyWindow(window_name)                   # 销毁特定窗口
```

## 使用建议

### 多显示器环境
- 可将确认窗口拖拽到第二显示器
- 调整窗口位置参数适应不同分辨率

### 单显示器优化
- 默认布局已优化为左右分布
- 如仍有遮挡，可手动拖拽调整

### 自定义设置
用户可修改 `setup_opencv_windows()` 中的参数：
```python
cv2.moveWindow("Hand-Eye Calibration Preview", x, y)  # 调整主窗口位置
cv2.moveWindow(window_name, x, y)                     # 调整确认窗口位置
```

## 兼容性

- 适用于所有OpenCV支持的平台
- 与现有标定流程完全兼容
- 不影响标定精度和功能

## 后续改进建议

1. **配置文件**: 支持用户自定义窗口布局配置
2. **自适应布局**: 根据屏幕分辨率自动调整窗口位置
3. **窗口记忆**: 保存用户偏好的窗口位置
4. **多窗口模式**: 支持同时显示多个角度的标定反馈
