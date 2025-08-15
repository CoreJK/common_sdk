# -*- coding: utf-8 -*-
# camera_calibration.py - 相机标定模块

import cv2
import numpy as np
import os
import json
from typing import List, Tuple, Dict, Optional
from datetime import datetime
import glob

from armpi_common._log import logger


class CameraCalibration:
    """
    相机标定类
    
    用于获取相机的内参矩阵、畸变系数等参数，这些参数是手眼标定的前提条件
    """
    
    def __init__(self, 
                 board_size: Tuple[int, int] = (9, 6),
                 square_size: float = 0.025,
                 save_directory: str = "./camera_calibration"):
        """
        初始化相机标定器
        
        Args:
            board_size: 标定板角点数量 (列数, 行数)
            square_size: 标定板方格尺寸 (米)
            save_directory: 保存目录
        """
        self.board_size = board_size
        self.square_size = square_size
        self.save_directory = save_directory
        
        # 标定数据
        self.object_points = []  # 3D点
        self.image_points = []   # 2D点
        self.calibration_images = []  # 标定图像
        
        # 标定结果
        self.camera_matrix = None      # 相机内参矩阵
        self.distortion_coeffs = None  # 畸变系数
        self.rvecs = None             # 旋转向量
        self.tvecs = None             # 平移向量
        self.calibration_error = None # 重投影误差
        
        # 图像尺寸
        self.image_size = None
        
        # 创建保存目录
        os.makedirs(save_directory, exist_ok=True)
        
        # 生成标定板的世界坐标点
        self._generate_object_points()
        
        logger.info(f"相机标定器初始化完成，标定板尺寸: {board_size}, 方格大小: {square_size}m")
    
    def _generate_object_points(self):
        """生成标定板在其自身坐标系下的3D点坐标"""
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 
                              0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.template_object_points = objp
    
    def detect_chessboard(self, image: np.ndarray, 
                         show_corners: bool = False) -> Tuple[bool, np.ndarray]:
        """
        检测标定板角点
        
        Args:
            image: 输入图像
            show_corners: 是否显示检测到的角点
            
        Returns:
            (success, corners): 是否成功检测, 角点坐标
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        
        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, 
                                                cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                cv2.CALIB_CB_FAST_CHECK +
                                                cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if ret:
            # 亚像素精度优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            if show_corners:
                # 绘制角点
                vis_image = image.copy()
                cv2.drawChessboardCorners(vis_image, self.board_size, corners, ret)
                cv2.imshow('Camera Calibration - Chessboard Corners', vis_image)
                cv2.waitKey(1000)
                cv2.destroyAllWindows()
            
            logger.debug("成功检测到相机标定板角点")
            return True, corners
        else:
            logger.debug("未检测到相机标定板角点")
            return False, None
    
    def add_calibration_image(self, image: np.ndarray, 
                            save_image: bool = True) -> bool:
        """
        添加标定图像
        
        Args:
            image: 输入图像
            save_image: 是否保存图像到文件
            
        Returns:
            是否成功添加
        """
        # 检测角点
        success, corners = self.detect_chessboard(image)
        if not success:
            logger.warning("图像中未检测到标定板，跳过此图像")
            return False
        
        # 记录图像尺寸
        if self.image_size is None:
            self.image_size = (image.shape[1], image.shape[0])  # (width, height)
        elif self.image_size != (image.shape[1], image.shape[0]):
            logger.error("图像尺寸不一致，请使用相同分辨率的图像")
            return False
        
        # 添加到标定数据
        self.object_points.append(self.template_object_points)
        self.image_points.append(corners)
        self.calibration_images.append(image.copy())
        
        # 保存图像文件
        if save_image:
            image_filename = os.path.join(self.save_directory, 
                                        f"calibration_image_{len(self.calibration_images):03d}.jpg")
            cv2.imwrite(image_filename, image)
        
        logger.info(f"成功添加第 {len(self.calibration_images)} 张标定图像")
        return True
    
    def collect_calibration_images(self, 
                                 camera_source: int = 0,
                                 num_images: int = 20,
                                 capture_interval: float = 2.0,
                                 auto_capture: bool = False) -> int:
        """
        采集标定图像
        
        Args:
            camera_source: 相机设备ID
            num_images: 目标图像数量
            capture_interval: 自动采集间隔（秒）
            auto_capture: 是否自动采集
            
        Returns:
            成功采集的图像数量
        """
        logger.info(f"开始采集相机标定图像，目标数量: {num_images}")
        
        cap = cv2.VideoCapture(camera_source)
        if not cap.isOpened():
            logger.error(f"无法打开相机: {camera_source}")
            return 0
        
        # 设置相机参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        collected_count = 0
        last_capture_time = 0
        
        try:
            while collected_count < num_images:
                ret, frame = cap.read()
                if not ret:
                    logger.warning("无法读取相机帧")
                    continue
                
                # 检测角点并显示
                success, corners = self.detect_chessboard(frame)
                
                # 创建显示图像
                display_frame = frame.copy()
                
                if success:
                    cv2.drawChessboardCorners(display_frame, self.board_size, corners, True)
                    status_text = f"Detected! ({collected_count}/{num_images})"
                    color = (0, 255, 0)
                else:
                    status_text = f"No board ({collected_count}/{num_images})"
                    color = (0, 0, 255)
                
                cv2.putText(display_frame, status_text, (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                
                if auto_capture:
                    cv2.putText(display_frame, "Auto capture mode", (10, 70), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                else:
                    cv2.putText(display_frame, "Press SPACE to capture, ESC to exit", (10, 70), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                cv2.imshow('Camera Calibration - Image Collection', display_frame)
                
                # 处理按键
                key = cv2.waitKey(30) & 0xFF
                
                # 采集逻辑
                should_capture = False
                current_time = cv2.getTickCount() / cv2.getTickFrequency()
                
                if auto_capture:
                    if (success and 
                        current_time - last_capture_time > capture_interval):
                        should_capture = True
                        last_capture_time = current_time
                else:
                    if key == ord(' ') and success:  # 空格键采集
                        should_capture = True
                
                if should_capture:
                    if self.add_calibration_image(frame):
                        collected_count += 1
                        logger.info(f"采集进度: {collected_count}/{num_images}")
                
                # 退出条件
                if key == 27:  # ESC键退出
                    logger.info("用户取消采集")
                    break
                    
        finally:
            cap.release()
            cv2.destroyAllWindows()
        
        logger.info(f"图像采集完成，共采集 {collected_count} 张有效图像")
        return collected_count
    
    def calibrate_camera(self) -> bool:
        """
        执行相机标定
        
        Returns:
            是否标定成功
        """
        if len(self.object_points) < 10:
            logger.error(f"标定图像数量不足，至少需要10张，当前有{len(self.object_points)}张")
            return False
        
        if self.image_size is None:
            logger.error("图像尺寸未设置")
            return False
        
        logger.info(f"开始相机标定，使用 {len(self.object_points)} 张图像")
        
        try:
            # 执行标定
            ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.object_points, 
                self.image_points, 
                self.image_size,
                None, 
                None,
                flags=cv2.CALIB_RATIONAL_MODEL
            )
            
            # 保存结果
            self.camera_matrix = camera_matrix
            self.distortion_coeffs = distortion_coeffs
            self.rvecs = rvecs
            self.tvecs = tvecs
            self.calibration_error = ret
            
            # 计算重投影误差
            self._compute_reprojection_error()
            
            logger.info(f"相机标定成功完成，重投影误差: {self.calibration_error:.6f} 像素")
            return True
            
        except Exception as e:
            logger.error(f"相机标定失败: {e}")
            return False
    
    def _compute_reprojection_error(self):
        """计算重投影误差"""
        total_error = 0
        total_points = 0
        
        for i in range(len(self.object_points)):
            # 重投影
            projected_points, _ = cv2.projectPoints(
                self.object_points[i], 
                self.rvecs[i], 
                self.tvecs[i], 
                self.camera_matrix, 
                self.distortion_coeffs
            )
            
            # 计算误差
            error = cv2.norm(self.image_points[i], projected_points, cv2.NORM_L2)
            total_error += error * error
            total_points += len(self.object_points[i])
        
        mean_error = np.sqrt(total_error / total_points)
        self.calibration_error = mean_error
        
        logger.debug(f"重投影误差: {mean_error:.4f} 像素")
    
    def save_calibration(self, filename: str = None) -> bool:
        """
        保存标定结果
        
        Args:
            filename: 保存文件名，默认为 camera_calibration.json
            
        Returns:
            是否保存成功
        """
        if self.camera_matrix is None:
            logger.error("没有标定结果可保存")
            return False
        
        if filename is None:
            filename = os.path.join(self.save_directory, "camera_calibration.json")
        
        try:
            calibration_data = {
                'timestamp': datetime.now().isoformat(),
                'camera_matrix': self.camera_matrix.tolist(),
                'distortion_coeffs': self.distortion_coeffs.tolist(),
                'calibration_error': float(self.calibration_error),
                'image_size': self.image_size,
                'board_size': self.board_size,
                'square_size': float(self.square_size),
                'num_images': len(self.calibration_images)
            }
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"相机标定结果已保存到: {filename}")
            return True
            
        except Exception as e:
            logger.error(f"保存标定结果失败: {e}")
            return False
    
    def load_calibration(self, filename: str) -> bool:
        """
        加载标定结果
        
        Args:
            filename: 文件路径
            
        Returns:
            是否加载成功
        """
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                calibration_data = json.load(f)
            
            self.camera_matrix = np.array(calibration_data['camera_matrix'])
            self.distortion_coeffs = np.array(calibration_data['distortion_coeffs'])
            self.calibration_error = calibration_data['calibration_error']
            self.image_size = tuple(calibration_data['image_size'])
            self.board_size = tuple(calibration_data['board_size'])
            self.square_size = calibration_data['square_size']
            
            logger.info(f"相机标定结果加载成功，重投影误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            logger.error(f"加载标定结果失败: {e}")
            return False
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """
        图像去畸变
        
        Args:
            image: 输入图像
            
        Returns:
            去畸变后的图像
        """
        if self.camera_matrix is None or self.distortion_coeffs is None:
            logger.error("相机参数未标定，无法进行去畸变")
            return image
        
        return cv2.undistort(image, self.camera_matrix, self.distortion_coeffs)
    
    def get_optimal_camera_matrix(self, alpha: float = 1.0) -> np.ndarray:
        """
        获取优化的相机矩阵
        
        Args:
            alpha: 缩放参数，0=仅保留有效像素，1=保留所有像素
            
        Returns:
            优化的相机矩阵
        """
        if self.camera_matrix is None or self.image_size is None:
            logger.error("相机参数不完整")
            return None
        
        optimal_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, 
            self.distortion_coeffs, 
            self.image_size, 
            alpha, 
            self.image_size
        )
        
        return optimal_matrix
    
    def validate_calibration(self, test_images: List[np.ndarray] = None) -> Dict:
        """
        验证标定精度
        
        Args:
            test_images: 测试图像列表，为空则使用标定图像
            
        Returns:
            验证结果
        """
        if self.camera_matrix is None:
            logger.error("尚未完成相机标定")
            return {}
        
        if test_images is None:
            test_images = self.calibration_images
        
        if not test_images:
            logger.error("没有可用的测试图像")
            return {}
        
        errors = []
        detected_count = 0
        
        for i, image in enumerate(test_images):
            success, corners = self.detect_chessboard(image)
            if success:
                detected_count += 1
                
                # 估计位姿
                success_pnp, rvec, tvec = cv2.solvePnP(
                    self.template_object_points,
                    corners,
                    self.camera_matrix,
                    self.distortion_coeffs
                )
                
                if success_pnp:
                    # 重投影
                    projected_points, _ = cv2.projectPoints(
                        self.template_object_points,
                        rvec, tvec,
                        self.camera_matrix,
                        self.distortion_coeffs
                    )
                    
                    # 计算误差
                    error = cv2.norm(corners, projected_points, cv2.NORM_L2)
                    errors.append(error)
        
        if errors:
            validation_results = {
                'mean_error': np.mean(errors),
                'std_error': np.std(errors),
                'max_error': np.max(errors),
                'min_error': np.min(errors),
                'detection_rate': detected_count / len(test_images),
                'tested_images': len(test_images),
                'detected_images': detected_count
            }
        else:
            validation_results = {
                'error': '没有有效的测试结果',
                'detection_rate': 0,
                'tested_images': len(test_images),
                'detected_images': 0
            }
        
        logger.info(f"标定验证完成: "
                   f"检测率={validation_results.get('detection_rate', 0):.1%}, "
                   f"平均误差={validation_results.get('mean_error', 0):.3f}像素")
        
        return validation_results
    
    def clear_calibration_data(self):
        """清空标定数据"""
        self.object_points.clear()
        self.image_points.clear()
        self.calibration_images.clear()
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.calibration_error = None
        self.image_size = None
        logger.info("已清空相机标定数据")
    
    def get_calibration_info(self) -> Dict:
        """获取标定信息摘要"""
        info = {
            'num_images': len(self.calibration_images),
            'is_calibrated': self.camera_matrix is not None,
            'calibration_error': self.calibration_error,
            'image_size': self.image_size,
            'board_size': self.board_size,
            'square_size': self.square_size
        }
        
        if self.camera_matrix is not None:
            info['camera_matrix'] = self.camera_matrix.tolist()
            info['distortion_coeffs'] = self.distortion_coeffs.tolist()
            
            # 计算焦距和主点
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            info['focal_length'] = [fx, fy]
            info['principal_point'] = [cx, cy]
        
        return info


def quick_camera_calibration(camera_source: int = 0,
                           num_images: int = 20,
                           board_size: Tuple[int, int] = (9, 6),
                           square_size: float = 0.025,
                           save_directory: str = "./quick_camera_calibration") -> CameraCalibration:
    """
    快速相机标定流程
    
    Args:
        camera_source: 相机源
        num_images: 采集图像数量
        board_size: 标定板尺寸
        square_size: 方格大小
        save_directory: 保存目录
        
    Returns:
        标定完成的相机标定器
    """
    logger.info("开始快速相机标定流程")
    
    calibrator = CameraCalibration(board_size, square_size, save_directory)
    
    # 采集图像
    collected = calibrator.collect_calibration_images(camera_source, num_images, auto_capture=True)
    
    if collected < 10:
        logger.error("采集的有效图像数量不足")
        return calibrator
    
    # 执行标定
    success = calibrator.calibrate_camera()
    
    if success:
        # 保存结果
        calibrator.save_calibration()
        logger.info("快速相机标定完成")
    else:
        logger.error("相机标定失败")
    
    return calibrator


if __name__ == "__main__":
    # 示例用法
    
    # 快速标定
    calibrator = quick_camera_calibration(
        camera_source=0,
        num_images=15,
        board_size=(9, 6),
        square_size=0.025
    )
    
    # 显示标定信息
    info = calibrator.get_calibration_info()
    print("相机标定信息:")
    for key, value in info.items():
        print(f"  {key}: {value}")
