# -*- coding: utf-8 -*-
# hand_eye_calibration.py - 眼在手上标定模块

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from spatialmath import SE3
from spatialmath.base import tr2rpy, rpy2tr
import logging
from typing import List, Tuple, Dict, Optional, Union
import json
import time
from datetime import datetime

from armpi_common._log import logger

class HandEyeCalibration:
    """
    眼在手上（Eye-in-Hand）标定类
    
    该类实现了手眼标定的完整流程，解决 AX = XB 问题：
    - A: 机器人base到end-effector的变换序列
    - X: end-effector到camera的变换（待求解）
    - B: camera到标定板的变换序列
    
    眼在手上配置中，相机固定在机械臂末端，随末端一起运动
    """
    
    def __init__(self, 
                 camera_matrix: np.ndarray = None,
                 distortion_coeffs: np.ndarray = None,
                 board_size: Tuple[int, int] = (9, 6),
                 square_size: float = 0.025):
        """
        初始化手眼标定器
        
        Args:
            camera_matrix: 相机内参矩阵 (3x3)
            distortion_coeffs: 相机畸变系数
            board_size: 标定板角点数量 (列数, 行数)
            square_size: 标定板方格尺寸 (米)
        """
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        self.board_size = board_size
        self.square_size = square_size
        
        # 标定数据存储
        self.robot_poses = []  # 机器人末端位姿列表 (base -> end-effector)
        self.camera_poses = []  # 相机位姿列表 (camera -> board)
        self.calibration_images = []  # 标定图像
        self.board_corners = []  # 检测到的角点
        
        # 标定结果
        self.hand_eye_transform = None  # 手眼变换矩阵 (end-effector -> camera)
        self.calibration_error = None  # 标定误差
        
        # 生成标定板的世界坐标点
        self._generate_object_points()
        
        logger.info("手眼标定器初始化完成")
    
    def _generate_object_points(self):
        """生成标定板在其自身坐标系下的3D点坐标"""
        # 创建标定板角点的3D坐标 (在标定板坐标系下)
        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.board_size[0], 
                              0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.object_points = objp
        
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
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)
        
        if ret:
            # 亚像素精度优化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            if show_corners:
                # 绘制角点
                vis_image = image.copy()
                cv2.drawChessboardCorners(vis_image, self.board_size, corners, ret)
                cv2.imshow('Chessboard Corners', vis_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            
            logger.debug("成功检测到标定板角点")
            return True, corners
        else:
            logger.warning("未检测到标定板角点")
            return False, None
    
    def estimate_board_pose(self, corners: np.ndarray) -> Optional[SE3]:
        """
        估计标定板相对于相机的位姿
        
        Args:
            corners: 检测到的角点坐标
            
        Returns:
            标定板位姿 (camera -> board) 的SE3变换
        """
        if self.camera_matrix is None or self.distortion_coeffs is None:
            logger.error("相机参数未设置，无法进行位姿估计")
            return None
        
        # 使用PnP算法求解位姿
        success, rvec, tvec = cv2.solvePnP(
            self.object_points, 
            corners, 
            self.camera_matrix, 
            self.distortion_coeffs
        )
        
        if success:
            # 转换为旋转矩阵
            rmat, _ = cv2.Rodrigues(rvec)
            
            # 构造变换矩阵
            T = np.eye(4)
            T[:3, :3] = rmat
            T[:3, 3] = tvec.flatten()
            
            return SE3(T)
        else:
            logger.error("PnP位姿估计失败")
            return None
    
    def add_calibration_sample(self, 
                              robot_pose: Union[SE3, np.ndarray, List[float]], 
                              image: np.ndarray,
                              verify_detection: bool = True) -> bool:
        """
        添加一个标定样本
        
        Args:
            robot_pose: 机器人末端位姿 (base -> end-effector)
            image: 相机图像
            verify_detection: 是否验证角点检测结果
            
        Returns:
            是否成功添加样本
        """
        # 转换机器人位姿格式
        if isinstance(robot_pose, list):
            if len(robot_pose) == 6:  # [x, y, z, rx, ry, rz]
                x, y, z, rx, ry, rz = robot_pose
                robot_se3 = SE3([x, y, z]) * SE3.RPY([rx, ry, rz], order='zyx')
            else:
                logger.error(f"机器人位姿列表长度应为6，实际为{len(robot_pose)}")
                return False
        elif isinstance(robot_pose, np.ndarray):
            if robot_pose.shape == (4, 4):
                robot_se3 = SE3(robot_pose)
            elif robot_pose.shape == (6,):
                x, y, z, rx, ry, rz = robot_pose
                robot_se3 = SE3([x, y, z]) * SE3.RPY([rx, ry, rz], order='zyx')
            else:
                logger.error(f"机器人位姿数组形状不正确: {robot_pose.shape}")
                return False
        elif isinstance(robot_pose, SE3):
            robot_se3 = robot_pose
        else:
            logger.error(f"不支持的机器人位姿格式: {type(robot_pose)}")
            return False
        
        # 检测标定板角点
        success, corners = self.detect_chessboard(image, show_corners=verify_detection)
        if not success:
            logger.warning("标定板检测失败，跳过此样本")
            return False
        
        # 估计标定板位姿
        board_pose = self.estimate_board_pose(corners)
        if board_pose is None:
            logger.warning("标定板位姿估计失败，跳过此样本")
            return False
        
        # 存储标定数据
        self.robot_poses.append(robot_se3)
        self.camera_poses.append(board_pose)
        self.calibration_images.append(image.copy())
        self.board_corners.append(corners)
        
        logger.info(f"成功添加第 {len(self.robot_poses)} 个标定样本")
        return True
    
    def solve_hand_eye_calibration(self, method: str = 'tsai') -> bool:
        """
        求解手眼标定
        
        Args:
            method: 标定方法 ('tsai', 'park', 'horaud', 'andreff', 'daniilidis')
            
        Returns:
            是否标定成功
        """
        if len(self.robot_poses) < 3:
            logger.error(f"标定样本数量不足，至少需要3个，当前有{len(self.robot_poses)}个")
            return False
        
        logger.info(f"开始手眼标定，使用 {method} 方法，样本数量: {len(self.robot_poses)}")
        
        # 准备OpenCV标定数据
        R_gripper2base = []  # 机器人末端到基座的旋转矩阵
        t_gripper2base = []  # 机器人末端到基座的平移向量
        R_target2cam = []    # 标定板到相机的旋转矩阵
        t_target2cam = []    # 标定板到相机的平移向量
        
        for robot_pose, camera_pose in zip(self.robot_poses, self.camera_poses):
            # 机器人位姿 (base -> end-effector)
            robot_T = robot_pose.A
            R_gripper2base.append(robot_T[:3, :3])
            t_gripper2base.append(robot_T[:3, 3])
            
            # 相机位姿 (camera -> board)，需要转换为 (board -> camera)
            camera_T = camera_pose.inv().A
            R_target2cam.append(camera_T[:3, :3])
            t_target2cam.append(camera_T[:3, 3])
        
        # 选择标定方法
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
        }
        
        if method not in method_map:
            logger.error(f"不支持的标定方法: {method}")
            return False
        
        try:
            # 执行手眼标定
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method_map[method]
            )
            
            # 构造手眼变换矩阵
            T_cam2gripper = np.eye(4)
            T_cam2gripper[:3, :3] = R_cam2gripper
            T_cam2gripper[:3, 3] = t_cam2gripper.flatten()
            
            self.hand_eye_transform = SE3(T_cam2gripper)
            
            # 计算标定误差
            self._compute_calibration_error()
            
            logger.info(f"手眼标定成功完成，重投影误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            logger.error(f"手眼标定失败: {e}")
            return False
    
    def _compute_calibration_error(self) -> float:
        """计算标定误差"""
        if self.hand_eye_transform is None:
            return float('inf')
        
        errors = []
        
        for i, (robot_pose, camera_pose) in enumerate(zip(self.robot_poses, self.camera_poses)):
            # 通过手眼变换预测标定板位姿
            predicted_board_pose = robot_pose * self.hand_eye_transform * camera_pose
            
            # 与第一个位姿比较（作为参考）
            if i == 0:
                reference_pose = predicted_board_pose
                continue
            
            # 计算位姿差异
            pose_diff = reference_pose.inv() * predicted_board_pose
            
            # 计算平移和旋转误差
            trans_error = np.linalg.norm(pose_diff.t)
            rot_error = np.linalg.norm(pose_diff.rpy())
            
            errors.append(trans_error + rot_error * 0.1)  # 权重调整
        
        self.calibration_error = np.mean(errors) if errors else 0.0
        return self.calibration_error
    
    def save_calibration(self, filepath: str) -> bool:
        """
        保存标定结果
        
        Args:
            filepath: 保存路径
            
        Returns:
            是否保存成功
        """
        if self.hand_eye_transform is None:
            logger.error("没有标定结果可保存")
            return False
        
        try:
            # 准备保存数据
            calibration_data = {
                'timestamp': datetime.now().isoformat(),
                'hand_eye_transform': self.hand_eye_transform.A.tolist(),
                'calibration_error': float(self.calibration_error),
                'num_samples': len(self.robot_poses),
                'camera_matrix': self.camera_matrix.tolist() if self.camera_matrix is not None else None,
                'distortion_coeffs': self.distortion_coeffs.tolist() if self.distortion_coeffs is not None else None,
                'board_size': self.board_size,
                'square_size': float(self.square_size)
            }
            
            # 保存到JSON文件
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(calibration_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"标定结果已保存到: {filepath}")
            return True
            
        except Exception as e:
            logger.error(f"保存标定结果失败: {e}")
            return False
    
    def load_calibration(self, filepath: str) -> bool:
        """
        加载标定结果
        
        Args:
            filepath: 文件路径
            
        Returns:
            是否加载成功
        """
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                calibration_data = json.load(f)
            
            # 加载手眼变换
            self.hand_eye_transform = SE3(np.array(calibration_data['hand_eye_transform']))
            self.calibration_error = calibration_data['calibration_error']
            
            # 加载相机参数
            if calibration_data['camera_matrix'] is not None:
                self.camera_matrix = np.array(calibration_data['camera_matrix'])
            if calibration_data['distortion_coeffs'] is not None:
                self.distortion_coeffs = np.array(calibration_data['distortion_coeffs'])
            
            # 加载标定板参数
            self.board_size = tuple(calibration_data['board_size'])
            self.square_size = calibration_data['square_size']
            
            logger.info(f"标定结果加载成功，标定误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            logger.error(f"加载标定结果失败: {e}")
            return False
    
    def get_camera_pose_in_base(self, robot_pose: Union[SE3, np.ndarray, List[float]]) -> Optional[SE3]:
        """
        根据机器人位姿计算相机在基座坐标系中的位姿
        
        Args:
            robot_pose: 机器人末端位姿 (base -> end-effector)
            
        Returns:
            相机位姿 (base -> camera)
        """
        if self.hand_eye_transform is None:
            logger.error("尚未完成手眼标定")
            return None
        
        # 转换机器人位姿格式
        if isinstance(robot_pose, list):
            if len(robot_pose) == 6:
                x, y, z, rx, ry, rz = robot_pose
                robot_se3 = SE3([x, y, z]) * SE3.RPY([rx, ry, rz], order='zyx')
            else:
                logger.error(f"机器人位姿列表长度应为6")
                return None
        elif isinstance(robot_pose, np.ndarray):
            if robot_pose.shape == (4, 4):
                robot_se3 = SE3(robot_pose)
            elif robot_pose.shape == (6,):
                x, y, z, rx, ry, rz = robot_pose
                robot_se3 = SE3([x, y, z]) * SE3.RPY([rx, ry, rz], order='zyx')
            else:
                logger.error(f"机器人位姿数组形状不正确")
                return None
        elif isinstance(robot_pose, SE3):
            robot_se3 = robot_pose
        else:
            logger.error(f"不支持的机器人位姿格式")
            return None
        
        # 计算相机位姿 = 机器人末端位姿 * 手眼变换
        camera_pose = robot_se3 * self.hand_eye_transform
        return camera_pose
    
    def validate_calibration(self, test_robot_pose: Union[SE3, np.ndarray, List[float]], 
                           test_image: np.ndarray) -> Dict[str, float]:
        """
        验证标定精度
        
        Args:
            test_robot_pose: 测试机器人位姿
            test_image: 测试图像
            
        Returns:
            验证结果字典，包含各种误差指标
        """
        if self.hand_eye_transform is None:
            logger.error("尚未完成手眼标定")
            return {}
        
        # 检测测试图像中的标定板
        success, corners = self.detect_chessboard(test_image)
        if not success:
            logger.error("测试图像中未检测到标定板")
            return {}
        
        # 估计标定板位姿
        board_pose = self.estimate_board_pose(corners)
        if board_pose is None:
            logger.error("测试图像标定板位姿估计失败")
            return {}
        
        # 通过手眼标定预测相机位姿
        predicted_camera_pose = self.get_camera_pose_in_base(test_robot_pose)
        if predicted_camera_pose is None:
            return {}
        
        # 计算预测的标定板位姿
        predicted_board_pose = predicted_camera_pose * board_pose
        
        # 与训练数据中的标定板位姿比较
        if not self.camera_poses:
            logger.error("没有参考标定板位姿")
            return {}
        
        reference_board_pose = self.robot_poses[0] * self.hand_eye_transform * self.camera_poses[0]
        
        # 计算误差
        pose_diff = reference_board_pose.inv() * predicted_board_pose
        
        translation_error = np.linalg.norm(pose_diff.t)
        rotation_error = np.linalg.norm(pose_diff.rpy())
        
        validation_results = {
            'translation_error_mm': translation_error * 1000,
            'rotation_error_deg': np.degrees(rotation_error),
            'total_error': translation_error + rotation_error * 0.1
        }
        
        logger.info(f"标定验证结果: "
                   f"平移误差={validation_results['translation_error_mm']:.2f}mm, "
                   f"旋转误差={validation_results['rotation_error_deg']:.2f}°")
        
        return validation_results
    
    def clear_samples(self):
        """清空所有标定样本"""
        self.robot_poses.clear()
        self.camera_poses.clear()
        self.calibration_images.clear()
        self.board_corners.clear()
        logger.info("已清空所有标定样本")
    
    def get_calibration_info(self) -> Dict:
        """获取标定信息摘要"""
        info = {
            'num_samples': len(self.robot_poses),
            'is_calibrated': self.hand_eye_transform is not None,
            'calibration_error': self.calibration_error,
            'board_size': self.board_size,
            'square_size': self.square_size,
            'camera_matrix_available': self.camera_matrix is not None,
            'distortion_coeffs_available': self.distortion_coeffs is not None
        }
        
        if self.hand_eye_transform is not None:
            # 提取手眼变换的平移和旋转
            t = self.hand_eye_transform.t
            rpy = self.hand_eye_transform.rpy()
            info['hand_eye_translation'] = t.tolist()
            info['hand_eye_rotation_rpy'] = rpy.tolist()
        
        return info


def create_checkerboard_image(board_size: Tuple[int, int] = (9, 6), 
                            square_size_pixels: int = 50,
                            save_path: str = None) -> np.ndarray:
    """
    创建标定板图像用于测试
    
    Args:
        board_size: 标定板大小 (列数, 行数)
        square_size_pixels: 方格像素大小
        save_path: 保存路径
        
    Returns:
        标定板图像
    """
    cols, rows = board_size
    total_cols = cols + 1
    total_rows = rows + 1
    
    # 创建棋盘格图像
    board_image = np.zeros((total_rows * square_size_pixels, 
                           total_cols * square_size_pixels), dtype=np.uint8)
    
    for i in range(total_rows):
        for j in range(total_cols):
            if (i + j) % 2 == 0:
                y_start = i * square_size_pixels
                y_end = (i + 1) * square_size_pixels
                x_start = j * square_size_pixels
                x_end = (j + 1) * square_size_pixels
                board_image[y_start:y_end, x_start:x_end] = 255
    
    if save_path:
        cv2.imwrite(save_path, board_image)
        logger.info(f"标定板图像已保存到: {save_path}")
    
    return board_image


if __name__ == "__main__":
    # 示例用法
    
    # 模拟相机内参
    camera_matrix = np.array([
        [800, 0, 320],
        [0, 800, 240],
        [0, 0, 1]
    ], dtype=np.float32)
    
    distortion_coeffs = np.array([0.1, -0.2, 0.001, 0.001, 0.1], dtype=np.float32)
    
    # 创建手眼标定器
    calibrator = HandEyeCalibration(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        board_size=(9, 6),
        square_size=0.025
    )
    
    # 创建测试标定板图像
    test_board = create_checkerboard_image((9, 6), 50, "test_checkerboard.png")
    
    # 显示标定信息
    info = calibrator.get_calibration_info()
    print("标定器信息:", info)
