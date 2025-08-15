# -*- coding: utf-8 -*-
# eye_to_hand_calibration.py - 眼在手外标定模块

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

class EyeToHandCalibration:
    """
    眼在手外（Eye-to-Hand）标定类
    
    该类实现了眼在手外标定的完整流程，解决 AX = YB 问题：
    - A: 机器人base到end-effector的变换序列
    - X: base到camera的变换（待求解）
    - Y: end-effector到标定板的变换（已知固定变换）
    - B: camera到标定板的变换序列
    
    眼在手外配置中，相机固定在外部位置，标定板安装在机械臂末端
    """
    
    def __init__(self, 
                 camera_matrix: np.ndarray = None,
                 distortion_coeffs: np.ndarray = None,
                 board_size: Tuple[int, int] = (9, 6),
                 square_size: float = 0.025,
                 board_to_end_transform: np.ndarray = None):
        """
        初始化眼在手外标定器
        
        Args:
            camera_matrix: 相机内参矩阵 (3x3)
            distortion_coeffs: 相机畸变系数
            board_size: 标定板角点数量 (列数, 行数)
            square_size: 标定板方格尺寸 (米)
            board_to_end_transform: 标定板相对于机械臂末端的变换矩阵 (4x4)
        """
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        self.board_size = board_size
        self.square_size = square_size
        
        # 标定板到末端的变换（已知）
        if board_to_end_transform is None:
            # 默认：标定板与末端重合
            self.board_to_end_transform = SE3()
        else:
            self.board_to_end_transform = SE3(board_to_end_transform)
        
        # 标定数据存储
        self.robot_poses = []  # 机器人末端位姿列表 (base -> end-effector)
        self.camera_poses = []  # 相机观测到的标定板位姿列表 (camera -> board)
        self.calibration_images = []  # 标定图像
        self.board_corners = []  # 检测到的角点
        
        # 标定结果
        self.eye_to_hand_transform = None  # 眼在手外变换矩阵 (base -> camera)
        self.calibration_error = None  # 标定误差
        
        # 生成标定板的世界坐标点
        self._generate_object_points()
        
        logger.info("眼在手外标定器初始化完成")
    
    def _generate_object_points(self):
        """生成标定板在其自身坐标系下的3D点坐标"""
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
                cv2.imshow('Eye-to-Hand Calibration - Chessboard Corners', vis_image)
                cv2.waitKey(1000)
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
    
    def solve_eye_to_hand_calibration(self, method: str = 'tsai') -> bool:
        """
        求解眼在手外标定
        
        Args:
            method: 标定方法 ('tsai', 'park', 'horaud', 'andreff', 'daniilidis')
            
        Returns:
            是否标定成功
        """
        if len(self.robot_poses) < 3:
            logger.error(f"标定样本数量不足，至少需要3个，当前有{len(self.robot_poses)}个")
            return False
        
        logger.info(f"开始眼在手外标定，使用 {method} 方法，样本数量: {len(self.robot_poses)}")
        
        # 准备OpenCV标定数据
        # 对于眼在手外标定，我们需要变换 AX = YB 到 A'X' = X'B' 的形式
        # 其中 A' = A * Y^(-1), X' = Y * X, B' = Y * B
        
        R_gripper2base = []  # 机器人末端到基座的旋转矩阵
        t_gripper2base = []  # 机器人末端到基座的平移向量
        R_target2cam = []    # 标定板到相机的旋转矩阵
        t_target2cam = []    # 标定板到相机的平移向量
        
        # 获取标定板到末端的变换
        Y = self.board_to_end_transform
        Y_inv = Y.inv()
        
        for robot_pose, camera_pose in zip(self.robot_poses, self.camera_poses):
            # 机器人位姿 (base -> end-effector)
            A = robot_pose
            
            # 变换 A' = A * Y^(-1)
            A_prime = A * Y_inv
            A_prime_T = A_prime.A
            R_gripper2base.append(A_prime_T[:3, :3])
            t_gripper2base.append(A_prime_T[:3, 3])
            
            # 相机位姿 (camera -> board)，需要转换为 (board -> camera)
            B = camera_pose.inv()
            
            # 变换 B' = Y * B
            B_prime = Y * B
            B_prime_T = B_prime.A
            R_target2cam.append(B_prime_T[:3, :3])
            t_target2cam.append(B_prime_T[:3, 3])
        
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
            
            # 构造变换矩阵 X' = Y * X
            T_cam2gripper = np.eye(4)
            T_cam2gripper[:3, :3] = R_cam2gripper
            T_cam2gripper[:3, 3] = t_cam2gripper.flatten()
            
            X_prime = SE3(T_cam2gripper)
            
            # 恢复原始变换 X = Y^(-1) * X'
            self.eye_to_hand_transform = Y_inv * X_prime
            
            # 计算标定误差
            self._compute_calibration_error()
            
            logger.info(f"眼在手外标定成功完成，重投影误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            logger.error(f"眼在手外标定失败: {e}")
            return False
    
    def _compute_calibration_error(self) -> float:
        """计算标定误差"""
        if self.eye_to_hand_transform is None:
            return float('inf')
        
        errors = []
        
        for i, (robot_pose, camera_pose) in enumerate(zip(self.robot_poses, self.camera_poses)):
            # 通过眼在手外变换预测标定板位姿
            # 标定板在基座坐标系中的位姿
            board_in_base = robot_pose * self.board_to_end_transform
            
            # 通过相机观测预测的标定板位姿
            predicted_board_in_base = self.eye_to_hand_transform.inv() * camera_pose.inv()
            
            # 计算位姿差异
            pose_diff = board_in_base.inv() * predicted_board_in_base
            
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
        if self.eye_to_hand_transform is None:
            logger.error("没有标定结果可保存")
            return False
        
        try:
            # 准备保存数据
            calibration_data = {
                'timestamp': datetime.now().isoformat(),
                'calibration_type': 'eye_to_hand',
                'eye_to_hand_transform': self.eye_to_hand_transform.A.tolist(),
                'board_to_end_transform': self.board_to_end_transform.A.tolist(),
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
            
            logger.info(f"眼在手外标定结果已保存到: {filepath}")
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
            
            # 检查标定类型
            if calibration_data.get('calibration_type') != 'eye_to_hand':
                logger.warning("加载的不是眼在手外标定文件")
            
            # 加载眼在手外变换
            self.eye_to_hand_transform = SE3(np.array(calibration_data['eye_to_hand_transform']))
            self.board_to_end_transform = SE3(np.array(calibration_data['board_to_end_transform']))
            self.calibration_error = calibration_data['calibration_error']
            
            # 加载相机参数
            if calibration_data['camera_matrix'] is not None:
                self.camera_matrix = np.array(calibration_data['camera_matrix'])
            if calibration_data['distortion_coeffs'] is not None:
                self.distortion_coeffs = np.array(calibration_data['distortion_coeffs'])
            
            # 加载标定板参数
            self.board_size = tuple(calibration_data['board_size'])
            self.square_size = calibration_data['square_size']
            
            logger.info(f"眼在手外标定结果加载成功，标定误差: {self.calibration_error:.6f}")
            return True
            
        except Exception as e:
            logger.error(f"加载标定结果失败: {e}")
            return False
    
    def get_camera_pose_in_base(self) -> Optional[SE3]:
        """
        获取相机在基座坐标系中的位姿
        
        Returns:
            相机位姿 (base -> camera)
        """
        if self.eye_to_hand_transform is None:
            logger.error("尚未完成眼在手外标定")
            return None
        
        return self.eye_to_hand_transform
    
    def get_board_pose_from_robot(self, robot_pose: Union[SE3, np.ndarray, List[float]]) -> Optional[SE3]:
        """
        根据机器人位姿计算标定板在基座坐标系中的位姿
        
        Args:
            robot_pose: 机器人末端位姿 (base -> end-effector)
            
        Returns:
            标定板位姿 (base -> board)
        """
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
        
        # 计算标定板位姿 = 机器人末端位姿 * 末端到标定板变换
        board_pose = robot_se3 * self.board_to_end_transform
        return board_pose
    
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
        if self.eye_to_hand_transform is None:
            logger.error("尚未完成眼在手外标定")
            return {}
        
        # 检测测试图像中的标定板
        success, corners = self.detect_chessboard(test_image)
        if not success:
            logger.error("测试图像中未检测到标定板")
            return {}
        
        # 估计标定板位姿
        observed_board_pose = self.estimate_board_pose(corners)
        if observed_board_pose is None:
            logger.error("测试图像标定板位姿估计失败")
            return {}
        
        # 通过机器人位姿和眼在手外标定预测标定板位姿
        predicted_board_in_base = self.get_board_pose_from_robot(test_robot_pose)
        if predicted_board_in_base is None:
            return {}
        
        # 将预测的标定板位姿转换到相机坐标系
        predicted_board_in_camera = self.eye_to_hand_transform * predicted_board_in_base
        
        # 计算误差
        pose_diff = observed_board_pose.inv() * predicted_board_in_camera
        
        translation_error = np.linalg.norm(pose_diff.t)
        rotation_error = np.linalg.norm(pose_diff.rpy())
        
        validation_results = {
            'translation_error_mm': translation_error * 1000,
            'rotation_error_deg': np.degrees(rotation_error),
            'total_error': translation_error + rotation_error * 0.1
        }
        
        logger.info(f"眼在手外标定验证结果: "
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
            'calibration_type': 'eye_to_hand',
            'num_samples': len(self.robot_poses),
            'is_calibrated': self.eye_to_hand_transform is not None,
            'calibration_error': self.calibration_error,
            'board_size': self.board_size,
            'square_size': self.square_size,
            'camera_matrix_available': self.camera_matrix is not None,
            'distortion_coeffs_available': self.distortion_coeffs is not None
        }
        
        if self.eye_to_hand_transform is not None:
            # 提取眼在手外变换的平移和旋转
            t = self.eye_to_hand_transform.t
            rpy = self.eye_to_hand_transform.rpy()
            info['eye_to_hand_translation'] = t.tolist()
            info['eye_to_hand_rotation_rpy'] = rpy.tolist()
        
        if self.board_to_end_transform is not None:
            # 提取标定板到末端变换的平移和旋转
            t = self.board_to_end_transform.t
            rpy = self.board_to_end_transform.rpy()
            info['board_to_end_translation'] = t.tolist()
            info['board_to_end_rotation_rpy'] = rpy.tolist()
        
        return info


if __name__ == "__main__":
    # 示例用法
    
    # 模拟相机内参
    camera_matrix = np.array([
        [800, 0, 320],
        [0, 800, 240],
        [0, 0, 1]
    ], dtype=np.float32)
    
    distortion_coeffs = np.array([0.1, -0.2, 0.001, 0.001, 0.1], dtype=np.float32)
    
    # 创建眼在手外标定器
    calibrator = EyeToHandCalibration(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        board_size=(9, 6),
        square_size=0.025
    )
    
    # 显示标定信息
    info = calibrator.get_calibration_info()
    print("眼在手外标定器信息:", info)
