# -*- coding: utf-8 -*-
# calibration_data_collector.py - 标定数据收集器

import cv2
import numpy as np
import time
import threading
from typing import List, Dict, Optional, Callable, Tuple, Union
from spatialmath import SE3
import json
from datetime import datetime
import os

from armpi_common._log import logger
from armpi_common.robot_arm_controller import RobotArmController
from armpi_common.hand_eye_calibration import HandEyeCalibration
from armpi_common.eye_to_hand_calibration import EyeToHandCalibration


class CalibrationDataCollector:
    """
    标定数据收集器
    
    该类负责自动化收集手眼标定所需的数据，支持两种标定模式：
    1. 眼在手上 (Eye-in-Hand): 相机固定在机械臂末端，标定板固定在外部
    2. 眼在手外 (Eye-to-Hand): 相机固定在外部，标定板固定在机械臂末端
    
    功能包括：
    1. 控制机械臂移动到不同位姿
    2. 采集相机图像
    3. 检测标定板
    4. 记录机器人位姿和相机数据
    """
    
    def __init__(self, 
                 robot_controller: RobotArmController,
                 camera_source: int = 0,
                 save_directory: str = "./calibration_data",
                 calibration_type: str = "eye_in_hand",
                 camera_pose: List[float] = None):
        """
        初始化标定数据收集器
        
        Args:
            robot_controller: 机械臂控制器
            camera_source: 相机设备ID或视频文件路径
            save_directory: 数据保存目录
            calibration_type: 标定类型 ("eye_in_hand" 或 "eye_to_hand")
            camera_pose: 眼在手外模式下相机的固定位姿 [x,y,z,rx,ry,rz] (可选)
        """
        self.robot_controller = robot_controller
        self.camera_source = camera_source
        self.save_directory = save_directory
        
        # 标定类型验证
        if calibration_type not in ["eye_in_hand", "eye_to_hand"]:
            raise ValueError("calibration_type 必须是 'eye_in_hand' 或 'eye_to_hand'")
        self.calibration_type = calibration_type
        
        # 相机位姿（用于眼在手外模式）
        self.camera_pose = camera_pose
        
        # 相机对象
        self.cap = None
        
        # 数据存储
        self.collected_poses = []  # 机器人位姿列表
        self.collected_images = []  # 采集的图像列表
        self.collection_timestamps = []  # 采集时间戳
        
        # 标定器
        self.calibrator = None
        
        # 采集状态
        self.is_collecting = False
        self.collection_thread = None
        
        # 创建保存目录
        os.makedirs(save_directory, exist_ok=True)
        
        logger.info(f"标定数据收集器初始化完成，模式: {calibration_type}，保存目录: {save_directory}")
    
    def initialize_camera(self) -> bool:
        """
        初始化相机
        
        Returns:
            是否初始化成功
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_source)
            if not self.cap.isOpened():
                logger.error(f"无法打开相机: {self.camera_source}")
                return False
            
            # 设置相机参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            logger.info("相机初始化成功")
            return True
            
        except Exception as e:
            logger.error(f"相机初始化失败: {e}")
            return False
    
    def capture_image(self) -> Optional[np.ndarray]:
        """
        采集一帧图像
        
        Returns:
            采集的图像，失败返回None
        """
        if self.cap is None:
            logger.error("相机未初始化")
            return None
        
        ret, frame = self.cap.read()
        if ret:
            return frame
        else:
            logger.warning("图像采集失败")
            return None
    
    def generate_calibration_poses(self, 
                                 num_poses: int = 15,
                                 workspace_center: List[float] = [0.15, 0.0, 0.20],
                                 workspace_radius: float = 0.05,
                                 height_variation: float = 0.03,
                                 orientation_variation: float = 0.3) -> List[List[float]]:
        """
        生成标定用的机器人位姿序列
        
        Args:
            num_poses: 生成位姿数量
            workspace_center: 工作空间中心 [x, y, z]
            workspace_radius: 工作空间半径
            height_variation: 高度变化范围
            orientation_variation: 姿态变化范围（弧度）
            
        Returns:
            位姿列表，每个位姿为 [x, y, z, rx, ry, rz]
        """
        poses = []
        
        if self.calibration_type == "eye_in_hand":
            # 眼在手上：相机朝下看固定的标定板
            base_orientation = [0.0, 0.0, -np.pi]
            
            for i in range(num_poses):
                # 在圆形区域内随机生成位置
                angle = 2 * np.pi * i / num_poses + np.random.uniform(-0.2, 0.2)
                radius = workspace_radius * np.random.uniform(0.3, 1.0)
                
                x = workspace_center[0] + radius * np.cos(angle)
                y = workspace_center[1] + radius * np.sin(angle)
                z = workspace_center[2] + np.random.uniform(-height_variation, height_variation)
                
                # 随机姿态变化
                rx = base_orientation[0] + np.random.uniform(-orientation_variation, orientation_variation)
                ry = base_orientation[1] + np.random.uniform(-orientation_variation, orientation_variation)
                rz = base_orientation[2] + np.random.uniform(-orientation_variation, orientation_variation)
                
                poses.append([x, y, z, rx, ry, rz])
                
        elif self.calibration_type == "eye_to_hand":
            # 眼在手外：标定板在机械臂末端，需要面向固定的相机
            if self.camera_pose is not None:
                # 已知相机位姿的情况，生成面向相机的位姿
                cam_x, cam_y, cam_z = self.camera_pose[:3]
                camera_position = np.array([cam_x, cam_y, cam_z])
            else:
                # 默认相机位置（假设相机在机械臂前方上方）
                camera_position = np.array([0.0, -0.3, 0.4])
            
            for i in range(num_poses):
                # 在球形区域内生成位置，确保标定板能被相机看到
                # 使用球坐标系生成均匀分布的位置
                phi = np.random.uniform(0, 2 * np.pi)  # 方位角
                theta = np.random.uniform(np.pi/6, np.pi/3)  # 仰角（避免过于垂直）
                radius = workspace_radius * np.random.uniform(0.5, 1.0)
                
                # 相对于工作空间中心的偏移
                x_offset = radius * np.sin(theta) * np.cos(phi)
                y_offset = radius * np.sin(theta) * np.sin(phi)
                z_offset = radius * np.cos(theta) + np.random.uniform(-height_variation, height_variation)
                
                x = workspace_center[0] + x_offset
                y = workspace_center[1] + y_offset
                z = workspace_center[2] + z_offset
                
                # 计算从标定板位置到相机的方向向量
                board_position = np.array([x, y, z])
                direction_to_camera = camera_position - board_position
                direction_to_camera = direction_to_camera / np.linalg.norm(direction_to_camera)
                
                # 计算旋转使标定板法向量指向相机
                # 标定板默认法向量为 [0, 0, 1] (z轴正方向)
                z_axis = direction_to_camera
                
                # 生成合理的x轴（避免与z轴平行）
                if abs(z_axis[2]) < 0.9:
                    x_axis = np.cross([0, 0, 1], z_axis)
                else:
                    x_axis = np.cross([1, 0, 0], z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                
                # y轴通过叉积得到
                y_axis = np.cross(z_axis, x_axis)
                
                # 构造旋转矩阵
                rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
                
                # 转换为欧拉角 (ZYX顺序)
                from spatialmath.base import tr2rpy
                rpy = tr2rpy(rotation_matrix, order='zyx')
                
                # 添加随机姿态变化
                rx = rpy[0] + np.random.uniform(-orientation_variation, orientation_variation)
                ry = rpy[1] + np.random.uniform(-orientation_variation, orientation_variation)
                rz = rpy[2] + np.random.uniform(-orientation_variation, orientation_variation)
                
                poses.append([x, y, z, rx, ry, rz])
        
        logger.info(f"生成了 {len(poses)} 个{self.calibration_type}标定位姿")
        return poses
    
    def move_to_pose_safely(self, 
                           target_pose: List[float], 
                           move_time: int = 3000,
                           verify_arrival: bool = True) -> bool:
        """
        安全地移动到目标位姿
        
        Args:
            target_pose: 目标位姿 [x, y, z, rx, ry, rz]
            move_time: 移动时间（毫秒）
            verify_arrival: 是否验证到达目标位姿
            
        Returns:
            是否成功到达目标位姿
        """
        try:
            # 检查目标位姿是否可达
            ik_result = self.robot_controller.get_joint_ikine(target_pose)
            if not ik_result['status']:
                logger.warning(f"目标位姿不可达: {target_pose}")
                return False
            
            # 发送运动指令
            move_result = self.robot_controller.set_joint_move_with_coordinate(
                target_pose, move_type=0, move_time=move_time
            )
            if not move_result['status']:
                logger.error(f"运动指令发送失败: {move_result['info']}")
                return False
            
            # 等待运动完成
            time.sleep(move_time / 1000.0 + 0.5)
            
            if verify_arrival:
                # 验证是否到达目标位姿
                current_pose = self.robot_controller.get_joint_fkine(current_pose=True)
                if current_pose['fkine'] is None:
                    logger.warning("无法获取当前位姿进行验证")
                    return True  # 假设成功
                
                # 检查位姿差异
                current = np.array(current_pose['fkine'])
                target = np.array(target_pose)
                
                position_error = np.linalg.norm(current[:3] - target[:3])
                orientation_error = np.linalg.norm(current[3:] - target[3:])
                
                if position_error > 0.01 or orientation_error > 0.1:  # 容差
                    logger.warning(f"位姿到达精度不足，位置误差: {position_error:.4f}m, "
                                 f"姿态误差: {orientation_error:.4f}rad")
                    return False
            
            logger.debug(f"成功移动到位姿: {target_pose}")
            return True
            
        except Exception as e:
            logger.error(f"移动到位姿时发生异常: {e}")
            return False
    
    def collect_single_sample(self, 
                            robot_pose: List[float],
                            sample_id: int,
                            show_preview: bool = True,
                            save_image: bool = True) -> bool:
        """
        采集单个标定样本
        
        Args:
            robot_pose: 机器人目标位姿
            sample_id: 样本编号
            show_preview: 是否显示预览
            save_image: 是否保存图像
            
        Returns:
            是否成功采集样本
        """
        logger.info(f"开始采集第 {sample_id} 个样本")
        
        # 移动到目标位姿
        if not self.move_to_pose_safely(robot_pose):
            logger.warning(f"移动到样本 {sample_id} 位姿失败")
            return False
        
        # 等待稳定
        time.sleep(1.0)
        
        # 采集图像
        image = self.capture_image()
        if image is None:
            logger.warning(f"样本 {sample_id} 图像采集失败")
            return False
        
        # 获取实际机器人位姿
        actual_pose = self.robot_controller.get_joint_fkine(current_pose=True)
        if actual_pose['fkine'] is None:
            logger.warning(f"获取样本 {sample_id} 实际位姿失败")
            return False
        
        # 显示预览（可选）
        if show_preview and self.calibrator is not None:
            success, corners = self.calibrator.detect_chessboard(image, show_corners=False)
            preview_image = image.copy()
            
            if success:
                cv2.drawChessboardCorners(preview_image, 
                                        self.calibrator.board_size, 
                                        corners, True)
                cv2.putText(preview_image, f"Sample {sample_id} - OK", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(preview_image, f"Sample {sample_id} - Failed", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            cv2.imshow(f'Calibration Sample {sample_id}', preview_image)
            cv2.waitKey(1000)  # 显示1秒
            cv2.destroyAllWindows()
            
            if not success:
                logger.warning(f"样本 {sample_id} 中未检测到标定板")
                return False
        
        # 保存数据
        self.collected_poses.append(actual_pose['fkine'])
        self.collected_images.append(image)
        self.collection_timestamps.append(datetime.now().isoformat())
        
        # 保存图像文件（可选）
        if save_image:
            image_filename = os.path.join(self.save_directory, f"sample_{sample_id:03d}.jpg")
            cv2.imwrite(image_filename, image)
        
        logger.info(f"成功采集样本 {sample_id}")
        return True
    
    def collect_calibration_data(self, 
                               poses: List[List[float]],
                               show_preview: bool = True,
                               save_images: bool = True,
                               retry_failed: bool = True) -> Dict:
        """
        批量采集标定数据
        
        Args:
            poses: 机器人位姿列表
            show_preview: 是否显示预览
            save_images: 是否保存图像
            retry_failed: 是否重试失败的样本
            
        Returns:
            采集结果统计
        """
        logger.info(f"开始批量采集标定数据，总数: {len(poses)}")
        
        # 初始化相机
        if not self.initialize_camera():
            return {'success': False, 'error': '相机初始化失败'}
        
        successful_samples = 0
        failed_samples = 0
        failed_indices = []
        
        try:
            for i, pose in enumerate(poses):
                success = self.collect_single_sample(
                    pose, i+1, show_preview, save_images
                )
                
                if success:
                    successful_samples += 1
                else:
                    failed_samples += 1
                    failed_indices.append(i)
                
                # 短暂停顿
                time.sleep(0.5)
            
            # 重试失败的样本
            if retry_failed and failed_indices:
                logger.info(f"重试 {len(failed_indices)} 个失败的样本")
                
                for idx in failed_indices.copy():
                    logger.info(f"重试样本 {idx+1}")
                    success = self.collect_single_sample(
                        poses[idx], idx+1, show_preview, save_images
                    )
                    
                    if success:
                        successful_samples += 1
                        failed_samples -= 1
                        failed_indices.remove(idx)
        
        finally:
            # 清理资源
            if self.cap is not None:
                self.cap.release()
                cv2.destroyAllWindows()
        
        # 保存采集元数据
        self._save_collection_metadata(poses, failed_indices)
        
        result = {
            'success': True,
            'total_samples': len(poses),
            'successful_samples': successful_samples,
            'failed_samples': failed_samples,
            'failed_indices': failed_indices,
            'success_rate': successful_samples / len(poses) if poses else 0
        }
        
        logger.info(f"数据采集完成: 成功 {successful_samples}/{len(poses)} 个样本 "
                   f"(成功率: {result['success_rate']:.1%})")
        
        return result
    
    def _save_collection_metadata(self, poses: List[List[float]], failed_indices: List[int]):
        """保存采集元数据"""
        metadata = {
            'collection_time': datetime.now().isoformat(),
            'total_poses': len(poses),
            'successful_poses': len(self.collected_poses),
            'failed_indices': failed_indices,
            'target_poses': poses,
            'actual_poses': self.collected_poses,
            'timestamps': self.collection_timestamps
        }
        
        metadata_file = os.path.join(self.save_directory, 'collection_metadata.json')
        with open(metadata_file, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
        
        logger.info(f"采集元数据已保存到: {metadata_file}")
    
    def load_collected_data(self) -> bool:
        """
        加载之前采集的数据
        
        Returns:
            是否加载成功
        """
        try:
            metadata_file = os.path.join(self.save_directory, 'collection_metadata.json')
            
            if not os.path.exists(metadata_file):
                logger.warning("未找到采集元数据文件")
                return False
            
            with open(metadata_file, 'r', encoding='utf-8') as f:
                metadata = json.load(f)
            
            self.collected_poses = metadata['actual_poses']
            self.collection_timestamps = metadata['timestamps']
            
            # 加载图像
            self.collected_images = []
            for i in range(len(self.collected_poses)):
                image_file = os.path.join(self.save_directory, f"sample_{i+1:03d}.jpg")
                if os.path.exists(image_file):
                    image = cv2.imread(image_file)
                    self.collected_images.append(image)
                else:
                    logger.warning(f"图像文件不存在: {image_file}")
                    return False
            
            logger.info(f"成功加载 {len(self.collected_poses)} 个采集样本")
            return True
            
        except Exception as e:
            logger.error(f"加载采集数据失败: {e}")
            return False
    
    def add_data_to_calibrator(self, calibrator: Union[HandEyeCalibration, EyeToHandCalibration]) -> int:
        """
        将采集的数据添加到标定器
        
        Args:
            calibrator: 手眼标定器或眼在手外标定器
            
        Returns:
            成功添加的样本数量
        """
        self.calibrator = calibrator
        
        if not self.collected_poses or not self.collected_images:
            logger.warning("没有可用的采集数据")
            return 0
        
        # 验证标定器类型与收集器模式匹配
        if self.calibration_type == "eye_in_hand" and not isinstance(calibrator, HandEyeCalibration):
            logger.error("眼在手上模式需要使用 HandEyeCalibration")
            return 0
        elif self.calibration_type == "eye_to_hand" and not isinstance(calibrator, EyeToHandCalibration):
            logger.error("眼在手外模式需要使用 EyeToHandCalibration")
            return 0
        
        successful_additions = 0
        
        for i, (pose, image) in enumerate(zip(self.collected_poses, self.collected_images)):
            success = calibrator.add_calibration_sample(pose, image, verify_detection=False)
            if success:
                successful_additions += 1
            else:
                logger.warning(f"样本 {i+1} 添加到标定器失败")
        
        logger.info(f"成功添加 {successful_additions}/{len(self.collected_poses)} 个样本到{self.calibration_type}标定器")
        return successful_additions
    
    def quick_collect_and_calibrate(self, 
                                  num_poses: int = 15,
                                  calibration_method: str = 'tsai',
                                  camera_matrix: np.ndarray = None,
                                  distortion_coeffs: np.ndarray = None) -> Dict:
        """
        快速采集和标定流程
        
        Args:
            num_poses: 采集位姿数量
            calibration_method: 标定方法
            camera_matrix: 相机内参矩阵
            distortion_coeffs: 相机畸变系数
            
        Returns:
            标定结果
        """
        logger.info("开始快速采集和标定流程")
        
        # 生成标定位姿
        poses = self.generate_calibration_poses(num_poses)
        
        # 采集数据
        collection_result = self.collect_calibration_data(poses)
        if not collection_result['success'] or collection_result['successful_samples'] < 3:
            logger.error("数据采集失败或样本数量不足")
            return {'success': False, 'error': '数据采集失败'}
        
        # 创建标定器
        calibrator = HandEyeCalibration(
            camera_matrix=camera_matrix,
            distortion_coeffs=distortion_coeffs
        )
        
        # 添加采集数据
        added_samples = self.add_data_to_calibrator(calibrator)
        if added_samples < 3:
            logger.error("有效标定样本数量不足")
            return {'success': False, 'error': '有效样本不足'}
        
        # 执行标定
        calibration_success = calibrator.solve_hand_eye_calibration(calibration_method)
        if not calibration_success:
            logger.error("手眼标定失败")
            return {'success': False, 'error': '标定失败'}
        
        # 保存标定结果
        calibration_file = os.path.join(self.save_directory, 'hand_eye_calibration.json')
        calibrator.save_calibration(calibration_file)
        
        result = {
            'success': True,
            'collection_result': collection_result,
            'added_samples': added_samples,
            'calibration_error': calibrator.calibration_error,
            'calibration_file': calibration_file,
            'calibrator': calibrator
        }
        
        logger.info(f"快速标定完成，误差: {calibrator.calibration_error:.6f}")
        return result
    
    def clear_collected_data(self):
        """清空采集的数据"""
        self.collected_poses.clear()
        self.collected_images.clear()
        self.collection_timestamps.clear()
        logger.info("已清空采集数据")


def create_auto_calibration_workflow(robot_controller: RobotArmController,
                                   camera_source: int = 0,
                                   save_directory: str = "./auto_calibration") -> CalibrationDataCollector:
    """
    创建自动化标定工作流
    
    Args:
        robot_controller: 机械臂控制器
        camera_source: 相机源
        save_directory: 保存目录
        
    Returns:
        配置好的数据收集器
    """
    collector = CalibrationDataCollector(
        robot_controller=robot_controller,
        camera_source=camera_source,
        save_directory=save_directory
    )
    
    logger.info("自动化标定工作流已创建")
    return collector


if __name__ == "__main__":
    # 示例用法
    from armpi_common.robot_arm_controller import RobotArmController
    
    # 创建机械臂控制器（模拟）
    try:
        controller = RobotArmController(device="/dev/ttyUSB0")
        controller.enable_reception(True)
        
        # 创建数据收集器
        collector = create_auto_calibration_workflow(
            controller, 
            camera_source=0,
            save_directory="./test_calibration"
        )
        
        # 生成测试位姿
        test_poses = collector.generate_calibration_poses(5)
        print(f"生成了 {len(test_poses)} 个测试位姿")
        
        # 显示位姿信息
        for i, pose in enumerate(test_poses):
            print(f"位姿 {i+1}: {pose}")
        
    except Exception as e:
        logger.error(f"示例运行失败: {e}")
        print("请确保机械臂已连接并配置正确的设备路径")
