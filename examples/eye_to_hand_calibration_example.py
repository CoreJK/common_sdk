#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手外标定完整示例

此示例演示了如何使用 armpi_common SDK 进行眼在手外（Eye-to-Hand）标定的完整流程：

1. 相机标定（获取相机内参）
2. 眼在手外标定（获取相机到机械臂基座的变换关系）
3. 标定验证（评估标定精度）
4. 实际应用（使用标定结果进行视觉引导操作）

使用前准备：
- 确保机械臂已正确连接
- 准备标定板（推荐 9x6 棋盘格，25mm方格）
- 将标定板安装在机械臂末端
- 将相机固定在外部位置（如桌面支架或天花板）
- 确保相机能清晰看到机械臂末端的标定板

配置说明：
眼在手外（Eye-to-Hand）配置中：
- 相机位置固定，不随机械臂运动
- 标定板安装在机械臂末端，随机械臂运动
- 标定目标：求解相机相对于机械臂基座的变换关系

作者: AI Assistant
日期: 2024年
"""

import sys
import os
import time
import numpy as np
import cv2

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.armpi_common.robot_arm_controller import RobotArmController
from src.armpi_common._log import logger, set_stream_level

# 设置日志级别
set_stream_level("INFO")


def step1_camera_calibration(controller: RobotArmController, 
                           camera_source: int = 2) -> dict:
    """
    步骤1: 相机标定
    
    获取相机的内参矩阵和畸变系数，这是眼在手外标定的前提条件
    """
    print("=" * 60)
    print("步骤 1: 相机标定")
    print("=" * 60)
    print("请按照以下步骤进行相机标定:")
    print("1. 确保相机已固定在外部位置")
    print("2. 手动移动标定板到不同位置和角度（不要移动相机）")
    print("3. 程序将自动采集不同角度的标定板图像")
    print("4. 确保标定板在图像中完整可见")
    print()
    
    input("准备好后按 Enter 键开始相机标定...")
    
    # 执行相机标定
    camera_result = controller.perform_camera_calibration(
        camera_source=camera_source,
        num_images=20,  # 采集20张标定图像
        board_size=(9, 6),  # 9x6标定板
        square_size=0.025,  # 25mm方格
        save_directory="./camera_calibration_data"
    )
    
    if camera_result['status']:
        print("✅ 相机标定成功完成!")
        print(f"   重投影误差: {camera_result['calibration_error']:.4f} 像素")
        print(f"   验证结果: {camera_result['validation']}")
        print(f"   标定文件已保存到: ./camera_calibration_data/")
        
        return camera_result
    else:
        print(f"❌ 相机标定失败: {camera_result['info']}")
        return None


def step2_eye_to_hand_calibration(controller: RobotArmController,
                                camera_matrix: np.ndarray,
                                distortion_coeffs: np.ndarray,
                                camera_source: int = 2,
                                camera_pose: list = None) -> dict:
    """
    步骤2: 眼在手外标定
    
    使用已标定的相机进行眼在手外标定，获取相机与机械臂基座的变换关系
    """
    print("\n" + "=" * 60)
    print("步骤 2: 眼在手外标定")
    print("=" * 60)
    print("请按照以下步骤进行眼在手外标定:")
    print("1. 确保标定板已牢固安装在机械臂末端")
    print("2. 确保相机位置固定且能看到机械臂工作区域")
    print("3. 程序将自动控制机械臂移动到不同位姿")
    print("4. 在每个位姿下采集标定板图像")
    print("5. 确保在所有位姿下相机都能看到完整的标定板")
    print()
    
    # 询问相机位姿信息
    if camera_pose is None:
        print("提示：如果已知相机在基座坐标系中的大致位置，可以提供该信息以优化位姿生成")
        try:
            user_input = input("请输入相机位置 [x,y,z] (米)，格式如 '0.0,-0.3,0.4'，或直接按 Enter 跳过: ").strip()
            if user_input:
                coords = [float(x.strip()) for x in user_input.split(',')]
                if len(coords) == 3:
                    camera_pose = coords + [0.0, 0.0, 0.0]  # 添加默认姿态
                    print(f"使用相机位置: {coords}")
                else:
                    print("格式错误，使用默认配置")
        except:
            print("输入错误，使用默认配置")
    
    input("准备好后按 Enter 键开始眼在手外标定...")
    
    # 初始化眼在手外标定系统
    init_result = controller.initialize_eye_to_hand_calibration(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        board_size=(9, 6),
        square_size=0.025,
        camera_source=camera_source,
        camera_pose=camera_pose,
        save_directory="./eye_to_hand_calibration_data"
    )
    
    if not init_result['status']:
        print(f"❌ 眼在手外标定系统初始化失败: {init_result['info']}")
        return None
    
    print("✅ 眼在手外标定系统初始化成功")
    
    # 执行眼在手外标定
    print("开始自动采集标定数据...")
    
    eye_to_hand_result = controller.perform_eye_to_hand_calibration(
        num_poses=15,  # 采集15个不同位姿
        calibration_method='tsai',  # 使用Tsai标定方法
        workspace_center=[0.15, 0.0, 0.20],  # 工作空间中心
        workspace_radius=0.05,  # 工作空间半径
        camera_source=camera_source,
        show_preview=True  # 显示检测预览
    )
    
    if eye_to_hand_result['status']:
        print("✅ 眼在手外标定成功完成!")
        print(f"   采集样本数量: {eye_to_hand_result['added_samples']}")
        print(f"   标定误差: {eye_to_hand_result['calibration_error']:.6f}")
        print(f"   标定文件已保存到: {eye_to_hand_result['calibration_file']}")
        
        # 显示眼在手外变换矩阵
        transform = np.array(eye_to_hand_result['eye_to_hand_transform'])
        print("   眼在手外变换矩阵 (基座->相机):")
        print("   ", transform)
        
        return eye_to_hand_result
    else:
        print(f"❌ 眼在手外标定失败: {eye_to_hand_result['info']}")
        return None


def step3_validation(controller: RobotArmController, 
                    camera_source: int = 2) -> dict:
    """
    步骤3: 标定验证
    
    验证眼在手外标定的精度
    """
    print("\n" + "=" * 60)
    print("步骤 3: 标定验证")
    print("=" * 60)
    print("验证眼在手外标定精度...")
    
    # 执行标定验证
    validation_result = controller.validate_eye_to_hand_calibration(
        test_poses=None,  # 自动生成测试位姿
        camera_source=camera_source,
        num_test_poses=5  # 5个测试位姿
    )
    
    if validation_result['status']:
        print("✅ 标定验证完成!")
        print(f"   验证成功率: {validation_result['success_rate']:.1%}")
        print(f"   平均平移误差: {validation_result['mean_translation_error_mm']:.2f} mm")
        print(f"   平移误差标准差: {validation_result['std_translation_error_mm']:.2f} mm")
        print(f"   平均旋转误差: {validation_result['mean_rotation_error_deg']:.2f} °")
        print(f"   旋转误差标准差: {validation_result['std_rotation_error_deg']:.2f} °")
        
        # 评估标定质量
        avg_trans_error = validation_result['mean_translation_error_mm']
        avg_rot_error = validation_result['mean_rotation_error_deg']
        
        if avg_trans_error < 2.0 and avg_rot_error < 2.0:
            print("🎉 标定质量: 优秀")
        elif avg_trans_error < 5.0 and avg_rot_error < 5.0:
            print("👍 标定质量: 良好")
        elif avg_trans_error < 10.0 and avg_rot_error < 10.0:
            print("⚠️  标定质量: 一般，建议重新标定")
        else:
            print("❌ 标定质量: 较差，请重新标定")
        
        return validation_result
    else:
        print(f"❌ 标定验证失败: {validation_result['info']}")
        return None


def step4_demonstration(controller: RobotArmController) -> bool:
    """
    步骤4: 应用演示
    
    演示如何使用眼在手外标定结果进行实际应用
    """
    print("\n" + "=" * 60)
    print("步骤 4: 应用演示")
    print("=" * 60)
    print("演示眼在手外标定的实际应用...")
    
    try:
        # 获取固定相机的位姿
        camera_pose_result = controller.get_fixed_camera_pose()
        if camera_pose_result['status']:
            camera_pose = camera_pose_result['camera_pose']
            print(f"固定相机位姿: {camera_pose}")
            print("✅ 相机位姿计算成功")
        else:
            print(f"❌ 相机位姿计算失败: {camera_pose_result['info']}")
            return False
        
        # 获取当前机械臂位姿对应的标定板位姿
        board_pose_result = controller.get_board_pose_from_robot_pose()
        if board_pose_result['status']:
            board_pose = board_pose_result['board_pose']
            print(f"当前标定板位姿: {board_pose}")
            print("✅ 标定板位姿计算成功")
        else:
            print(f"❌ 标定板位姿计算失败: {board_pose_result['info']}")
            return False
        
        # 获取标定信息
        calibration_info = controller.get_eye_to_hand_calibration_info()
        if calibration_info['status']:
            print("\n眼在手外标定信息摘要:")
            print(f"  标定类型: {calibration_info['calibration_type']}")
            print(f"  标定样本数量: {calibration_info['num_samples']}")
            print(f"  标定误差: {calibration_info['calibration_error']:.6f}")
            if 'eye_to_hand_translation' in calibration_info:
                trans = calibration_info['eye_to_hand_translation']
                rot = calibration_info['eye_to_hand_rotation_rpy']
                print(f"  眼在手外平移 (m): [{trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f}]")
                print(f"  眼在手外旋转 (rad): [{rot[0]:.4f}, {rot[1]:.4f}, {rot[2]:.4f}]")
        
        # 演示实际应用场景
        print("\n实际应用演示:")
        print("1. 视觉定位: 通过相机观测确定目标物体位置")
        print("2. 路径规划: 根据相机坐标系中的目标位置规划机械臂运动")
        print("3. 抓取操作: 控制机械臂末端的标定板(或夹爪)到达目标位置")
        
        # 演示坐标变换
        print("\n坐标变换演示:")
        
        # 假设在相机坐标系中检测到目标
        target_in_camera = [0.1, 0.05, 0.3]  # 相机坐标系中的目标位置
        print(f"相机坐标系中的目标: {target_in_camera}")
        
        # 转换到基座坐标系（需要相机到基座的变换）
        camera_to_base_transform = np.array(camera_pose_result['camera_transform_matrix'])
        target_in_camera_homo = np.append(target_in_camera, 1.0)
        target_in_base_homo = camera_to_base_transform @ target_in_camera_homo
        target_in_base = target_in_base_homo[:3]
        
        print(f"基座坐标系中的目标: [{target_in_base[0]:.4f}, {target_in_base[1]:.4f}, {target_in_base[2]:.4f}]")
        print("✅ 坐标变换演示成功")
        
        return True
        
    except Exception as e:
        print(f"❌ 应用演示失败: {e}")
        return False


def main():
    """主函数 - 完整的眼在手外标定流程"""
    print("🤖 Armpi-FPV 眼在手外标定系统")
    print("=" * 60)
    print("此程序将引导您完成完整的眼在手外标定流程")
    print()
    print("配置说明:")
    print("- 相机固定在外部位置（如桌面支架）")
    print("- 标定板安装在机械臂末端")
    print("- 标定目标：求解相机相对于机械臂基座的变换关系")
    print()
    
    # 检查参数
    camera_source = 2  # 默认相机ID
    robot_device = "/dev/ttyUSB0"  # 默认机械臂设备
    
    # 询问用户设置
    try:
        user_camera = input(f"请输入相机设备ID (默认: {camera_source}): ").strip()
        if user_camera:
            camera_source = int(user_camera)
        
        user_device = input(f"请输入机械臂设备路径 (默认: {robot_device}): ").strip()
        if user_device:
            robot_device = user_device
            
    except ValueError:
        print("输入格式错误，使用默认设置")
    
    # 连接机械臂
    print(f"\n正在连接机械臂: {robot_device}")
    try:
        controller = RobotArmController(device=robot_device)
        controller.enable_reception(True)
        print("✅ 机械臂连接成功")
        
        # 检查电机安全状态
        safety_status = controller.check_motors_safety_status()
        if safety_status['status']:
            print(f"📊 电机状态检查: 使能关节{safety_status['enabled_joints']}, 卸载关节{safety_status['disabled_joints']}")
            if safety_status['is_safe_mode']:
                print("⚠️  检测到电机已处于安全模式（已卸载使能）")
            else:
                print("ℹ️  电机当前处于正常工作状态")
        
    except Exception as e:
        print(f"❌ 机械臂连接失败: {e}")
        print("请检查设备连接和权限设置")
        return
    
    try:
        # 步骤1: 相机标定
        camera_result = step1_camera_calibration(controller, camera_source)
        if camera_result is None:
            print("相机标定失败，无法继续")
            return
        
        # 步骤2: 眼在手外标定
        eye_to_hand_result = step2_eye_to_hand_calibration(
            controller,
            np.array(camera_result['camera_matrix']),
            np.array(camera_result['distortion_coeffs']),
            camera_source
        )
        if eye_to_hand_result is None:
            print("眼在手外标定失败，无法继续")
            return
        
        # 步骤3: 标定验证
        validation_result = step3_validation(controller, camera_source)
        if validation_result is None:
            print("标定验证失败")
        
        # 步骤4: 应用演示
        demo_success = step4_demonstration(controller)
        
        # 总结
        print("\n" + "=" * 60)
        print("🎉 眼在手外标定流程完成!")
        print("=" * 60)
        
        if camera_result and eye_to_hand_result:
            print("✅ 相机标定成功")
            print("✅ 眼在手外标定成功")
            
            if validation_result:
                print("✅ 标定验证成功")
            
            if demo_success:
                print("✅ 应用演示成功")
            
            print("\n标定文件保存位置:")
            print("  相机标定: ./camera_calibration_data/camera_calibration.json")
            print("  眼在手外标定: ./eye_to_hand_calibration_data/eye_to_hand_calibration.json")
            
            print("\n现在您可以在应用程序中加载这些标定文件:")
            print("  controller.load_eye_to_hand_calibration('眼在手外标定文件路径')")
            print("  camera_pose = controller.get_fixed_camera_pose()")
            print("  board_pose = controller.get_board_pose_from_robot_pose()")
        
    except KeyboardInterrupt:
        print("\n\n用户中断，正在清理...")
    
    except Exception as e:
        print(f"\n❌ 程序执行出错: {e}")
        logger.error(f"主程序异常: {e}")
    
    finally:
        # 清理资源
        try:
            controller.close_connection()
            print("✅ 机械臂连接已关闭")
        except:
            pass
        
        cv2.destroyAllWindows()
        print("程序结束")


def quick_demo():
    """快速演示 - 假设已有标定文件"""
    print("🚀 快速演示模式 - 眼在手外标定")
    print("假设您已经有了标定文件，演示如何使用眼在手外标定结果")
    
    try:
        # 连接机械臂
        controller = RobotArmController(device="/dev/ttyUSB0")
        controller.enable_reception(True)
        
        # 加载眼在手外标定
        calibration_file = "./eye_to_hand_calibration_data/eye_to_hand_calibration.json"
        
        if os.path.exists(calibration_file):
            load_result = controller.load_eye_to_hand_calibration(calibration_file)
            if load_result['status']:
                print("✅ 眼在手外标定加载成功")
                
                # 获取固定相机位姿
                camera_pose = controller.get_fixed_camera_pose()
                if camera_pose['status']:
                    print(f"固定相机位姿: {camera_pose['camera_pose']}")
                else:
                    print(f"计算相机位姿失败: {camera_pose['info']}")
                
                # 获取当前标定板位姿
                board_pose = controller.get_board_pose_from_robot_pose()
                if board_pose['status']:
                    print(f"当前标定板位姿: {board_pose['board_pose']}")
                else:
                    print(f"计算标定板位姿失败: {board_pose['info']}")
                    
            else:
                print(f"加载眼在手外标定失败: {load_result['info']}")
        else:
            print(f"标定文件不存在: {calibration_file}")
            print("请先运行完整标定流程")
        
        controller.close_connection()
        
    except Exception as e:
        print(f"快速演示失败: {e}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Armpi-FPV 眼在手外标定系统")
    parser.add_argument("--mode", choices=["full", "quick"], default="full",
                       help="运行模式: full=完整标定流程, quick=快速演示")
    
    args = parser.parse_args()
    
    if args.mode == "full":
        main()
    else:
        quick_demo()
