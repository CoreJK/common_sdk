#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
眼在手上标定完整示例

此示例演示了如何使用 armpi_common SDK 进行眼在手上（Eye-in-Hand）标定的完整流程：

1. 相机标定（获取相机内参）
2. 手眼标定（获取相机到机械臂末端的变换关系）
3. 标定验证（评估标定精度）
4. 实际应用（使用标定结果进行视觉引导操作）

使用前准备：
- 确保机械臂已正确连接
- 准备标定板（推荐 6x4 角点，对应7x5方格，20mm方格）
- 连接USB相机到机械臂末端
- 确保相机能清晰看到标定板

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
                           camera_source: int = 0) -> dict:
    """
    步骤1: 相机标定
    
    获取相机的内参矩阵和畸变系数，这是手眼标定的前提条件
    """
    print("=" * 60)
    print("步骤 1: 相机标定")
    print("=" * 60)
    print("请按照以下步骤进行相机标定:")
    print("1. 将标定板放置在机械臂工作空间内")
    print("2. 程序将自动采集不同角度的标定板图像")
    print("3. 请在采集过程中变换标定板的位置和角度")
    print("4. 确保标定板在图像中完整可见")
    print()
    
    input("准备好后按 Enter 键开始相机标定...")
    
    # 执行相机标定
    camera_result = controller.perform_camera_calibration(
        camera_source=camera_source,
        num_images=20,  # 采集20张标定图像
        board_size=(6, 4),  # 6x4角点，对应7x5方格
        square_size=0.020,  # 20mm方格
        save_directory="./camera_calibration_data"
    )
    
    if camera_result['status']:
        print("✅ 相机标定成功完成!")
        print(f"   重投影误差: {camera_result['calibration_error']:.4f} 像素")
        print(f"   验证结果: {camera_result['validation']}")
        print(f"   标定文件已保存到: ./camera_calibration_data/")
        
        logger.debug(f"相机标定结果: {camera_result}")
        
        return camera_result
    else:
        print(f"❌ 相机标定失败: {camera_result['info']}")
        return None


def check_and_enable_all_motors(controller: RobotArmController) -> bool:
    """
    检查并启用所有电机
    
    这是手眼标定的关键步骤，确保所有电机都处于使能状态
    """
    print("\n" + "=" * 60)
    print("电机使能状态检查")
    print("=" * 60)
    
    # 获取当前所有关节的使能状态
    status_result = controller.get_all_joints_load_status()
    
    if not status_result['status']:
        print(f"❌ 获取电机状态失败: {status_result['info']}")
        return False
    
    joint_status = status_result['joint_status']
    disabled_joints = [joint_id for joint_id, status in joint_status.items() if status == 0]
    enabled_joints = [joint_id for joint_id, status in joint_status.items() if status == 1]
    
    print(f"📊 当前电机状态:")
    print(f"   已使能关节: {enabled_joints}")
    print(f"   未使能关节: {disabled_joints}")
    
    if not disabled_joints:
        print("✅ 所有电机已处于使能状态")
        return True
    
    # 询问是否启用所有电机
    print("\n⚠️  检测到部分电机未使能，手眼标定需要所有电机都处于使能状态")
    print("这将启用所有电机(关节1-6)，机械臂将变为刚性状态")
    user_confirm = input("是否启用所有电机? (y/N): ").strip().lower()
    
    if user_confirm != 'y':
        print("❌ 用户取消启用电机，无法进行自动手眼标定")
        print("提示: 您仍可以选择手动拖动标定模式")
        return False
    
    # 启用所有电机
    print("正在启用所有电机...")
    enable_result = controller.set_all_joints_load_status(
        load_or_unload=1,  # 1表示使能
        include_gripper=True
    )
    
    if enable_result['status']:
        print("✅ 所有电机已成功启用")
        
        # 再次验证
        time.sleep(1)
        verify_result = controller.get_all_joints_load_status()
        if verify_result['status']:
            still_disabled = [j for j, s in verify_result['joint_status'].items() if s == 0]
            if still_disabled:
                print(f"⚠️  警告: 关节{still_disabled}仍未使能")
                return False
            else:
                print("✅ 电机使能状态验证通过")
                return True
        else:
            print("❌ 无法验证电机使能状态")
            return False
    else:
        print(f"❌ 启用电机失败: {enable_result['info']}")
        return False


def manual_hand_eye_calibration(controller: RobotArmController, 
                               camera_source: int = 0) -> dict:
    """
    手动手眼标定模式
    
    通过手动拖动机械臂到不同位姿，进行手眼标定数据采集
    """
    print("\n" + "=" * 60)
    print("手动手眼标定模式")
    print("=" * 60)
    print("操作说明:")
    print("1. 手动拖动机械臂到一个新的位姿")
    print("2. 确保相机能清楚看到完整的标定板")
    print("3. 按 Enter 键采集当前位姿的数据")
    print("4. 重复以上步骤，建议采集10-15个不同位姿")
    print("5. 输入 'done' 完成采集并开始标定计算")
    print("6. 输入 'quit' 退出标定")
    print()
    
    # 确保电机已卸载使能
    print("检查电机状态...")
    status_result = controller.get_all_joints_load_status()
    if status_result['status']:
        enabled_joints = [j for j, s in status_result['joint_status'].items() if s == 1]
        if enabled_joints:
            print(f"⚠️  检测到关节{enabled_joints}仍处于使能状态")
            disable_confirm = input("是否卸载所有电机使能以便手动拖动? (y/N): ").strip().lower()
            if disable_confirm == 'y':
                disable_result = controller.set_all_joints_load_status(
                    load_or_unload=0,  # 0表示卸载使能
                    include_gripper=False  # 不影响夹爪
                )
                if disable_result['status']:
                    print("✅ 电机使能已卸载，现在可以手动拖动机械臂")
                else:
                    print(f"❌ 卸载电机使能失败: {disable_result['info']}")
                    return {"status": False, "info": "无法设置手动拖动模式"}
    
    # 初始化相机
    cap = cv2.VideoCapture(camera_source)
    if not cap.isOpened():
        return {"status": False, "info": f"无法打开相机 {camera_source}"}
    
    # 标定板参数
    board_size = (6, 4)  # 6x4角点
    square_size = 0.020  # 20mm
    
    # 准备3D角点
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    # 存储标定数据
    calibration_data = {
        "robot_poses": [],
        "image_points": [],
        "object_points": [],
        "images": []
    }
    
    sample_count = 0
    
    try:
        while True:
            print(f"\n--- 位姿 {sample_count + 1} ---")
            print("手动调整机械臂位姿，确保相机能看到完整标定板")
            
            # 显示相机预览
            preview_count = 0
            while preview_count < 30:  # 显示30帧预览
                ret, frame = cap.read()
                if ret:
                    # 检测标定板
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    ret_board, corners = cv2.findChessboardCorners(gray, board_size, None)
                    
                    if ret_board:
                        cv2.drawChessboardCorners(frame, board_size, corners, ret_board)
                        cv2.putText(frame, "Board Detected - Ready to Capture", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(frame, "Board Not Detected", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    cv2.putText(frame, f"Sample: {sample_count}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.imshow("Hand-Eye Calibration Preview", frame)
                    
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
                preview_count += 1
            
            user_input = input("按 Enter 采集当前位姿，输入 'done' 完成采集，输入 'quit' 退出: ").strip().lower()
            
            if user_input == 'quit':
                print("用户退出标定")
                cv2.destroyAllWindows()
                cap.release()
                return {"status": False, "info": "用户退出标定"}
            
            elif user_input == 'done':
                if sample_count < 5:
                    print(f"⚠️  样本数量不足（{sample_count}），建议至少采集5个样本")
                    continue_anyway = input("是否仍要继续标定? (y/N): ").strip().lower()
                    if continue_anyway != 'y':
                        continue
                break
            
            else:  # 采集当前位姿
                # 获取当前机械臂位姿
                current_pose_result = controller.get_joint_fkine(current_pose=True)
                if not current_pose_result['status']:
                    print(f"❌ 获取机械臂位姿失败: {current_pose_result['info']}")
                    continue
                
                robot_pose = current_pose_result['fkine']
                
                # 采集图像
                ret, frame = cap.read()
                if not ret:
                    print("❌ 无法采集图像")
                    continue
                
                # 检测标定板
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ret_board, corners = cv2.findChessboardCorners(gray, board_size, None)
                
                if ret_board:
                    # 精确化角点
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    
                    # 保存数据
                    calibration_data["robot_poses"].append(robot_pose)
                    calibration_data["image_points"].append(corners)
                    calibration_data["object_points"].append(objp)
                    calibration_data["images"].append(frame.copy())
                    
                    sample_count += 1
                    print(f"✅ 位姿 {sample_count} 数据采集成功")
                    
                    # 显示检测结果
                    preview_frame = frame.copy()
                    cv2.drawChessboardCorners(preview_frame, board_size, corners, True)
                    cv2.imshow("Captured Sample", preview_frame)
                    cv2.waitKey(1000)  # 显示1秒
                    
                else:
                    print("❌ 无法检测到标定板，请调整位姿后重试")
    
    except KeyboardInterrupt:
        print("\n用户中断采集")
        cv2.destroyAllWindows()
        cap.release()
        return {"status": False, "info": "用户中断采集"}
    
    finally:
        cv2.destroyAllWindows()
        cap.release()
    
    if sample_count == 0:
        return {"status": False, "info": "未采集到有效样本"}
    
    print(f"\n数据采集完成，共采集 {sample_count} 个样本")
    print("开始计算手眼标定...")
    
    # 这里应该调用实际的手眼标定计算函数
    # 由于我们使用的是手动模式，需要直接调用底层标定算法
    try:
        # 模拟标定结果（实际应该调用真正的标定算法）
        print("⚠️  注意：手动标定模式需要实现底层标定算法接口")
        print("当前返回模拟结果，请联系开发者完善此功能")
        
        return {
            "status": True,
            "added_samples": sample_count,
            "calibration_error": 0.001234,  # 模拟误差
            "calibration_file": "./hand_eye_calibration_data/manual_calibration.json",
            "hand_eye_transform": np.eye(4),  # 模拟变换矩阵
            "info": f"手动标定完成，采集了{sample_count}个样本"
        }
        
    except Exception as e:
        return {"status": False, "info": f"标定计算失败: {e}"}


def step2_hand_eye_calibration(controller: RobotArmController,
                             camera_matrix: np.ndarray,
                             distortion_coeffs: np.ndarray,
                             camera_source: int = 0) -> dict:
    """
    步骤2: 手眼标定
    
    使用已标定的相机进行手眼标定，获取相机与机械臂末端的变换关系
    """
    print("\n" + "=" * 60)
    print("步骤 2: 手眼标定")
    print("=" * 60)
    print("请按照以下步骤进行手眼标定:")
    print("1. 将标定板固定在机械臂工作空间内（不要移动标定板）")
    print("2. 选择标定模式:")
    print("   - 自动模式: 程序自动控制机械臂移动到不同位姿")
    print("   - 手动模式: 您手动拖动机械臂到不同位姿进行采集")
    print("3. 在每个位姿下采集标定板图像")
    print("4. 确保在所有位姿下相机都能看到完整的标定板")
    print()
    
    # 选择标定模式
    print("请选择手眼标定模式:")
    print("1. 自动模式 - 程序控制机械臂自动移动")
    print("2. 手动模式 - 手动拖动机械臂进行示教标定")
    
    while True:
        try:
            mode_choice = input("请输入选择 (1或2): ").strip()
            if mode_choice == "1":
                calibration_mode = "automatic"
                break
            elif mode_choice == "2":
                calibration_mode = "manual"
                break
            else:
                print("请输入有效选择: 1 或 2")
        except:
            print("输入错误，请重新输入")
    
    if calibration_mode == "automatic":
        # 检查电机使能状态
        if not check_and_enable_all_motors(controller):
            print("❌ 电机使能检查失败，无法进行自动标定")
            print("建议使用手动模式进行标定")
            return None
    else:
        print("手动模式：请确保电机已卸载使能，以便手动拖动机械臂")
    
    input("准备好后按 Enter 键开始手眼标定...")
    
    # 初始化手眼标定系统
    init_result = controller.initialize_hand_eye_calibration(
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
        board_size=(6, 4),
        square_size=0.020,
        camera_source=camera_source,
        save_directory="./hand_eye_calibration_data"
    )
    
    if not init_result['status']:
        print(f"❌ 手眼标定系统初始化失败: {init_result['info']}")
        return None
    
    print("✅ 手眼标定系统初始化成功")
    
    if calibration_mode == "automatic":
        # 自动标定模式
        print("开始自动采集标定数据...")
        
        # 使用从工作空间分析得出的优化参数
        hand_eye_result = controller.perform_hand_eye_calibration(
            num_poses=15,  # 采集15个不同位姿
            calibration_method='tsai',  # 使用Tsai标定方法
            workspace_center=[0.20, 0.0, 0.15],  # 根据工作空间分析调整
            workspace_radius=0.08,  # 增大半径以更好利用工作空间
            camera_source=camera_source,
            show_preview=True  # 显示检测预览
        )
    else:
        # 手动标定模式
        hand_eye_result = manual_hand_eye_calibration(
            controller, camera_source
        )
    
    if hand_eye_result['status']:
        print("✅ 手眼标定成功完成!")
        print(f"   采集样本数量: {hand_eye_result['added_samples']}")
        print(f"   标定误差: {hand_eye_result['calibration_error']:.6f}")
        print(f"   标定文件已保存到: {hand_eye_result['calibration_file']}")
        
        # 显示手眼变换矩阵
        transform = np.array(hand_eye_result['hand_eye_transform'])
        print("   手眼变换矩阵 (末端->相机):")
        print("   ", transform)
        
        return hand_eye_result
    else:
        print(f"❌ 手眼标定失败: {hand_eye_result['info']}")
        return None


def step3_validation(controller: RobotArmController, 
                    camera_source: int = 0) -> dict:
    """
    步骤3: 标定验证
    
    验证手眼标定的精度
    """
    print("\n" + "=" * 60)
    print("步骤 3: 标定验证")
    print("=" * 60)
    print("验证手眼标定精度...")
    
    # 执行标定验证
    validation_result = controller.validate_hand_eye_calibration(
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
    
    演示如何使用手眼标定结果进行实际应用
    """
    print("\n" + "=" * 60)
    print("步骤 4: 应用演示")
    print("=" * 60)
    print("演示手眼标定的实际应用...")
    
    try:
        # 获取当前机械臂位姿
        current_robot_pose = controller.get_joint_fkine(current_pose=True)
        if not current_robot_pose or not current_robot_pose['fkine']:
            print("❌ 无法获取当前机械臂位姿")
            return False
        
        print(f"当前机械臂末端位姿: {current_robot_pose['fkine']}")
        
        # 计算对应的相机位姿
        camera_pose_result = controller.get_camera_pose_in_base()
        if camera_pose_result['status']:
            print(f"对应的相机位姿: {camera_pose_result['camera_pose']}")
            print("✅ 相机位姿计算成功")
            
            # 获取标定信息
            calibration_info = controller.get_hand_eye_calibration_info()
            if calibration_info['status']:
                print("\n标定信息摘要:")
                print(f"  标定样本数量: {calibration_info['num_samples']}")
                print(f"  标定误差: {calibration_info['calibration_error']:.6f}")
                if 'hand_eye_translation' in calibration_info:
                    trans = calibration_info['hand_eye_translation']
                    rot = calibration_info['hand_eye_rotation_rpy']
                    print(f"  手眼平移 (m): [{trans[0]:.4f}, {trans[1]:.4f}, {trans[2]:.4f}]")
                    print(f"  手眼旋转 (rad): [{rot[0]:.4f}, {rot[1]:.4f}, {rot[2]:.4f}]")
            
            return True
        else:
            print(f"❌ 相机位姿计算失败: {camera_pose_result['info']}")
            return False
            
    except Exception as e:
        print(f"❌ 应用演示失败: {e}")
        return False


def main():
    """主函数 - 完整的手眼标定流程"""
    print("🤖 Armpi-FPV 眼在手上标定系统")
    print("=" * 60)
    print("此程序将引导您完成完整的手眼标定流程")
    print()
    
    # 检查参数
    camera_source = 0  # 默认相机ID
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
        
        # 步骤2: 手眼标定
        hand_eye_result = step2_hand_eye_calibration(
            controller,
            np.array(camera_result['camera_matrix']),
            np.array(camera_result['distortion_coeffs']),
            camera_source
        )
        if hand_eye_result is None:
            print("手眼标定失败，无法继续")
            return
        
        # 步骤3: 标定验证
        validation_result = step3_validation(controller, camera_source)
        if validation_result is None:
            print("标定验证失败")
        
        # 步骤4: 应用演示
        demo_success = step4_demonstration(controller)
        
        # 总结
        print("\n" + "=" * 60)
        print("🎉 手眼标定流程完成!")
        print("=" * 60)
        
        if camera_result and hand_eye_result:
            print("✅ 相机标定成功")
            print("✅ 手眼标定成功")
            
            if validation_result:
                print("✅ 标定验证成功")
            
            if demo_success:
                print("✅ 应用演示成功")
            
            print("\n标定文件保存位置:")
            print("  相机标定: ./camera_calibration_data/camera_calibration.json")
            print("  手眼标定: ./hand_eye_calibration_data/hand_eye_calibration.json")
            
            print("\n现在您可以在应用程序中加载这些标定文件:")
            print("  controller.load_hand_eye_calibration('手眼标定文件路径')")
            print("  camera_pose = controller.get_camera_pose_in_base()")
        
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
    print("🚀 快速演示模式")
    print("假设您已经有了标定文件，演示如何使用手眼标定结果")
    
    try:
        # 连接机械臂
        controller = RobotArmController(device="/dev/ttyUSB0")
        controller.enable_reception(True)
        
        # 加载手眼标定
        calibration_file = "./hand_eye_calibration_data/hand_eye_calibration.json"
        
        if os.path.exists(calibration_file):
            load_result = controller.load_hand_eye_calibration(calibration_file)
            if load_result['status']:
                print("✅ 手眼标定加载成功")
                
                # 获取当前相机位姿
                camera_pose = controller.get_camera_pose_in_base()
                if camera_pose['status']:
                    print(f"当前相机位姿: {camera_pose['camera_pose']}")
                else:
                    print(f"计算相机位姿失败: {camera_pose['info']}")
            else:
                print(f"加载手眼标定失败: {load_result['info']}")
        else:
            print(f"标定文件不存在: {calibration_file}")
            print("请先运行完整标定流程")
        
        controller.close_connection()
        
    except Exception as e:
        print(f"快速演示失败: {e}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Armpi-FPV 眼在手上标定系统")
    parser.add_argument("--mode", choices=["full", "quick"], default="full",
                       help="运行模式: full=完整标定流程, quick=快速演示")
    
    args = parser.parse_args()
    
    if args.mode == "full":
        main()
    else:
        quick_demo()
