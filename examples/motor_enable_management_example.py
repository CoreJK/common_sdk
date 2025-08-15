#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂电机使能管理示例

此示例演示了如何安全地管理 Armpi-FPV 机械臂的电机使能状态，
这是进行标定或维护操作时的重要安全措施。

主要功能：
1. 检查当前电机使能状态
2. 安全卸载所有电机使能
3. 恢复电机使能状态
4. 批量管理电机使能
5. 安全模式切换

安全注意事项：
- 卸载电机使能后，机械臂将失去驱动力，可能因重力而下垂
- 在进行标定或维护前，务必确保机械臂处于安全位置
- 卸载使能期间，避免外力作用于机械臂
- 操作完成后，及时恢复电机使能

作者: AI Assistant
日期: 2024年
"""

import sys
import os
import time

# 添加项目根目录到路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.armpi_common.robot_arm_controller import RobotArmController
from src.armpi_common._log import logger, set_stream_level

# 设置日志级别
set_stream_level("INFO")


def display_motor_status(controller: RobotArmController):
    """显示当前电机使能状态"""
    print("\n" + "=" * 50)
    print("📊 电机使能状态检查")
    print("=" * 50)
    
    # 获取所有关节状态
    status_result = controller.get_all_joints_load_status()
    
    if not status_result['status']:
        print(f"❌ 获取电机状态失败: {status_result['info']}")
        return False
    
    joint_status = status_result['joint_status']
    
    print("关节使能状态详情:")
    joint_names = ["底座旋转", "大臂俯仰", "小臂俯仰", "腕部俯仰", "腕部旋转", "夹爪"]
    
    for joint_id in range(1, 7):
        if joint_id in joint_status:
            status = joint_status[joint_id]
            status_desc = "🔴 卸载掉电" if status == 0 else "🟢 装载电机"
            joint_name = joint_names[joint_id - 1] if joint_id <= len(joint_names) else f"关节{joint_id}"
            print(f"  关节{joint_id} ({joint_name}): {status_desc}")
        else:
            print(f"  关节{joint_id}: ❓ 状态未知")
    
    # 安全状态检查
    safety_result = controller.check_motors_safety_status()
    if safety_result['status']:
        if safety_result['is_safe_mode']:
            print("\n🔒 当前处于安全模式（主要关节已卸载使能）")
        else:
            print("\n⚡ 当前处于工作模式（电机使能正常）")
        
        enabled_count = len(safety_result['enabled_joints'])
        disabled_count = len(safety_result['disabled_joints'])
        print(f"📈 统计: {enabled_count}个关节装载电机, {disabled_count}个关节卸载掉电")
    
    return True


def demonstrate_unload_motors(controller: RobotArmController):
    """演示卸载电机使能"""
    print("\n" + "=" * 50)
    print("🔴 演示：卸载电机使能（进入安全模式）")
    print("=" * 50)
    
    print("⚠️  警告：即将卸载所有电机使能")
    print("   - 机械臂将失去驱动力")
    print("   - 请确保机械臂处于安全位置")
    print("   - 避免外力作用于机械臂")
    
    confirm = input("\n确认执行？(y/N): ").strip().lower()
    if confirm != 'y':
        print("操作已取消")
        return False
    
    # 执行卸载操作
    result = controller.unload_all_motors(include_gripper=False)
    
    if result['status']:
        print("✅ 电机使能卸载成功")
        print("🔒 机械臂已进入安全模式")
        
        # 显示新状态
        time.sleep(1)
        display_motor_status(controller)
        return True
    else:
        print(f"❌ 电机使能卸载失败: {result['info']}")
        return False


def demonstrate_reload_motors(controller: RobotArmController):
    """演示恢复电机使能"""
    print("\n" + "=" * 50)
    print("🟢 演示：恢复电机使能（退出安全模式）")
    print("=" * 50)
    
    print("正在恢复电机使能...")
    
    # 执行恢复操作
    result = controller.reload_all_motors(include_gripper=False, restore_previous=True)
    
    if result['status']:
        print("✅ 电机使能恢复成功")
        print("⚡ 机械臂已恢复工作模式")
        
        # 显示新状态
        time.sleep(1)
        display_motor_status(controller)
        return True
    else:
        print(f"❌ 电机使能恢复失败: {result['info']}")
        return False


def demonstrate_single_joint_control(controller: RobotArmController):
    """演示单个关节使能控制"""
    print("\n" + "=" * 50)
    print("🎛️  演示：单个关节使能控制")
    print("=" * 50)
    
    print("可用关节:")
    joint_names = ["底座旋转", "大臂俯仰", "小臂俯仰", "腕部俯仰", "腕部旋转", "夹爪"]
    for i, name in enumerate(joint_names, 1):
        print(f"  {i}. {name}")
    
    try:
        joint_id = int(input("请选择要控制的关节 (1-6): "))
        if joint_id < 1 or joint_id > 6:
            print("无效的关节ID")
            return False
        
        print(f"当前操作关节: {joint_names[joint_id-1]}")
        
        # 获取当前状态
        current_status = controller.get_joint_load_or_unload(joint_id)
        if current_status.get('load_or_unload') is not None:
            current_state = current_status['load_or_unload']
            state_desc = "卸载掉电" if current_state == 0 else "装载电机"
            print(f"当前状态: {state_desc}")
            
            # 切换状态
            new_state = 1 - current_state  # 0->1, 1->0
            new_desc = "卸载掉电" if new_state == 0 else "装载电机"
            
            confirm = input(f"是否切换到 {new_desc} 状态？(y/N): ").strip().lower()
            if confirm == 'y':
                controller.set_joint_load_or_unload(joint_id, new_state)
                time.sleep(0.5)
                
                # 验证状态
                verify_status = controller.get_joint_load_or_unload(joint_id)
                if verify_status.get('load_or_unload') == new_state:
                    print(f"✅ 关节{joint_id}状态已切换为: {new_desc}")
                else:
                    print(f"❌ 状态切换可能失败，请检查")
                
                return True
            else:
                print("操作已取消")
                return False
        else:
            print(f"❌ 无法获取关节{joint_id}状态")
            return False
            
    except ValueError:
        print("输入格式错误")
        return False


def demonstrate_batch_control(controller: RobotArmController):
    """演示批量电机控制"""
    print("\n" + "=" * 50)
    print("🔄 演示：批量电机使能控制")
    print("=" * 50)
    
    print("选择操作:")
    print("1. 批量装载所有关节（不包括夹爪）")
    print("2. 批量卸载所有关节（不包括夹爪）")
    print("3. 批量装载所有关节（包括夹爪）")
    print("4. 批量卸载所有关节（包括夹爪）")
    
    try:
        choice = int(input("请选择操作 (1-4): "))
        
        operations = {
            1: (1, False, "批量装载所有关节（不包括夹爪）"),
            2: (0, False, "批量卸载所有关节（不包括夹爪）"),
            3: (1, True, "批量装载所有关节（包括夹爪）"),
            4: (0, True, "批量卸载所有关节（包括夹爪）")
        }
        
        if choice not in operations:
            print("无效选择")
            return False
        
        load_state, include_gripper, description = operations[choice]
        
        print(f"即将执行: {description}")
        confirm = input("确认执行？(y/N): ").strip().lower()
        
        if confirm == 'y':
            result = controller.set_all_joints_load_status(load_state, include_gripper)
            
            if result['status']:
                print(f"✅ {description} 成功")
                time.sleep(1)
                display_motor_status(controller)
                return True
            else:
                print(f"❌ 操作失败: {result['info']}")
                return False
        else:
            print("操作已取消")
            return False
            
    except ValueError:
        print("输入格式错误")
        return False


def main():
    """主函数"""
    print("🤖 Armpi-FPV 电机使能管理演示")
    print("=" * 60)
    print("此程序演示如何安全地管理机械臂电机使能状态")
    print("⚠️  重要：电机卸载后机械臂将失去驱动力，请确保安全操作")
    print()
    
    # 设备连接
    device = "/dev/ttyUSB0"
    try:
        user_device = input(f"请输入机械臂设备路径 (默认: {device}): ").strip()
        if user_device:
            device = user_device
    except:
        pass
    
    # 连接机械臂
    print(f"\n正在连接机械臂: {device}")
    try:
        controller = RobotArmController(device=device)
        controller.enable_reception(True)
        print("✅ 机械臂连接成功")
    except Exception as e:
        print(f"❌ 机械臂连接失败: {e}")
        print("请检查设备连接和权限设置")
        return
    
    try:
        while True:
            # 显示主菜单
            print("\n" + "=" * 60)
            print("📋 电机使能管理菜单")
            print("=" * 60)
            print("1. 📊 检查电机状态")
            print("2. 🔴 卸载所有电机使能（安全模式）")
            print("3. 🟢 恢复所有电机使能（工作模式）")
            print("4. 🎛️  单个关节使能控制")
            print("5. 🔄 批量电机使能控制")
            print("6. ❓ 显示帮助信息")
            print("0. 🚪 退出程序")
            
            try:
                choice = input("\n请选择操作 (0-6): ").strip()
                
                if choice == '0':
                    print("正在安全退出...")
                    break
                elif choice == '1':
                    display_motor_status(controller)
                elif choice == '2':
                    demonstrate_unload_motors(controller)
                elif choice == '3':
                    demonstrate_reload_motors(controller)
                elif choice == '4':
                    demonstrate_single_joint_control(controller)
                elif choice == '5':
                    demonstrate_batch_control(controller)
                elif choice == '6':
                    print("\n📖 帮助信息:")
                    print("   - 卸载掉电：电机失去驱动力，用于标定或维护")
                    print("   - 装载电机：电机正常工作，可以控制运动")
                    print("   - 安全模式：主要关节(1-5)卸载掉电")
                    print("   - 工作模式：电机装载正常")
                    print("   - 标定前建议进入安全模式")
                else:
                    print("无效选择，请重新输入")
                    
            except KeyboardInterrupt:
                print("\n\n用户中断，正在退出...")
                break
            except Exception as e:
                print(f"操作发生异常: {e}")
                
    except KeyboardInterrupt:
        print("\n\n用户中断，正在清理...")
    
    finally:
        # 清理资源
        try:
            # 检查是否处于安全模式
            safety_status = controller.check_motors_safety_status()
            if safety_status['status'] and safety_status['is_safe_mode']:
                print("\n⚠️  检测到机械臂处于安全模式")
                restore = input("是否恢复到工作模式后退出？(Y/n): ").strip().lower()
                if restore != 'n':
                    print("正在装载电机...")
                    result = controller.reload_all_motors(include_gripper=False, restore_previous=False)
                    if result['status']:
                        print("✅ 电机已装载恢复")
                    else:
                        print(f"❌ 恢复失败: {result['info']}")
            
            controller.close_connection()
            print("✅ 机械臂连接已关闭")
        except:
            pass
        
        print("程序结束")


if __name__ == "__main__":
    main()
