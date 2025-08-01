# -*- coding: utf-8 -*- 
# robot_arm_controller.py - 机械臂控制器

import threading
from queue import Queue
import struct
import time

import serial

from armpi_common.cmdTable import CMD_TABLE
from armpi_common.utils import calculate_checksum, get_command_info_by_id, split_to_bytes

class RobotArmController:
    def __init__(self, device='/dev/ttyUSB0', baudrate=115200, timeout=0):
        self.port = device
        self.baudrate = baudrate
        self.serial_client = serial.Serial(device, baudrate, timeout=timeout)
        self.serial_client.rts = False
        self.serial_client.dtr = False
        
    def bus_write(self, cmd_data):
        """只负责发送已经构造好的数据"""
        # todo 改成线程发送
        self.serial_client.write(cmd_data)
    
    def set_joint_mode(self, joint_id, servo_mode, speed):
        """设置指定关节的舵机工作模式

        :param int joint_id: 关节ID
        :param int servo_mode: 
            0 - 位置控制模式
            1 - 电机控制模式
            默认是 0
        :param int speed: 转动速度值，范围 -1000 ~ 1000，只在电机控制模式时有效
            控制电机的转速，该值为负值代表反转，正值代表正转
            注意：由于转动速度为 signed short int 型数据，需要转换为补码形式
        """
        # 参数验证
        if servo_mode not in [0, 1]:
            raise ValueError('servo_mode must be 0 or 1')
        
        if not -1000 <= speed <= 1000:
            raise ValueError('speed must be between -1000 and 1000')
        
        # 将有符号短整数转换为无符号短整数（补码形式）
        # 使用 struct.pack 将 signed short 转换为 unsigned short
        speed_bytes = struct.pack('<h', speed)  # '<h' 表示 little-endian signed short
        speed_unsigned = struct.unpack('<H', speed_bytes)[0]  # '<H' 表示 little-endian unsigned short
        
        # 使用工具函数分离低八位和高八位
        rotate_low, rotate_high = split_to_bytes(speed_unsigned)
        
        cmd_data = CMD_TABLE['SERVO_OR_MOTOR_MODE_WRITE'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(servo_mode)
        cmd_data.append(0)  # 参数2：空值
        cmd_data.append(rotate_low)   # 参数3：转动速度值的低八位
        cmd_data.append(rotate_high)  # 参数4：转动速度值的高八位
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_load_or_unload(self, joint_id, load_or_unload):
        """设置关节舵机使能状态

        :param int joint_id: 关节ID
        :param int load_or_unload: 
            0 - 负载
        """
        cmd_data = CMD_TABLE['SERVO_LOAD_OR_UNLOAD_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(load_or_unload)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_id(self, joint_id, new_id):
        """设置指定关节的 ID
        
        :param int joint_id: 关节ID
        :param int new_id: 新的ID
        """
        cmd_data = CMD_TABLE['SERVO_ID_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(new_id)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_vin_limit(self, joint_id, vin_min, vin_max):
        """设置指定关节的电压限制
        
        :param int joint_id: 关节ID
        :param int vin_min: 电压最小值: 4500 ~ 12000 毫伏
        :param int vin_max: 电压最大值： 4500 ~ 12000 毫伏
        """
        cmd_data = CMD_TABLE['SERVO_VIN_LIMIT_WRITE']
        cmd_data[2] = joint_id
        cmd_data.extend(list(struct.pack('<HH', vin_min, vin_max)))
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_angle_limit(self, joint_id, angle_min, angle_max):
        """设置指定关节的角度限制
        
        :param int joint_id: 关节ID
        :param int angle_min: 角度最小值
        :param int angle_max: 角度最大值
        """
        cmd_data = CMD_TABLE['SERVO_ANGLE_LIMIT_WRITE']
        cmd_data[2] = joint_id
        cmd_data.extend(list(struct.pack('<HH', angle_min, angle_max)))
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_angle_offset_adjust(self, joint_id, angle_offset):
        """设置指定关节的角度偏移量调整， 用于临时校准舵机角度，掉电不保存
        
        :param int joint_id: 关节ID
        :param int angle_offset: 角度偏移量, 舵机内部的偏差值，范围 -125~125 ，对应角度为-30°~30°
        """
        cmd_data = CMD_TABLE['SERVO_ANGLE_OFFSET_ADJUST']
        cmd_data[2] = joint_id
        cmd_data.append(angle_offset)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
        
    def set_joint_angle_offset_write(self, joint_id, angle_offset):
        """设置指定关节的角度偏移量，掉电保存
        
        :param int joint_id: 关节ID
        :param int angle_offset: 角度偏移量
        """
        cmd_data = CMD_TABLE['SERVO_ANGLE_OFFSET_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(angle_offset)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_temp_limit_range(self, joint_id, temp_limit):
        """设置指定关节的温度限制范围
        
            如果舵机内部温度超过了此值，舵机的 LED 灯将会闪烁报警， (如果设置了 LED 报警) 为了
            保护舵机，其内的电机将会处于卸载断电状态，此时舵机将不会输出力矩，直
            到温度低于此值舵机会再次进入工作状态，并且此值支持掉电保存。
            
        :param int joint_id: 关节ID
        :param int temp_limit: 温度限制 50 ~ 100 摄氏度, 默认为 85 摄氏度
        """
        cmd_data = CMD_TABLE['SERVO_TEMP_MAX_LIMIT_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(temp_limit)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_led(self, joint_id, led_ctrl):
        """设置指定关节的 LED 灯的亮灭状态
        
        :param int joint_id: 关节ID
        :param int led_ctrl: 
            0 - LED 常亮
            1 - LED 常灭
        """
        cmd_data = CMD_TABLE['SERVO_LED_CTRL_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(led_ctrl)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_led_error(self, joint_id, led_error):
        """设置那些情况会导致指定舵机的 LED 灯发生闪烁

        :param int joint_id: 关节ID
        :param int led_error: 
            0 - 没有报警
            1 - 过温
            2 - 过压
            3 - 过温和过压
            4 - 堵转
            5 - 过温和堵转
            6 - 过压和堵转
            7 - 过温、过压和堵转
        """
        cmd_data = CMD_TABLE['SERVO_LED_ERROR_WRITE']
        cmd_data[2] = joint_id
        cmd_data.append(led_error)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    
    def set_joint_angle_use_time(self, joint_id, angle, time):
        """设置指定关节的角度和到达该角度的预计时间

        :param joint_id: 关节ID
        :param angle: 关节角度
        :param time: 到达该角度的花费的时间
        """
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_WRITE']
        cmd_data[2] = joint_id
        if angle is not None and time is not None:
            cmd_data.extend(list(struct.pack('<HH', angle, time)))
        else:
            raise ValueError('angle and time must be not None')
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_angle_with_time_after_start(self, joint_id, angle, delay_time):
        """设置指定关节的角度和到达该角度的延迟时间

        :param joint_id: 关节ID
        :param angle: 关节角度
        :param delay_time: 延迟指定时间后到达该角度的
        """
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_WAIT_WRITE']
        cmd_data[2] = joint_id
        if angle is not None and delay_time is not None:
            cmd_data.extend(list(struct.pack('<HH', angle, delay_time)))
        else:
            raise ValueError('angle and delay_time must be not None')
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_move_start(self, joint_id):
        """启动指定关节的运动"""
        cmd_data = CMD_TABLE['SERVO_MOVE_START']
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_emergency_stop(self, joint_id):
        """指定关节紧急停止运动"""
        cmd_data = CMD_TABLE['SERVO_MOVE_STOP']
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
if __name__ == '__main__':
    controller = RobotArmController()
    # controller.set_joint_angle_use_time(1, 300, 1000)
    # controller.set_joint_angle_with_time_after_start(1, 100, 10000)
    # controller.set_joint_move_start(1)
    # time.sleep(2)
    # controller.set_joint_emergency_stop(1)
    # controller.set_joint_load_or_unload(1, 1)
    # controller.set_joint_temp_limit_range(1, 85)
    controller.set_joint_led(3, 0)