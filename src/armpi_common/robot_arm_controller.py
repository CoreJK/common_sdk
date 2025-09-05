# -*- coding: utf-8 -*- 
# robot_arm_controller.py - 机械臂控制器

import enum
import time
import struct
import queue
import threading

import serial

from armpi_common.utils import calculate_checksum, split_to_bytes
from armpi_common.cmdTable import CMD_TABLE
from armpi_common._log import logger, set_file_level, set_stream_level, disable_logging
from armpi_common.armipi_module import RobotArmModule
from armpi_common.armipi_module import angle2pulse, pulse2angle
from armpi_common.utils import is_flat
from armpi_common.kinematics import ArmKinematics

import numpy as np
from spatialmath import SE3
from spatialmath.base import rpy2tr
from typing import List

set_stream_level("DEBUG")

class PacketControllerState(enum.IntEnum):
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1
    PACKET_CONTROLLER_STATE_ID = 2
    PACKET_CONTROLLER_STATE_LENGTH = 3
    PACKET_CONTROLLER_STATE_CMD = 4
    PACKET_CONTROLLER_STATE_DATA = 5
    PACKET_CONTROLLER_STATE_CHECKSUM = 6

class RobotArmController:
    def __init__(self, device='/dev/ttyUSB0', baudrate=115200, timeout=0):
        self.port = device
        self.baudrate = baudrate
        self.serial_client = serial.Serial(device, baudrate, timeout=timeout)
        self.serial_client.rts = False
        self.serial_client.dtr = False
        self.__enable_recv = False  # 是否开启接收数据包功能
        self.robot_arm_module = RobotArmModule()
        
        # 初始化运动学计算模块
        self.kinematics = ArmKinematics(self.robot_arm_module)
        
        # 数据接收相关        
        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
        self.frame = []  # 数据包
        self.recv_count = 0  # 接收计数器
        self.data_length = 0  # 数据长度 
        self.retry_times = 10  # 重试次数
        self.servo_recv_queue = queue.Queue(maxsize=1)  # 舵机数据接收队列
        self.servo_read_lock = threading.Lock()  # 舵机数据读取线程锁
        self.start_recv_task()  # 启动数据接收线程
        

    def start_recv_task(self):
        threading.Thread(target=self.recv_task, daemon=True).start()
        time.sleep(0.1)
        
    def enable_reception(self, enable=True):
        self.__enable_recv = enable
    
    def packet_report_serial_servo(self, data):
        try:
            self.servo_recv_queue.put_nowait(data)
        except:
            pass
    
    def recv_task(self):
        try:
            logger.info("接收数据线程启动")
            while True:
                if self.__enable_recv:
                    recv_data = self.serial_client.read()
                    if recv_data:
                        for data in recv_data:
                            if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                                if data == 0x55:
                                    logger.debug(f"接收到的 帧头 1: %0.2x" % data)
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                                else:
                                    logger.warning(f"接收到的 帧头 1: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                                if data == 0x55:
                                    logger.debug(f"接收到的 帧头 2: %0.2x" % data)
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_ID
                                else:
                                    logger.warning(f"接收到的 帧头 2: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_ID:
                                if data is not None:
                                    logger.debug(f"接收到的 ID: %0.2x" % data)
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                                else:
                                    logger.warning(f"接收到的 ID: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                                if data is not None:
                                    logger.debug(f"接收到的期望长度: %0.2x" % data)
                                    self.frame.append(data)
                                    self.data_length = self.frame[-1] - 3 # 减去ID、长度、指令字段的长度
                                    self.recv_count = 0  # 重置接收计数器
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CMD
                                else:
                                    logger.warning(f"接收到的 长度: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CMD:
                                if data is not None:
                                    logger.debug(f"接收到的 指令: %0.2x" % data)
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA
                                else:
                                    logger.warning(f"接收到的 指令: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                                if data is not None:
                                    logger.debug(f"接收到的 参数值: %0.2x" % data)
                                    self.frame.append(data)
                                    self.recv_count += 1
                                    if self.recv_count >= self.data_length:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                                else:
                                    logger.warning(f"接收到的 参数值: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                                # todo 计算校验和， 如果校验和正确，则将数据包发送给队列
                                logger.debug(f"接收到的校验和: %0.2x" % data)
                                crc_checksum = calculate_checksum(self.frame)
                                logger.debug(f"计算校验和: %0.2x" % crc_checksum)
                                if data is not None:
                                    self.packet_report_serial_servo(self.frame)
                                    self.frame.append(data)
                                    logger.debug(f"接收到数据包, 验证完整, 发送给队列: {list(map(lambda x: hex(x), self.frame))}")
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                else:
                                    logger.warning(f"接收到的 校验和: %0.2x 不正确" % data)
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                    else:
                        time.sleep(0.01)
        except Exception as e:
            logger.error(f"发生异常: {e}")
    
    def servo_read_and_unpack(self, cmd):
        """读取舵机数据并解包"""
        if self.__enable_recv:
            with self.servo_read_lock:
                logger.info("开始读取舵机数据")
                logger.debug(f"发送的读取指令: {list(map(lambda x: hex(x), cmd))}")
                count = 0
                while True:
                    self.bus_write(bytes(cmd))
                    
                    try:
                        recv_data = self.servo_recv_queue.get(block=True, timeout=0.1)
                        break
                    except queue.Empty:
                        logger.warning(f"读取舵机数据失败，重试 {count} 次")
                        count += 1
                        # todo: 重发机制
                        # logger.debug(f"重新发送命令: {list(map(lambda x: hex(x), cmd))}")
                        # self.bus_write(bytes(cmd))
                        if count > self.retry_times:
                            recv_data = None
                            break                        
                
                if recv_data is not None:
                    logger.debug(f"解包数据: {list(map(lambda x: hex(x), recv_data))}")
                    return {
                        'status': True,
                        'data': recv_data,
                        'info': "数据解析成功"
                    }
                else:
                    logger.warning("返回的数据为空")
                    return {
                        'status': False,
                        'data': None,
                        'info': "返回的数据为空"
                    }
        else:
            return {
                "status": False,
                "data": None,
                "info": "未开启接收数据功能"
            }
    
    def bus_write(self, cmd_data):
        """只负责发送已经构造好的数据"""
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
            0 - 卸载掉电（无力矩输出）
            1 - 装载电机（有力矩输出）
        """
        cmd_data = CMD_TABLE['SERVO_LOAD_OR_UNLOAD_WRITE'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(load_or_unload)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_id(self, joint_id, new_id):
        """设置指定关节的 ID
        
        :param int joint_id: 关节ID
        :param int new_id: 新的ID
        """
        cmd_data = CMD_TABLE['SERVO_ID_WRITE'].copy()
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
        cmd_data = CMD_TABLE['SERVO_VIN_LIMIT_WRITE'].copy()
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
        cmd_data = CMD_TABLE['SERVO_ANGLE_LIMIT_WRITE'].copy()
        cmd_data[2] = joint_id
        cmd_data.extend(list(struct.pack('<HH', angle_min, angle_max)))
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_angle_offset_adjust(self, joint_id, angle_offset):
        """设置指定关节的角度偏移量调整， 用于临时校准舵机角度，掉电不保存
        
        :param int joint_id: 关节ID
        :param int angle_offset: 角度偏移量, 舵机内部的偏差值，范围 -125~125 ，对应角度为-30°~30°
        """
        cmd_data = CMD_TABLE['SERVO_ANGLE_OFFSET_ADJUST'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(angle_offset)
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
        
    def set_joint_angle_offset_write(self, joint_id, angle_offset):
        """设置指定关节的角度偏移量，掉电保存
        
        :param int joint_id: 关节ID
        :param int angle_offset: 角度偏移量
        """
        cmd_data = CMD_TABLE['SERVO_ANGLE_OFFSET_WRITE'].copy()
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
        cmd_data = CMD_TABLE['SERVO_TEMP_MAX_LIMIT_WRITE'].copy()
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
        cmd_data = CMD_TABLE['SERVO_LED_CTRL_WRITE'].copy()
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
        cmd_data = CMD_TABLE['SERVO_LED_ERROR_WRITE'].copy()
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
        logger.info(f"设置关节{joint_id}的角度为{angle}，预计花费{time}ms")
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_WRITE'].copy()   
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
        logger.info(f"设置关节{joint_id}的角度为{angle}，延迟{delay_time}ms后到达")
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_WAIT_WRITE'].copy()
        cmd_data[2] = joint_id
        if angle is not None and delay_time is not None:
            cmd_data.extend(list(struct.pack('<HH', angle, delay_time)))
        else:
            raise ValueError('angle and delay_time must be not None')
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_move_start(self, joint_id):
        """启动指定关节的运动"""
        logger.info(f"启动关节{joint_id}的运动")
        cmd_data = CMD_TABLE['SERVO_MOVE_START'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def set_joint_move_with_coordinate(self, coordinate_list, move_type=0, move_time=0):
        """根据坐标列表和运动类型，设置关节运动
        
        :param list[int] args: 坐标与姿态列表
        :param move_type: 0- set_joint_angle_use_time, 1- set_joint_angle_with_time_after_start
        :param int move_time: 运动时间，单位为毫秒
        """
        if not is_flat(coordinate_list):
            logger.error(f"坐标列表不能嵌套, 只能是一维列表")
            return {
                "status": False,
                "info": "坐标列表不能嵌套, 只能是一维列表"
            }
        
        if len(coordinate_list) != 6:
            logger.error(f"坐标列表长度为 {len(coordinate_list)}，预期为 6")
            return {
                "status": False,
                "info": "坐标列表长度为 {len(coordinate_list)}，预期为 6"
            }
        
        if not all(isinstance(x, (int, float)) for x in coordinate_list):
            logger.error(f"坐标列表中的元素必须是整数或浮点数")
            return {
                "status": False,
                "info": "坐标列表中的元素必须是整数或浮点数"
            }
        
        # 逆解
        ikine = self.get_joint_ikine(coordinate_list, current_pose=False).get("ikine")
        if ikine is None:
            logger.error(f"获取机械臂的逆解失败")
            return {
                "status": False,
                "info": "获取机械臂的逆解失败"
            }
        
        if move_type == 0:
            for joint_id, joint_pulse in enumerate(ikine):
                self.set_joint_angle_use_time(joint_id + 1, joint_pulse, move_time)
        elif move_type == 1:
            for joint_id, joint_pulse in enumerate(ikine):
                self.set_joint_angle_with_time_after_start(joint_id + 1, joint_pulse, move_time)
        
        return {
            "status": True,
            "info": "设置关节运动成功"
        }

    def move_between_coordinates(self, start_coordinate_list, end_coordinate_list, duration_ms=2000, steps=60, mask=None, blocking=True):
        """在两个末端位姿坐标之间平滑过渡

        :param list[float] start_coordinate_list: 起点位姿 [x,y,z,roll,pitch,yaw]，单位 m / rad
        :param list[float] end_coordinate_list: 终点位姿 [x,y,z,roll,pitch,yaw]，单位 m / rad
        :param int duration_ms: 总时长（毫秒）
        :param int steps: 轨迹离散步数
        :param list[int] mask: 逆解掩码，长度6，默认仅约束 [x,y,z,yaw] -> [1,1,1,0,0,1]
        :param bool blocking: 是否阻塞等待执行（按步 sleep）
        
        Returns:
            Dict containing:
                - status (bool): 执行是否成功
                - steps (int): 实际完成的步数
                - info (str): 状态信息
        """
        logger.info("在两个末端位姿之间执行平滑过渡")
        
        # 委托给ArmKinematics模块进行轨迹规划
        trajectory_result = self.kinematics.plan_trajectory(
            start_coordinate_list=start_coordinate_list,
            end_coordinate_list=end_coordinate_list,
            duration_ms=duration_ms,
            steps=steps,
            mask=mask
        )
        
        if not trajectory_result["status"]:
            logger.error(f"轨迹规划失败: {trajectory_result['info']}")
            return {
                "status": False,
                "steps": 0,
                "info": f"轨迹规划失败: {trajectory_result['info']}"
            }
        
        # 获取规划好的轨迹
        trajectory = trajectory_result["trajectory"]
        planned_steps = trajectory_result["steps"]
        
        # 时间分配
        step_time_ms = max(1, int(round(duration_ms / planned_steps)))
        
        # 执行轨迹 - 硬件控制逻辑保留在控制器中
        steps_executed = 0
        for step_idx, joint_pulses in enumerate(trajectory):
            try:
                # 下发本步目标到硬件
                for joint_id, pulse in enumerate(joint_pulses, start=1):
                    self.set_joint_angle_use_time(joint_id, pulse, step_time_ms)
                
                steps_executed += 1
                
                # 阻塞等待（如果需要）
                if blocking:
                    time.sleep(step_time_ms / 1000.0)
                    
            except Exception as e:
                logger.error(f"执行第 {step_idx+1}/{planned_steps} 步时发生错误: {e}")
                break
        
        # 返回执行结果
        success = steps_executed == planned_steps
        info = "轨迹完成" if success else f"仅完成 {steps_executed}/{planned_steps} 步"
        
        return {
            "status": success,
            "steps": steps_executed,
            "info": info
        }
        
    def set_joint_emergency_stop(self, joint_id):
        """指定关节紧急停止运动"""
        logger.warning(f"紧急停止关节{joint_id}的运动")
        cmd_data = CMD_TABLE['SERVO_MOVE_STOP'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        self.bus_write(bytes(cmd_data))
    
    def get_joint_move_and_time(self, joint_id):
        """获取指定关节的最后一次角度参数和时间"""
        logger.info(f"获取关节 {joint_id} 最后一次的角度参数和时间")
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            # 数据包结构: [帧头1, 帧头2, ID, 长度, 指令, 角度低字节, 角度高字节, 时间低字节, 时间高字节, 校验和]
            # 机械臂返回的数据使用小端序格式，直接读取即可
            data = recv_data['data']
            
            # 验证数据包长度
            expected_length = 9  # 帧头(2) + ID(1) + 长度(1) + 指令(1) + 角度(2) + 时间(2)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'angle': None,
                    'time_ms': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            
            try:
                angle = struct.unpack('<H', bytes(data[5:7]))[0]  # 角度参数值
                time_ms = struct.unpack('<H', bytes(data[7:9]))[0]  # 时间值（毫秒）

                return {
                    'id': joint_id,
                    'angle': angle,
                    'time_ms': time_ms
                }
            except struct.error as e:
                return {
                    'id': joint_id,
                    'angle': None,
                    'time_ms': None,
                    "info": f"数据解析错误: {e}"
                }
        else:
            return {
                'id': joint_id,
                'angle': None,
                'time_ms': None,
                "info": recv_data['info']
            }
    
    def get_joint_move_and_wait_time(self, joint_id):
        """获取指定关节的最后一次角度参数和延迟启动时间"""
        logger.info(f"获取关节 {joint_id} 最后一次的角度参数和延迟启动时间")
        cmd_data = CMD_TABLE['SERVO_MOVE_TIME_WAIT_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 9  # 帧头(2) + ID(1) + 长度(1) + 指令(1) + 角度(2) + 时间(2)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'angle': None,
                    'delay_time': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            try:
                angle = struct.unpack('<H', bytes(data[5:7]))[0]  # 角度参数值
                delay_time = struct.unpack('<H', bytes(data[7:9]))[0]  # 时间值（毫秒）
                return {
                    'id': joint_id,
                    'angle': angle,
                    'delay_time': delay_time
                }
            except struct.error as e:   
                return {
                    'id': joint_id,
                    'angle': None,
                    'delay_time': None,
                    "info": f"数据解析错误: {e}"
                }
        else:
            return {
                'id': joint_id,
                'angle': None,
                'delay_time': None,
                "info": recv_data['info']
            }

    def get_joint_angle_offset(self, joint_id):
        """获取指定关节的角度偏移量"""
        logger.info(f"获取关节 {joint_id} 的角度偏移量")
        cmd_data = CMD_TABLE['SERVO_ANGLE_OFFSET_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'angle_offset': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                 # 角度偏移量是 signed byte，范围 -125~125
                 angle_offset = struct.unpack('b', struct.pack('B', data[5]))[0]
                 return {
                     'id': joint_id,
                     'angle_offset': angle_offset
                 }
        else:
            return {
                'id': joint_id,
                'angle_offset': None,
                'info': recv_data['info']
            }
            
    def get_joint_angle_limit(self, joint_id):
        """获取指定关节的角度限制"""
        logger.info(f"获取关节 {joint_id} 的角度限制")
        cmd_data = CMD_TABLE['SERVO_ANGLE_LIMIT_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'angle_limit': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                return {
                    'id': joint_id,
                    'angle_limit': data[5]
                }
        else:
            return {
                'id': joint_id,
                'angle_limit': None,
                'info': recv_data['info']
            }
    
    def get_joint_id(self, joint_id):
        """获取指定关节的ID"""
        logger.info(f"获取关节{joint_id}的ID")
        cmd_data = CMD_TABLE['SERVO_ID_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        if recv_data['status'] == True:
            return {
                'id': joint_id,
                'current_id': recv_data['data'][5]
            }
        else:
            return {
                'id': joint_id,
                'current_id': None,
                'info': recv_data['info']
            }
    
    def get_joint_vin_limit(self, joint_id):
        """获取指定关节的电压限制"""
        logger.info(f"获取关节 {joint_id} 的电压限制")
        cmd_data = CMD_TABLE['SERVO_VIN_LIMIT_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 7  # 帧头(2) + ID(1) + 长度(1) + 指令(1) + 电压低字节(2) + 电压高字节(2)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'vin_limit': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                vin_limit_low = struct.unpack('<H', bytes(data[5:7]))[0]
                vin_limit_high = struct.unpack('<H', bytes(data[7:9]))[0]
                return {
                    'id': joint_id,
                    'vin_limit_low': vin_limit_low,
                    'vin_limit_high': vin_limit_high
                }
        else:
            return {
                'id': joint_id,
                'vin_limit_low': None,
                'vin_limit_high': None,
                'info': recv_data['info']
            }

    def get_joint_temp_max_limit(self, joint_id):
        """获取指定关节的温度限制"""
        logger.info(f"获取关节 {joint_id} 的温度限制")
        cmd_data = CMD_TABLE['SERVO_TEMP_MAX_LIMIT_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'temp_max_limit': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                return {
                    'id': joint_id,
                    'temp_max_limit': data[5]
                }
        else:
            return {
                'id': joint_id,
                'temp_max_limit': None,
                'info': recv_data['info']
            }
    
    def get_joint_temp(self, joint_id):
        """获取指定关节的温度"""
        logger.info(f"获取关节 {joint_id} 的温度")
        cmd_data = CMD_TABLE['SERVO_TEMP_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'temp': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                return {
                    'id': joint_id,
                    'temp': data[5]
                }
        else:
            return {
                'id': joint_id,
                'temp': None,
                'info': recv_data['info']
            }
    
    def get_joint_input_voltage(self, joint_id):
        """获取指定关节的输入电压"""
        logger.info(f"获取关节 {joint_id} 的输入电压")
        cmd_data = CMD_TABLE['SERVO_VIN_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'input_voltage': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                input_voltage = struct.unpack('<H', bytes(data[5:7]))[0]
                return {
                    'id': joint_id,
                    'input_voltage': input_voltage
                }
        else:
            return {
                'id': joint_id,
                'input_voltage': None,
                'info': recv_data['info']
            }
    
    def get_joint_position(self, joint_id):
        """获取指定关节的位置"""
        logger.info(f"获取关节 {joint_id} 的位置")
        cmd_data = CMD_TABLE['SERVO_POS_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'position': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                joint_position = struct.unpack('<h', bytes(data[5:7]))[0]
                return {
                    'id': joint_id,
                    'position': joint_position
                }
        else:
            return {
                'id': joint_id,
                'position': None,
                'info': recv_data['info']
            }
    
    def get_all_joint_position(self, pulse_to_angle=False):
        """获取所有关节的位置, 默认返回脉冲值

        :param bool pulse_to_angle: True, 返回角度的弧度值, False, 返回脉冲值
        """
        logger.info(f"获取所有关节的位置")
        position_list = []
        # 不会获取夹爪的关节位置, 因为不参与正逆解的计算
        for i in range(1, 6):
            joint_data = self.get_joint_position(i)
            logger.debug(f"获取关节 {i} 的位置，位置为 {joint_data['position']}")
            if joint_data['position'] is not None:
                position_list.append(joint_data['position'])
            else:
                logger.error(f"获取关节 {i} 的位置失败")
                return {
                    "position_list": None,
                    "info": f"获取关节 {i} 的位置失败"
                }
        
        if pulse_to_angle:
            position_list = pulse2angle(position_list)
            logger.debug(f"获取所有关节的位置成功，位置列表为 {np.round(np.degrees(position_list), 3).tolist()}")
        else:
            logger.debug(f"获取所有关节的位置成功，位置列表为 {position_list}")
            
        return {
            "position_list": position_list,
            "info": "获取所有关节的位置成功"
        }
    
    def get_joint_fkine(self, joint_position_list=None, current_pose=False):
        """获取机械臂的正解
        
        Args:
            joint_position_list: 关节位置列表（脉冲值），如果为None则使用current_pose参数
            current_pose: 是否使用当前机械臂位置
            
        Returns:
            Dict containing:
                - status (bool): 计算是否成功
                - fkine (List[float] or None): 末端位姿 [x, y, z, rx, ry, rz]
                - info (str): 状态信息
        """
        logger.info("获取机械臂的正解")
        
        if current_pose:
            # 使用当前位置
            position_result = self.get_all_joint_position(pulse_to_angle=True)
            if position_result.get("position_list") is None:
                logger.error("获取当前关节位置失败")
                return {
                    "status": False,
                    "fkine": None,
                    "info": "获取当前关节位置失败"
                }
            position_list = position_result["position_list"]
        else:
            # 使用提供的关节位置列表
            if joint_position_list is None:
                logger.error("必须提供关节位置列表或设置current_pose=True")
                return {
                    "status": False,
                    "fkine": None,
                    "info": "必须提供关节位置列表或设置current_pose=True"
                }
            
            # 验证输入格式（保持与原始实现的兼容性）
            if not is_flat(joint_position_list):
                logger.error("关节脉冲列表不能嵌套, 只能是一维列表")
                return {
                    "status": False,
                    "fkine": None,
                    "info": "关节脉冲列表不能嵌套, 只能是一维列表"
                }
            
            if len(joint_position_list) != 5:
                logger.error(f"关节脉冲列表长度为 {len(joint_position_list)}，预期为 5")
                return {
                    "status": False,
                    "fkine": None,
                    "info": f"关节脉冲列表长度为 {len(joint_position_list)}，预期为 5"
                }
            
            if not all(isinstance(x, int) for x in joint_position_list):
                logger.error("关节脉冲列表中的元素必须是整数")
                return {
                    "status": False,
                    "fkine": None,
                    "info": "关节脉冲列表中的元素必须是整数"
                }
            
            # 转换脉冲值为角度值
            position_list = pulse2angle(joint_position_list)
        
        # 委托给ArmKinematics模块进行计算
        result = self.kinematics.forward_kinematics(position_list)
        
        # 保持原有的返回格式和信息
        if result["status"]:
            return {
                "status": True,
                "fkine": result["fkine"],
                "info": "获取机械臂的正解成功"
            }
        else:
            return {
                "status": False,
                "fkine": None,
                "info": "获取机械臂的正解失败"
            }
    
    def get_joint_ikine(self, end_tool_coordinate_list=None, current_pose=False):
        """获取机械臂的逆解
        
        Args:
            end_tool_coordinate_list: 末端位姿列表 [x,y,z,rx,ry,rz]，如果为None则使用current_pose参数
            current_pose: 是否使用当前机械臂位姿作为目标
            
        Returns:
            Dict containing:
                - status (bool): 计算是否成功
                - ikine (List[int] or None): 关节脉冲值列表
                - info (str): 状态信息
        """
        logger.info("获取机械臂的逆解")
        
        if current_pose:
            # 使用当前位姿
            fkine_result = self.get_joint_fkine(current_pose=True)
            if not fkine_result.get("status") or fkine_result.get("fkine") is None:
                logger.error("获取机械臂当前位姿失败")
                return {
                    "status": False,
                    "ikine": None,
                    "info": "获取机械臂当前位姿失败"
                }
            coordinate_list = fkine_result["fkine"]
        else:
            # 使用提供的末端坐标列表
            if end_tool_coordinate_list is None:
                logger.error("必须提供末端坐标列表或设置current_pose=True")
                return {
                    "status": False,
                    "ikine": None,
                    "info": "必须提供末端坐标列表或设置current_pose=True"
                }
            
            # 验证输入格式（保持与原始实现的兼容性）
            if not is_flat(end_tool_coordinate_list):
                logger.error("末端坐标列表不能嵌套, 只能是一维列表")
                return {
                    "status": False,
                    "ikine": None,
                    "info": "末端坐标列表不能嵌套, 只能是一维列表"
                }
            
            if len(end_tool_coordinate_list) != 6:
                logger.error(f"末端坐标列表长度为 {len(end_tool_coordinate_list)}，预期为 6")
                return {
                    "status": False,
                    "ikine": None,
                    "info": f"末端坐标列表长度为 {len(end_tool_coordinate_list)}，预期为 6"
                }
            
            if not all(isinstance(x, (int, float)) for x in end_tool_coordinate_list):
                logger.error("末端坐标列表中的元素必须是数值类型")
                return {
                    "status": False,
                    "ikine": None,
                    "info": "末端坐标列表中的元素必须是数值类型"
                }
            
            coordinate_list = end_tool_coordinate_list
        
        # 委托给ArmKinematics模块进行计算
        result = self.kinematics.inverse_kinematics(coordinate_list)
        
        # 保持原有的返回格式和信息
        if result["status"]:
            return {
                "status": True,
                "ikine": result["ikine"],
                "info": "获取机械臂的逆解成功"
            }
        else:
            return {
                "status": False,
                "ikine": None,
                "info": "获取机械臂的逆解失败"
            }

    def get_joint_mode_and_speed(self, joint_id):
        """获取指定关节的模式和速度"""
        logger.info(f"获取关节 {joint_id} 的模式和速度")
        cmd_data = CMD_TABLE['SERVO_OR_MOTOR_MODE_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 10  # 帧头(2) + ID(1) + 长度(1) + 指令(1) + 模式(1) + 参数2(1) + 速度(2) + 校验和(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'servo_mode': None,
                    'joint_speed': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            
            try:
                servo_mode = data[5]        # 模式：0=位置控制，1=电机控制
                # 速度作为 signed short int 解析，范围 -1000~1000
                joint_speed = struct.unpack('<h', bytes(data[7:9]))[0]
                
                return {
                    'id': joint_id,
                    'servo_mode': servo_mode,
                    'joint_speed': joint_speed,
                }
            except (struct.error, IndexError) as e:
                return {
                    'id': joint_id,
                    'servo_mode': None,
                    'joint_speed': None,
                    "info": f"数据解析错误: {e}"
                }
        else:
            return {
                'id': joint_id,
                'servo_mode': None,
                'joint_speed': None,
                'info': recv_data['info']
            }
    
    def get_joint_load_or_unload(self, joint_id):
        """获取指定关节的使能状态"""
        logger.info(f"获取关节 {joint_id} 的使能状态")
        cmd_data = CMD_TABLE['SERVO_LOAD_OR_UNLOAD_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'load_or_unload': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                return {
                    'id': joint_id,
                    'load_or_unload': data[5]
                }
        else:
            return {
                'id': joint_id,
                'load_or_unload': None,
                'info': recv_data['info']
            }
    
    def get_joint_led_ctrl(self, joint_id):
        """获取指定关节的LED控制"""
        logger.info(f"获取关节 {joint_id} 的LED控制")
        cmd_data = CMD_TABLE['SERVO_LED_CTRL_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {    
                    'id': joint_id,
                    'led_ctrl': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }
            else:
                return {
                    'id': joint_id,
                    'led_ctrl': data[5]
                }
        else:
            return {
                'id': joint_id,
                'led_ctrl': None,
                'info': recv_data['info']
            }
    
    def get_joint_led_error(self, joint_id):
        """获取指定关节的LED错误"""
        logger.info(f"获取关节 {joint_id} 的LED错误")
        cmd_data = CMD_TABLE['SERVO_LED_ERROR_READ'].copy()
        cmd_data[2] = joint_id
        cmd_data.append(calculate_checksum(cmd_data))
        recv_data= self.servo_read_and_unpack(cmd_data)
        
        if recv_data['status'] == True:
            data = recv_data['data']
            expected_length = 4  # 帧头(2) + ID(1) + 长度(1) + 指令(1)
            if len(data) < expected_length:
                return {
                    'id': joint_id,
                    'led_error': None,
                    "info": f"数据包长度不足，期望{expected_length}字节，实际{len(data)}字节"
                }   
            else:
                return {
                    'id': joint_id,
                    'led_error': data[5]
                }
        else:
            return {
                'id': joint_id,
                'led_error': None,
                'info': recv_data['info']
            }
    
    # ===== 电机使能管理相关方法 =====
    
    def get_all_joints_load_status(self) -> dict:
        """
        获取所有关节的使能状态
        
        Returns:
            包含所有关节使能状态的字典
        """
        logger.info("获取所有关节的使能状态")
        joint_status = {}
        failed_joints = []
        
        # 获取1-6号关节的使能状态（包括夹爪）
        for joint_id in range(1, 7):
            status_result = self.get_joint_load_or_unload(joint_id)
            if status_result.get('load_or_unload') is not None:
                joint_status[joint_id] = status_result['load_or_unload']
                logger.debug(f"关节{joint_id}使能状态: {status_result['load_or_unload']}")
            else:
                failed_joints.append(joint_id)
                logger.warning(f"获取关节{joint_id}使能状态失败")
        
        if failed_joints:
            return {
                "status": False,
                "info": f"获取关节{failed_joints}使能状态失败",
                "joint_status": joint_status,
                "failed_joints": failed_joints
            }
        else:
            return {
                "status": True,
                "info": "成功获取所有关节使能状态",
                "joint_status": joint_status
            }
    
    def set_all_joints_load_status(self, load_or_unload: int, 
                                 include_gripper: bool = True) -> dict:
        """
        批量设置所有关节的使能状态
        
        Args:
            load_or_unload: 使能状态
                0 - 卸载掉电（无力矩输出）
                1 - 装载电机（有力矩输出）
            include_gripper: 是否包括夹爪（第6关节）
            
        Returns:
            设置结果
        """
        logger.info(f"批量设置所有关节使能状态: {load_or_unload}")
        
        end_joint = 7 if include_gripper else 6
        failed_joints = []
        
        try:
            for joint_id in range(1, end_joint):
                try:
                    self.set_joint_load_or_unload(joint_id, load_or_unload)
                    time.sleep(0.1)  # 短暂延时确保指令执行
                    logger.debug(f"关节{joint_id}使能状态设置为: {load_or_unload}")
                except Exception as e:
                    failed_joints.append(joint_id)
                    logger.error(f"设置关节{joint_id}使能状态失败: {e}")
            
            if failed_joints:
                return {
                    "status": False,
                    "info": f"关节{failed_joints}使能状态设置失败",
                    "failed_joints": failed_joints
                }
            else:
                status_desc = "卸载掉电（无力矩输出）" if load_or_unload == 0 else "装载电机（有力矩输出）"
                return {
                    "status": True,
                    "info": f"成功设置所有关节为{status_desc}状态"
                }
                
        except Exception as e:
            logger.error(f"批量设置关节使能状态时发生异常: {e}")
            return {
                "status": False,
                "info": f"批量设置失败: {e}"
            }
    
    def unload_all_motors(self, include_gripper: bool = False) -> dict:
        """
        卸载所有电机的使能（标定前安全措施）
        
        Args:
            include_gripper: 是否包括夹爪电机
            
        Returns:
            卸载结果
        """
        logger.info("🔴 卸载所有电机使能（标定安全模式）")
        
        # 首先保存当前使能状态
        current_status = self.get_all_joints_load_status()
        if current_status['status']:
            self._saved_motor_status = current_status['joint_status']
            logger.debug(f"已保存当前电机使能状态: {self._saved_motor_status}")
        else:
            logger.warning("无法保存当前电机使能状态，继续执行卸载操作")
            self._saved_motor_status = None
        
        # 卸载所有电机使能
        result = self.set_all_joints_load_status(
            load_or_unload=0,  # 0 = 卸载掉电（无力矩输出）
            include_gripper=include_gripper
        )
        
        if result['status']:
            logger.info("✅ 所有电机已卸载使能，进入标定安全模式")
        else:
            logger.error("❌ 电机卸载使能失败")
        
        return result
    
    def reload_all_motors(self, include_gripper: bool = False, 
                         restore_previous: bool = True) -> dict:
        """
        重新使能所有电机（标定后恢复）
        
        Args:
            include_gripper: 是否包括夹爪电机
            restore_previous: 是否恢复之前保存的状态
            
        Returns:
            恢复结果
        """
        logger.info("🟢 恢复所有电机使能（退出标定安全模式）")
        
        try:
            if restore_previous and hasattr(self, '_saved_motor_status') and self._saved_motor_status:
                # 恢复之前保存的状态
                logger.info("恢复到标定前的电机使能状态")
                failed_joints = []
                
                end_joint = 7 if include_gripper else 6
                for joint_id in range(1, end_joint):
                    if joint_id in self._saved_motor_status:
                        try:
                            original_status = self._saved_motor_status[joint_id]
                            self.set_joint_load_or_unload(joint_id, original_status)
                            time.sleep(0.1)
                            logger.debug(f"关节{joint_id}恢复到原状态: {original_status}")
                        except Exception as e:
                            failed_joints.append(joint_id)
                            logger.error(f"恢复关节{joint_id}状态失败: {e}")
                
                if failed_joints:
                    return {
                        "status": False,
                        "info": f"关节{failed_joints}状态恢复失败",
                        "failed_joints": failed_joints
                    }
                else:
                    return {
                        "status": True,
                        "info": "成功恢复所有关节到标定前状态"
                    }
            else:
                # 默认使能所有电机
                logger.info("装载所有电机到默认状态")
                result = self.set_all_joints_load_status(
                    load_or_unload=1,  # 1 = 装载电机（有力矩输出）
                    include_gripper=include_gripper
                )
                
                if result['status']:
                    logger.info("✅ 所有电机已重新使能")
                else:
                    logger.error("❌ 电机重新使能失败")
                
                return result
                
        except Exception as e:
            logger.error(f"恢复电机使能时发生异常: {e}")
            return {
                "status": False,
                "info": f"恢复失败: {e}"
            }
        finally:
            # 清理保存的状态
            if hasattr(self, '_saved_motor_status'):
                del self._saved_motor_status
    
    def check_motors_safety_status(self) -> dict:
        """
        检查电机安全状态（是否处于标定安全模式）
        
        Returns:
            安全状态检查结果
        """
        status_result = self.get_all_joints_load_status()
        
        if not status_result['status']:
            return {
                "status": False,
                "info": "无法获取电机状态",
                "is_safe_mode": None
            }
        
        joint_status = status_result['joint_status']
        
        # 检查1-5号关节（不包括夹爪）是否都已卸载掉电
        main_joints_unloaded = all(
            joint_status.get(joint_id, 1) == 0 
            for joint_id in range(1, 6)
        )
        
        # 统计装载和卸载的关节数量
        enabled_joints = [jid for jid, status in joint_status.items() if status == 1]  # 装载电机（有力矩输出）
        disabled_joints = [jid for jid, status in joint_status.items() if status == 0]  # 卸载掉电（无力矩输出）
        
        return {
            "status": True,
            "info": "电机安全状态检查完成",
            "is_safe_mode": main_joints_unloaded,
            "enabled_joints": enabled_joints,
            "disabled_joints": disabled_joints,
            "joint_status": joint_status
        }
    
    # ===== 手眼标定相关方法 =====
    
    def initialize_hand_eye_calibration(self, 
                                      camera_matrix: np.ndarray = None,
                                      distortion_coeffs: np.ndarray = None,
                                      board_size: tuple = (9, 6),
                                      square_size: float = 0.025,
                                      camera_source: int = 2,
                                      save_directory: str = "./hand_eye_calibration"):
        """
        初始化手眼标定系统
        
        Args:
            camera_matrix: 相机内参矩阵
            distortion_coeffs: 相机畸变系数
            board_size: 标定板尺寸 (列数, 行数)
            square_size: 标定板方格大小 (米)
            camera_source: 相机设备ID
            save_directory: 保存目录
            
        Returns:
            初始化结果
        """
        try:
            from armpi_common.hand_eye_calibration import HandEyeCalibration
            from armpi_common.calibration_data_collector import CalibrationDataCollector
            
            # 创建手眼标定器
            self.hand_eye_calibrator = HandEyeCalibration(
                camera_matrix=camera_matrix,
                distortion_coeffs=distortion_coeffs,
                board_size=board_size,
                square_size=square_size
            )
            
            # 创建数据收集器
            self.calibration_collector = CalibrationDataCollector(
                robot_controller=self,
                camera_source=camera_source,
                save_directory=save_directory
            )
            
            logger.info("手眼标定系统初始化成功")
            return {
                "status": True,
                "info": "手眼标定系统初始化成功"
            }
            
        except ImportError as e:
            logger.error(f"导入手眼标定模块失败: {e}")
            return {
                "status": False,
                "info": f"导入手眼标定模块失败: {e}"
            }
        except Exception as e:
            logger.error(f"手眼标定系统初始化失败: {e}")
            return {
                "status": False,
                "info": f"初始化失败: {e}"
            }
    
    def perform_camera_calibration(self, 
                                 camera_source: int = 0,
                                 num_images: int = 20,
                                 board_size: tuple = (9, 6),
                                 square_size: float = 0.025,
                                 save_directory: str = "./camera_calibration") -> dict:
        """
        执行相机标定
        
        Args:
            camera_source: 相机设备ID
            num_images: 采集图像数量
            board_size: 标定板尺寸
            square_size: 方格大小
            save_directory: 保存目录
            
        Returns:
            标定结果
        """
        try:
            from armpi_common.camera_calibration import CameraCalibration
            
            logger.info("开始相机标定流程")
            
            # 创建相机标定器
            camera_calibrator = CameraCalibration(
                board_size=board_size,
                square_size=square_size,
                save_directory=save_directory
            )
            
            # 采集标定图像
            collected = camera_calibrator.collect_calibration_images(
                camera_source=camera_source,
                num_images=num_images,
                auto_capture=True
            )
            
            if collected < 10:
                return {
                    "status": False,
                    "info": f"采集的有效图像数量不足，需要至少10张，实际采集{collected}张"
                }
            
            # 执行标定
            success = camera_calibrator.calibrate_camera()
            if not success:
                return {
                    "status": False,
                    "info": "相机标定失败"
                }
            
            # 保存结果
            camera_calibrator.save_calibration()
            
            # 验证标定
            validation = camera_calibrator.validate_calibration()
            
            return {
                "status": True,
                "info": "相机标定成功",
                "camera_matrix": camera_calibrator.camera_matrix.tolist(),
                "distortion_coeffs": camera_calibrator.distortion_coeffs.tolist(),
                "calibration_error": camera_calibrator.calibration_error,
                "validation": validation,
                "calibrator": camera_calibrator
            }
            
        except Exception as e:
            logger.error(f"相机标定失败: {e}")
            return {
                "status": False,
                "info": f"相机标定失败: {e}"
            }
    
    def perform_hand_eye_calibration(self,
                                   num_poses: int = 15,
                                   calibration_method: str = 'tsai',
                                   workspace_center: list = None,
                                   workspace_radius: float = 0.05,
                                   camera_source: int = 0,
                                   show_preview: bool = True) -> dict:
        """
        执行手眼标定
        
        Args:
            num_poses: 采集位姿数量
            calibration_method: 标定方法 ('tsai', 'park', 'horaud', 'andreff', 'daniilidis')
            workspace_center: 工作空间中心 [x, y, z]
            workspace_radius: 工作空间半径
            camera_source: 相机设备ID
            show_preview: 是否显示预览
            
        Returns:
            标定结果
        """
        try:
            # 检查手眼标定系统是否已初始化
            if not hasattr(self, 'hand_eye_calibrator') or not hasattr(self, 'calibration_collector'):
                return {
                    "status": False,
                    "info": "手眼标定系统未初始化，请先调用 initialize_hand_eye_calibration"
                }
            
            logger.info(f"开始手眼标定流程，采集{num_poses}个位姿")
            
            # 标定前安全措施：卸载所有电机使能
            logger.info("🔴 标定安全检查：卸载所有电机使能")
            unload_result = self.unload_all_motors(include_gripper=False)
            if not unload_result['status']:
                logger.warning(f"电机卸载使能失败，继续执行标定: {unload_result['info']}")
            else:
                logger.info("✅ 所有电机已安全卸载，标定可以安全进行")
            
            motors_unloaded = unload_result['status']  # 记录是否成功卸载
            
            # 设置默认工作空间中心
            if workspace_center is None:
                workspace_center = [0.15, 0.0, 0.20]
            
            # 生成标定位姿
            poses = self.calibration_collector.generate_calibration_poses(
                num_poses=num_poses,
                workspace_center=workspace_center,
                workspace_radius=workspace_radius
            )
            
            # 采集标定数据
            collection_result = self.calibration_collector.collect_calibration_data(
                poses=poses,
                show_preview=show_preview
            )
            
            if not collection_result['success'] or collection_result['successful_samples'] < 3:
                # 数据采集失败，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 数据采集失败：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": f"数据采集失败或样本数量不足: {collection_result}",
                    "collection_result": collection_result
                }
            
            # 添加数据到标定器
            added_samples = self.calibration_collector.add_data_to_calibrator(self.hand_eye_calibrator)
            if added_samples < 3:
                # 样本数量不足，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 样本不足：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": f"有效标定样本数量不足，需要至少3个，实际{added_samples}个"
                }
            
            # 执行手眼标定
            calibration_success = self.hand_eye_calibrator.solve_hand_eye_calibration(calibration_method)
            if not calibration_success:
                # 标定算法失败，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 标定算法失败：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": "手眼标定算法执行失败"
                }
            
            # 保存标定结果
            calibration_file = self.calibration_collector.save_directory + "/hand_eye_calibration.json"
            self.hand_eye_calibrator.save_calibration(calibration_file)
            
            # 标定成功完成，恢复电机使能状态
            if motors_unloaded:
                logger.info("🟢 标定完成：恢复电机使能状态")
                reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                if reload_result['status']:
                    logger.info("✅ 电机使能状态已恢复")
                else:
                    logger.warning(f"恢复电机使能失败: {reload_result['info']}")
            
            return {
                "status": True,
                "info": "手眼标定成功完成",
                "collection_result": collection_result,
                "added_samples": added_samples,
                "calibration_error": self.hand_eye_calibrator.calibration_error,
                "calibration_file": calibration_file,
                "hand_eye_transform": self.hand_eye_calibrator.hand_eye_transform.A.tolist(),
                "motor_status_restored": motors_unloaded and reload_result.get('status', False) if 'reload_result' in locals() else False
            }
            
        except Exception as e:
            logger.error(f"手眼标定失败: {e}")
            
            # 即使发生异常，也要尝试恢复电机使能状态
            if 'motors_unloaded' in locals() and motors_unloaded:
                logger.info("🟢 异常处理：尝试恢复电机使能状态")
                try:
                    reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                    if reload_result['status']:
                        logger.info("✅ 电机使能状态已恢复")
                    else:
                        logger.warning(f"恢复电机使能失败: {reload_result['info']}")
                except Exception as reload_e:
                    logger.error(f"恢复电机使能时发生异常: {reload_e}")
            
            return {
                "status": False,
                "info": f"手眼标定失败: {e}"
            }
    
    def load_hand_eye_calibration(self, calibration_file: str) -> dict:
        """
        加载手眼标定结果
        
        Args:
            calibration_file: 标定文件路径
            
        Returns:
            加载结果
        """
        try:
            if not hasattr(self, 'hand_eye_calibrator'):
                from armpi_common.hand_eye_calibration import HandEyeCalibration
                self.hand_eye_calibrator = HandEyeCalibration()
            
            success = self.hand_eye_calibrator.load_calibration(calibration_file)
            if success:
                return {
                    "status": True,
                    "info": "手眼标定结果加载成功",
                    "calibration_error": self.hand_eye_calibrator.calibration_error,
                    "hand_eye_transform": self.hand_eye_calibrator.hand_eye_transform.A.tolist()
                }
            else:
                return {
                    "status": False,
                    "info": "手眼标定结果加载失败"
                }
                
        except Exception as e:
            logger.error(f"加载手眼标定失败: {e}")
            return {
                "status": False,
                "info": f"加载失败: {e}"
            }
    
    def get_camera_pose_in_base(self, robot_pose: list = None) -> dict:
        """
        获取相机在基座坐标系中的位姿
        
        Args:
            robot_pose: 机器人末端位姿，为None时使用当前位姿
            
        Returns:
            相机位姿结果
        """
        try:
            if not hasattr(self, 'hand_eye_calibrator') or self.hand_eye_calibrator.hand_eye_transform is None:
                return {
                    "status": False,
                    "info": "手眼标定未完成，无法计算相机位姿"
                }
            
            # 获取机器人位姿
            if robot_pose is None:
                fk_result = self.get_joint_fkine(current_pose=True)
                if not fk_result or fk_result['fkine'] is None:
                    return {
                        "status": False,
                        "info": "无法获取当前机器人位姿"
                    }
                robot_pose = fk_result['fkine']
            
            # 计算相机位姿
            camera_pose = self.hand_eye_calibrator.get_camera_pose_in_base(robot_pose)
            if camera_pose is None:
                return {
                    "status": False,
                    "info": "相机位姿计算失败"
                }
            
            # 提取位姿信息
            camera_position = camera_pose.t.tolist()
            camera_orientation = camera_pose.rpy(order='zyx').tolist()
            
            return {
                "status": True,
                "info": "相机位姿计算成功",
                "camera_pose": camera_position + camera_orientation,
                "camera_transform_matrix": camera_pose.A.tolist()
            }
            
        except Exception as e:
            logger.error(f"计算相机位姿失败: {e}")
            return {
                "status": False,
                "info": f"计算失败: {e}"
            }
    
    def validate_hand_eye_calibration(self, 
                                    test_poses: list = None,
                                    camera_source: int = 0,
                                    num_test_poses: int = 5) -> dict:
        """
        验证手眼标定精度
        
        Args:
            test_poses: 测试位姿列表，为None时自动生成
            camera_source: 相机设备ID
            num_test_poses: 测试位姿数量
            
        Returns:
            验证结果
        """
        try:
            if not hasattr(self, 'hand_eye_calibrator') or self.hand_eye_calibrator.hand_eye_transform is None:
                return {
                    "status": False,
                    "info": "手眼标定未完成，无法进行验证"
                }
            
            # 生成测试位姿
            if test_poses is None:
                if hasattr(self, 'calibration_collector'):
                    test_poses = self.calibration_collector.generate_calibration_poses(
                        num_poses=num_test_poses,
                        workspace_center=[0.15, 0.0, 0.20],
                        workspace_radius=0.03
                    )
                else:
                    return {
                        "status": False,
                        "info": "无法生成测试位姿，请提供test_poses参数"
                    }
            
            validation_results = []
            successful_validations = 0
            
            # 初始化相机
            import cv2
            cap = cv2.VideoCapture(camera_source)
            if not cap.isOpened():
                return {
                    "status": False,
                    "info": f"无法打开相机: {camera_source}"
                }
            
            try:
                for i, pose in enumerate(test_poses):
                    logger.info(f"验证测试位姿 {i+1}/{len(test_poses)}")
                    
                    # 移动到测试位姿
                    move_result = self.set_joint_move_with_coordinate(pose, move_type=0, move_time=3000)
                    if not move_result['status']:
                        logger.warning(f"无法移动到测试位姿 {i+1}")
                        continue
                    
                    time.sleep(3.5)  # 等待移动完成
                    
                    # 采集图像
                    ret, image = cap.read()
                    if not ret:
                        logger.warning(f"采集测试图像 {i+1} 失败")
                        continue
                    
                    # 验证标定
                    validation = self.hand_eye_calibrator.validate_calibration(pose, image)
                    if validation:
                        validation_results.append(validation)
                        successful_validations += 1
                        logger.info(f"测试位姿 {i+1} 验证成功: "
                                  f"平移误差={validation.get('translation_error_mm', 0):.2f}mm")
                    else:
                        logger.warning(f"测试位姿 {i+1} 验证失败")
            
            finally:
                cap.release()
            
            # 计算统计结果
            if validation_results:
                translation_errors = [r.get('translation_error_mm', 0) for r in validation_results]
                rotation_errors = [r.get('rotation_error_deg', 0) for r in validation_results]
                
                summary = {
                    "status": True,
                    "info": "手眼标定验证完成",
                    "successful_tests": successful_validations,
                    "total_tests": len(test_poses),
                    "success_rate": successful_validations / len(test_poses),
                    "mean_translation_error_mm": np.mean(translation_errors),
                    "std_translation_error_mm": np.std(translation_errors),
                    "mean_rotation_error_deg": np.mean(rotation_errors),
                    "std_rotation_error_deg": np.std(rotation_errors),
                    "detailed_results": validation_results
                }
            else:
                summary = {
                    "status": False,
                    "info": "没有成功的验证结果",
                    "successful_tests": 0,
                    "total_tests": len(test_poses)
                }
            
            logger.info(f"手眼标定验证完成: 成功率={summary.get('success_rate', 0):.1%}")
            return summary
            
        except Exception as e:
            logger.error(f"手眼标定验证失败: {e}")
            return {
                "status": False,
                "info": f"验证失败: {e}"
            }
    
    def get_hand_eye_calibration_info(self) -> dict:
        """
        获取手眼标定信息摘要
        
        Returns:
            标定信息
        """
        if not hasattr(self, 'hand_eye_calibrator'):
            return {
                "status": False,
                "info": "手眼标定系统未初始化"
            }
        
        try:
            calibration_info = self.hand_eye_calibrator.get_calibration_info()
            calibration_info['status'] = True
            return calibration_info
            
        except Exception as e:
            logger.error(f"获取手眼标定信息失败: {e}")
            return {
                "status": False,
                "info": f"获取信息失败: {e}"
            }
    
    # ===== 眼在手外标定相关方法 =====
    
    def initialize_eye_to_hand_calibration(self, 
                                         camera_matrix: np.ndarray = None,
                                         distortion_coeffs: np.ndarray = None,
                                         board_size: tuple = (9, 6),
                                         square_size: float = 0.025,
                                         camera_source: int = 2,
                                         camera_pose: List[float] = None,
                                         board_to_end_transform: np.ndarray = None,
                                         save_directory: str = "./eye_to_hand_calibration"):
        """
        初始化眼在手外标定系统
        
        Args:
            camera_matrix: 相机内参矩阵
            distortion_coeffs: 相机畸变系数
            board_size: 标定板尺寸 (列数, 行数)
            square_size: 标定板方格大小 (米)
            camera_source: 相机设备ID
            camera_pose: 相机固定位姿 [x,y,z,rx,ry,rz] (可选)
            board_to_end_transform: 标定板相对于机械臂末端的变换矩阵 (4x4)
            save_directory: 保存目录
            
        Returns:
            初始化结果
        """
        try:
            from armpi_common.eye_to_hand_calibration import EyeToHandCalibration
            from armpi_common.calibration_data_collector import CalibrationDataCollector
            
            # 创建眼在手外标定器
            self.eye_to_hand_calibrator = EyeToHandCalibration(
                camera_matrix=camera_matrix,
                distortion_coeffs=distortion_coeffs,
                board_size=board_size,
                square_size=square_size,
                board_to_end_transform=board_to_end_transform
            )
            
            # 创建数据收集器（眼在手外模式）
            self.eye_to_hand_collector = CalibrationDataCollector(
                robot_controller=self,
                camera_source=camera_source,
                save_directory=save_directory,
                calibration_type="eye_to_hand",
                camera_pose=camera_pose
            )
            
            logger.info("眼在手外标定系统初始化成功")
            return {
                "status": True,
                "info": "眼在手外标定系统初始化成功"
            }
            
        except ImportError as e:
            logger.error(f"导入眼在手外标定模块失败: {e}")
            return {
                "status": False,
                "info": f"导入眼在手外标定模块失败: {e}"
            }
        except Exception as e:
            logger.error(f"眼在手外标定系统初始化失败: {e}")
            return {
                "status": False,
                "info": f"初始化失败: {e}"
            }
    
    def perform_eye_to_hand_calibration(self,
                                      num_poses: int = 15,
                                      calibration_method: str = 'tsai',
                                      workspace_center: list = None,
                                      workspace_radius: float = 0.05,
                                      camera_source: int = 2,
                                      show_preview: bool = True) -> dict:
        """
        执行眼在手外标定
        
        Args:
            num_poses: 采集位姿数量
            calibration_method: 标定方法 ('tsai', 'park', 'horaud', 'andreff', 'daniilidis')
            workspace_center: 工作空间中心 [x, y, z]
            workspace_radius: 工作空间半径
            camera_source: 相机设备ID
            show_preview: 是否显示预览
            
        Returns:
            标定结果
        """
        try:
            # 检查眼在手外标定系统是否已初始化
            if not hasattr(self, 'eye_to_hand_calibrator') or not hasattr(self, 'eye_to_hand_collector'):
                return {
                    "status": False,
                    "info": "眼在手外标定系统未初始化，请先调用 initialize_eye_to_hand_calibration"
                }
            
            logger.info(f"开始眼在手外标定流程，采集{num_poses}个位姿")
            
            # 标定前安全措施：卸载所有电机使能
            logger.info("🔴 标定安全检查：卸载所有电机使能")
            unload_result = self.unload_all_motors(include_gripper=False)
            if not unload_result['status']:
                logger.warning(f"电机卸载使能失败，继续执行标定: {unload_result['info']}")
            else:
                logger.info("✅ 所有电机已安全卸载，标定可以安全进行")
            
            motors_unloaded = unload_result['status']  # 记录是否成功卸载
            
            # 设置默认工作空间中心
            if workspace_center is None:
                workspace_center = [0.15, 0.0, 0.20]
            
            # 生成标定位姿
            poses = self.eye_to_hand_collector.generate_calibration_poses(
                num_poses=num_poses,
                workspace_center=workspace_center,
                workspace_radius=workspace_radius
            )
            
            # 采集标定数据
            collection_result = self.eye_to_hand_collector.collect_calibration_data(
                poses=poses,
                show_preview=show_preview
            )
            
            if not collection_result['success'] or collection_result['successful_samples'] < 3:
                # 数据采集失败，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 数据采集失败：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": f"数据采集失败或样本数量不足: {collection_result}",
                    "collection_result": collection_result
                }
            
            # 添加数据到标定器
            added_samples = self.eye_to_hand_collector.add_data_to_calibrator(self.eye_to_hand_calibrator)
            if added_samples < 3:
                # 样本数量不足，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 样本不足：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": f"有效标定样本数量不足，需要至少3个，实际{added_samples}个"
                }
            
            # 执行眼在手外标定
            calibration_success = self.eye_to_hand_calibrator.solve_eye_to_hand_calibration(calibration_method)
            if not calibration_success:
                # 标定算法失败，恢复电机使能后返回
                if motors_unloaded:
                    logger.info("🟢 标定算法失败：恢复电机使能状态")
                    try:
                        reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                        if reload_result['status']:
                            logger.info("✅ 电机使能状态已恢复")
                    except Exception as reload_e:
                        logger.error(f"恢复电机使能时发生异常: {reload_e}")
                
                return {
                    "status": False,
                    "info": "眼在手外标定算法执行失败"
                }
            
            # 保存标定结果
            calibration_file = self.eye_to_hand_collector.save_directory + "/eye_to_hand_calibration.json"
            self.eye_to_hand_calibrator.save_calibration(calibration_file)
            
            # 标定成功完成，恢复电机使能状态
            if motors_unloaded:
                logger.info("🟢 标定完成：恢复电机使能状态")
                reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                if reload_result['status']:
                    logger.info("✅ 电机使能状态已恢复")
                else:
                    logger.warning(f"恢复电机使能失败: {reload_result['info']}")
            
            return {
                "status": True,
                "info": "眼在手外标定成功完成",
                "collection_result": collection_result,
                "added_samples": added_samples,
                "calibration_error": self.eye_to_hand_calibrator.calibration_error,
                "calibration_file": calibration_file,
                "eye_to_hand_transform": self.eye_to_hand_calibrator.eye_to_hand_transform.A.tolist(),
                "motor_status_restored": motors_unloaded and reload_result.get('status', False) if 'reload_result' in locals() else False
            }
            
        except Exception as e:
            logger.error(f"眼在手外标定失败: {e}")
            
            # 即使发生异常，也要尝试恢复电机使能状态
            if 'motors_unloaded' in locals() and motors_unloaded:
                logger.info("🟢 异常处理：尝试恢复电机使能状态")
                try:
                    reload_result = self.reload_all_motors(include_gripper=False, restore_previous=True)
                    if reload_result['status']:
                        logger.info("✅ 电机使能状态已恢复")
                    else:
                        logger.warning(f"恢复电机使能失败: {reload_result['info']}")
                except Exception as reload_e:
                    logger.error(f"恢复电机使能时发生异常: {reload_e}")
            
            return {
                "status": False,
                "info": f"眼在手外标定失败: {e}"
            }
    
    def load_eye_to_hand_calibration(self, calibration_file: str) -> dict:
        """
        加载眼在手外标定结果
        
        Args:
            calibration_file: 标定文件路径
            
        Returns:
            加载结果
        """
        try:
            if not hasattr(self, 'eye_to_hand_calibrator'):
                from armpi_common.eye_to_hand_calibration import EyeToHandCalibration
                self.eye_to_hand_calibrator = EyeToHandCalibration()
            
            success = self.eye_to_hand_calibrator.load_calibration(calibration_file)
            if success:
                return {
                    "status": True,
                    "info": "眼在手外标定结果加载成功",
                    "calibration_error": self.eye_to_hand_calibrator.calibration_error,
                    "eye_to_hand_transform": self.eye_to_hand_calibrator.eye_to_hand_transform.A.tolist()
                }
            else:
                return {
                    "status": False,
                    "info": "眼在手外标定结果加载失败"
                }
                
        except Exception as e:
            logger.error(f"加载眼在手外标定失败: {e}")
            return {
                "status": False,
                "info": f"加载失败: {e}"
            }
    
    def get_fixed_camera_pose(self) -> dict:
        """
        获取固定相机在基座坐标系中的位姿（眼在手外配置）
        
        Returns:
            相机位姿结果
        """
        try:
            if not hasattr(self, 'eye_to_hand_calibrator') or self.eye_to_hand_calibrator.eye_to_hand_transform is None:
                return {
                    "status": False,
                    "info": "眼在手外标定未完成，无法获取相机位姿"
                }
            
            # 获取相机位姿
            camera_pose = self.eye_to_hand_calibrator.get_camera_pose_in_base()
            if camera_pose is None:
                return {
                    "status": False,
                    "info": "相机位姿计算失败"
                }
            
            # 提取位姿信息
            camera_position = camera_pose.t.tolist()
            camera_orientation = camera_pose.rpy(order='zyx').tolist()
            
            return {
                "status": True,
                "info": "固定相机位姿获取成功",
                "camera_pose": camera_position + camera_orientation,
                "camera_transform_matrix": camera_pose.A.tolist()
            }
            
        except Exception as e:
            logger.error(f"获取固定相机位姿失败: {e}")
            return {
                "status": False,
                "info": f"获取失败: {e}"
            }
    
    def get_board_pose_from_robot_pose(self, robot_pose: list = None) -> dict:
        """
        根据机器人位姿计算标定板在基座坐标系中的位姿（眼在手外配置）
        
        Args:
            robot_pose: 机器人末端位姿，为None时使用当前位姿
            
        Returns:
            标定板位姿结果
        """
        try:
            if not hasattr(self, 'eye_to_hand_calibrator') or self.eye_to_hand_calibrator.eye_to_hand_transform is None:
                return {
                    "status": False,
                    "info": "眼在手外标定未完成，无法计算标定板位姿"
                }
            
            # 获取机器人位姿
            if robot_pose is None:
                fk_result = self.get_joint_fkine(current_pose=True)
                if not fk_result or fk_result['fkine'] is None:
                    return {
                        "status": False,
                        "info": "无法获取当前机器人位姿"
                    }
                robot_pose = fk_result['fkine']
            
            # 计算标定板位姿
            board_pose = self.eye_to_hand_calibrator.get_board_pose_from_robot(robot_pose)
            if board_pose is None:
                return {
                    "status": False,
                    "info": "标定板位姿计算失败"
                }
            
            # 提取位姿信息
            board_position = board_pose.t.tolist()
            board_orientation = board_pose.rpy(order='zyx').tolist()
            
            return {
                "status": True,
                "info": "标定板位姿计算成功",
                "board_pose": board_position + board_orientation,
                "board_transform_matrix": board_pose.A.tolist()
            }
            
        except Exception as e:
            logger.error(f"计算标定板位姿失败: {e}")
            return {
                "status": False,
                "info": f"计算失败: {e}"
            }
    
    def validate_eye_to_hand_calibration(self, 
                                       test_poses: list = None,
                                       camera_source: int = 2,
                                       num_test_poses: int = 5) -> dict:
        """
        验证眼在手外标定精度
        
        Args:
            test_poses: 测试位姿列表，为None时自动生成
            camera_source: 相机设备ID
            num_test_poses: 测试位姿数量
            
        Returns:
            验证结果
        """
        try:
            if not hasattr(self, 'eye_to_hand_calibrator') or self.eye_to_hand_calibrator.eye_to_hand_transform is None:
                return {
                    "status": False,
                    "info": "眼在手外标定未完成，无法进行验证"
                }
            
            # 生成测试位姿
            if test_poses is None:
                if hasattr(self, 'eye_to_hand_collector'):
                    test_poses = self.eye_to_hand_collector.generate_calibration_poses(
                        num_poses=num_test_poses,
                        workspace_center=[0.15, 0.0, 0.20],
                        workspace_radius=0.03
                    )
                else:
                    return {
                        "status": False,
                        "info": "无法生成测试位姿，请提供test_poses参数"
                    }
            
            validation_results = []
            successful_validations = 0
            
            # 初始化相机
            import cv2
            cap = cv2.VideoCapture(camera_source)
            if not cap.isOpened():
                return {
                    "status": False,
                    "info": f"无法打开相机: {camera_source}"
                }
            
            try:
                for i, pose in enumerate(test_poses):
                    logger.info(f"验证测试位姿 {i+1}/{len(test_poses)}")
                    
                    # 移动到测试位姿
                    move_result = self.set_joint_move_with_coordinate(pose, move_type=0, move_time=3000)
                    if not move_result['status']:
                        logger.warning(f"无法移动到测试位姿 {i+1}")
                        continue
                    
                    time.sleep(3.5)  # 等待移动完成
                    
                    # 采集图像
                    ret, image = cap.read()
                    if not ret:
                        logger.warning(f"采集测试图像 {i+1} 失败")
                        continue
                    
                    # 验证标定
                    validation = self.eye_to_hand_calibrator.validate_calibration(pose, image)
                    if validation:
                        validation_results.append(validation)
                        successful_validations += 1
                        logger.info(f"测试位姿 {i+1} 验证成功: "
                                  f"平移误差={validation.get('translation_error_mm', 0):.2f}mm")
                    else:
                        logger.warning(f"测试位姿 {i+1} 验证失败")
            
            finally:
                cap.release()
            
            # 计算统计结果
            if validation_results:
                translation_errors = [r.get('translation_error_mm', 0) for r in validation_results]
                rotation_errors = [r.get('rotation_error_deg', 0) for r in validation_results]
                
                summary = {
                    "status": True,
                    "info": "眼在手外标定验证完成",
                    "successful_tests": successful_validations,
                    "total_tests": len(test_poses),
                    "success_rate": successful_validations / len(test_poses),
                    "mean_translation_error_mm": np.mean(translation_errors),
                    "std_translation_error_mm": np.std(translation_errors),
                    "mean_rotation_error_deg": np.mean(rotation_errors),
                    "std_rotation_error_deg": np.std(rotation_errors),
                    "detailed_results": validation_results
                }
            else:
                summary = {
                    "status": False,
                    "info": "没有成功的验证结果",
                    "successful_tests": 0,
                    "total_tests": len(test_poses)
                }
            
            logger.info(f"眼在手外标定验证完成: 成功率={summary.get('success_rate', 0):.1%}")
            return summary
            
        except Exception as e:
            logger.error(f"眼在手外标定验证失败: {e}")
            return {
                "status": False,
                "info": f"验证失败: {e}"
            }
    
    def get_eye_to_hand_calibration_info(self) -> dict:
        """
        获取眼在手外标定信息摘要
        
        Returns:
            标定信息
        """
        if not hasattr(self, 'eye_to_hand_calibrator'):
            return {
                "status": False,
                "info": "眼在手外标定系统未初始化"
            }
        
        try:
            calibration_info = self.eye_to_hand_calibrator.get_calibration_info()
            calibration_info['status'] = True
            return calibration_info
            
        except Exception as e:
            logger.error(f"获取眼在手外标定信息失败: {e}")
            return {
                "status": False,
                "info": f"获取信息失败: {e}"
            }
    
    
    def close_connection(self):
        logger.info("机械臂断开连接")
        self.enable_reception(False)
        self.serial_client.close()
    
if __name__ == '__main__':
    controller = RobotArmController(device="/dev/ttyUSB0")
    controller.enable_reception(True)
    
    current_pose = controller.get_joint_fkine(current_pose=True).get("fkine")
    logger.info(f"机械臂当前 pose: {current_pose}")
    # logger.debug(controller.get_joint_ikine([0.0, -0.0, 0.259799, 0.0, -0.0, -3.141593], current_pose=True))
    # [499, 498, 502, 499, 498]
    # [306, 498, 502, 499, 498]
    
    # logger.info(controller.get_joint_fkine(current_pose=True))
    # controller.set_joint_angle_use_time(1, 500, 2000)
    # time.sleep(2)
    # logger.info(controller.get_joint_fkine(current_pose=True))
    
    pose_b = [0.11855860931202489, -0.00347731659031457, 0.21505940094612014, 1.2262157409165864, -0.7071826799390757, 1.7705563538323623]
    home_pose = [0.009030721138624147, -0.0003784993595718179, 0.25958276486578696, 0.0014056788188084033, -0.0670059225018822, 3.07871368847899]
    
    logger.info(controller.set_joint_move_with_coordinate(home_pose, move_type=0, move_time=3000))
    # logger.info(controller.set_joint_move_with_coordinate(pose_b, move_type=0, move_time=1000))
    
    # controller.move_between_coordinates(current_pose, home_pose, duration_ms=2000, steps=10, mask=[1, 1, 1, 1, 1, 0], blocking=True)
    # controller.set_joint_angle_use_time(6, 300, 1000)
    # time.sleep(2)
    # controller.set_joint_angle_use_time(6, 500, 1000)
    # controller.set_joint_angle_use_time(1, 500, 2000)
    # controller.set_joint_angle_use_time(2, 200, 2000)
    # controller.set_joint_angle_use_time(3, 500, 2000)
    # controller.set_joint_angle_use_time(4, 500, 2000)
    # controller.set_joint_angle_use_time(5, 500, 2000)
    # controller.set_joint_angle_use_time(6, 500, 2000)
    
    # logger.info(f"关节1角度: {controller.get_joint_move_and_time(1)}")
    # logger.info(f"关节2角度: {controller.get_joint_move_and_time(2)}")
    # logger.info(f"关节3角度: {controller.get_joint_move_and_time(3)}")
    # logger.info(f"关节4角度: {controller.get_joint_move_and_time(4)}")
    # logger.info(f"关节5角度: {controller.get_joint_move_and_time(5)}")
    # logger.info(f"关节6角度: {controller.get_joint_move_and_time(6)}")
    
    # logger.info(f"关节1 ID: {controller.get_joint_id(1)}")
    # logger.info(f"关节2 ID: {controller.get_joint_id(2)}")
    # logger.info(f"关节3 ID: {controller.get_joint_id(3)}")
    # logger.info(f"关节4 ID: {controller.get_joint_id(4)}")
    # logger.info(f"关节5 ID: {controller.get_joint_id(5)}")
    # logger.info(f"关节6 ID: {controller.get_joint_id(6)}")
    
    # controller.set_joint_angle_with_time_after_start(1, 100, 5000)
    # controller.set_joint_move_start(1)
    # time.sleep(2)
    # todo: 待调试
    # logger.info(f"关节1角度和时间: {controller.get_joint_move_and_wait_time(1)}")
    
    # logger.info(f"关节1角度偏移量: {controller.get_joint_angle_offset(1)}")
    # logger.info(f"关节2角度偏移量: {controller.get_joint_angle_offset(2)}")
    # logger.info(f"关节3角度偏移量: {controller.get_joint_angle_offset(3)}")
    # logger.info(f"关节4角度偏移量: {controller.get_joint_angle_offset(4)}")
    # logger.info(f"关节5角度偏移量: {controller.get_joint_angle_offset(5)}")
    # logger.info(f"关节6角度偏移量: {controller.get_joint_angle_offset(6)}")
    
    # logger.info(f"关节1角度限制: {controller.get_joint_angle_limit(1)}")
    # logger.info(f"关节2角度限制: {controller.get_joint_angle_limit(2)}")
    # logger.info(f"关节3角度限制: {controller.get_joint_angle_limit(3)}")
    # logger.info(f"关节4角度限制: {controller.get_joint_angle_limit(4)}")
    # logger.info(f"关节5角度限制: {controller.get_joint_angle_limit(5)}")
    # logger.info(f"关节6角度限制: {controller.get_joint_angle_limit(6)}")
    
    # logger.info(f"关节1电压限制: {controller.get_joint_vin_limit(1)}")
    # logger.info(f"关节2电压限制: {controller.get_joint_vin_limit(2)}")
    # logger.info(f"关节3电压限制: {controller.get_joint_vin_limit(3)}")
    # logger.info(f"关节4电压限制: {controller.get_joint_vin_limit(4)}")
    # logger.info(f"关节5电压限制: {controller.get_joint_vin_limit(5)}")
    # logger.info(f"关节6电压限制: {controller.get_joint_vin_limit(6)}")
    
    # logger.info(f"关节1温度限制: {controller.get_joint_temp_max_limit(1)}")
    # logger.info(f"关节2温度限制: {controller.get_joint_temp_max_limit(2)}")
    # logger.info(f"关节3温度限制: {controller.get_joint_temp_max_limit(3)}")
    # logger.info(f"关节4温度限制: {controller.get_joint_temp_max_limit(4)}")
    # logger.info(f"关节5温度限制: {controller.get_joint_temp_max_limit(5)}")
    # logger.info(f"关节6温度限制: {controller.get_joint_temp_max_limit(6)}")
    
    # logger.info(f"关节1温度: {controller.get_joint_temp(1)}")
    # logger.info(f"关节2温度: {controller.get_joint_temp(2)}")
    # logger.info(f"关节3温度: {controller.get_joint_temp(3)}")
    # logger.info(f"关节4温度: {controller.get_joint_temp(4)}")
    # logger.info(f"关节5温度: {controller.get_joint_temp(5)}")
    # logger.info(f"关节6温度: {controller.get_joint_temp(6)}")
    
    # logger.info(f"关节1输入电压: {controller.get_joint_input_voltage(1)}")
    # logger.info(f"关节2输入电压: {controller.get_joint_input_voltage(2)}")
    # logger.info(f"关节3输入电压: {controller.get_joint_input_voltage(3)}")
    # logger.info(f"关节4输入电压: {controller.get_joint_input_voltage(4)}")
    # logger.info(f"关节5输入电压: {controller.get_joint_input_voltage(5)}")
    # logger.info(f"关节6输入电压: {controller.get_joint_input_voltage(6)}")
    
    # logger.info(f"关节1位置: {controller.get_joint_position(1)}")
    # logger.info(f"关节2位置: {controller.get_joint_position(2)}")
    # logger.info(f"关节3位置: {controller.get_joint_position(3)}")
    # logger.info(f"关节4位置: {controller.get_joint_position(4)}")
    # logger.info(f"关节5位置: {controller.get_joint_position(5)}")
    # logger.info(f"关节6位置: {controller.get_joint_position(6)}")
    
    # logger.info(f"关节1模式和速度: {controller.get_joint_mode_and_speed(1)}")
    # logger.info(f"关节2模式和速度: {controller.get_joint_mode_and_speed(2)}")
    # logger.info(f"关节3模式和速度: {controller.get_joint_mode_and_speed(3)}")
    # logger.info(f"关节4模式和速度: {controller.get_joint_mode_and_speed(4)}")
    # logger.info(f"关节5模式和速度: {controller.get_joint_mode_and_speed(5)}")
    # logger.info(f"关节6模式和速度: {controller.get_joint_mode_and_speed(6)}")
    
    # logger.info(f"关节1使能状态: {controller.get_joint_load_or_unload(1)}")
    # logger.info(f"关节2使能状态: {controller.get_joint_load_or_unload(2)}")
    # logger.info(f"关节3使能状态: {controller.get_joint_load_or_unload(3)}")
    # logger.info(f"关节4使能状态: {controller.get_joint_load_or_unload(4)}")
    # logger.info(f"关节5使能状态: {controller.get_joint_load_or_unload(5)}")
    # logger.info(f"关节6使能状态: {controller.get_joint_load_or_unload(6)}")
    
    # logger.info(f"关节1LED控制: {controller.get_joint_led_ctrl(1)}")
    # logger.info(f"关节2LED控制: {controller.get_joint_led_ctrl(2)}")
    # logger.info(f"关节3LED控制: {controller.get_joint_led_ctrl(3)}")
    # logger.info(f"关节4LED控制: {controller.get_joint_led_ctrl(4)}")
    # logger.info(f"关节5LED控制: {controller.get_joint_led_ctrl(5)}")
    # logger.info(f"关节6LED控制: {controller.get_joint_led_ctrl(6)}")
    
    # logger.info(f"关节1LED错误: {controller.get_joint_led_error(1)}")
    # logger.info(f"关节2LED错误: {controller.get_joint_led_error(2)}")
    # logger.info(f"关节3LED错误: {controller.get_joint_led_error(3)}")
    # logger.info(f"关节4LED错误: {controller.get_joint_led_error(4)}")
    # logger.info(f"关节5LED错误: {controller.get_joint_led_error(5)}")
    # logger.info(f"关节6LED错误: {controller.get_joint_led_error(6)}")