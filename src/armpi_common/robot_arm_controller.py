# -*- coding: utf-8 -*- 
# robot_arm_controller.py - 机械臂控制器

import enum
from itertools import count
import queue
import threading
from queue import Queue
import struct
import time
import copy

import serial

from armpi_common.cmdTable import CMD_TABLE
from armpi_common.utils import calculate_checksum, get_command_info_by_id, split_to_bytes
from armpi_common._log import logger

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
        self.enable_recv = False

        # 数据接收相关        
        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
        self.frame = []  # 数据包
        self.recv_count = 0
        self.data_length = 0
        self.retry_times = 10
        self.servo_recv_queue = Queue(maxsize=1)
        self.servo_read_lock = threading.Lock()
        threading.Thread(target=self.recv_task, daemon=True).start()
        time.sleep(0.1)
        
        
    def enable_reception(self, enable=True):
        self.enable_recv = enable
    
    def packet_report_serial_servo(self, data):
        try:
            self.servo_recv_queue.put_nowait(data)
        except:
            pass
    
    def recv_task(self):
        try:
            logger.info("接收数据线程启动")
            while True:
                if self.enable_recv:
                    recv_data = self.serial_client.read()
                    if recv_data:
                        for data in recv_data:
                            logger.debug(f"接收到的数据: %0.2x" % data)
                            if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                                if data == 0x55:
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                                if data == 0x55:
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_ID
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_ID:
                                if data:
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                                if data:
                                    self.frame.append(data)
                                    self.data_length = data - 3  # 减去ID、长度、指令字段的长度
                                    logger.debug(f"期望数据长度: {data}")
                                    self.recv_count = 0  # 重置接收计数器
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CMD
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CMD:
                                if data:
                                    self.frame.append(data)
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                                if data:
                                    # todo 根据数据包的长度 - 1, 判断返回参数的长度
                                    self.frame.append(data)
                                    self.recv_count += 1
                                    if self.recv_count >= self.data_length:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                
                            elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                                # todo 计算校验和， 如果校验和正确，则将数据包发送给队列
                                crc_checksum = calculate_checksum(self.frame)
                                logger.debug(f"计算校验和: %0.2x" % crc_checksum)
                                logger.debug(f"接收到的校验和: %0.2x" % data)
                                if data:
                                    self.packet_report_serial_servo(self.frame)
                                    logger.debug(f"接收到数据包, 验证完整, 发送给队列: {list(map(lambda x: hex(x), self.frame))}")
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                else:
                                    self.frame = []
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                    else:
                        time.sleep(0.01)
        except Exception as e:
            logger.error(f"发生异常: {e}")
    
    def servo_read_and_unpack(self, cmd):
        """读取舵机数据并解包"""
        if self.enable_recv:
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
                        # logger.debug(f"重新发送命令: {cmd}")
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
    
if __name__ == '__main__':
    controller = RobotArmController()
    controller.enable_reception(False)
    
    controller.set_joint_angle_use_time(1, 500, 2000)
    controller.set_joint_angle_use_time(2, 500, 2000)
    controller.set_joint_angle_use_time(3, 500, 2000)
    controller.set_joint_angle_use_time(4, 500, 2000)
    controller.set_joint_angle_use_time(5, 500, 2000)
    controller.set_joint_angle_use_time(6, 500, 2000)
    
    # logger.info(f"关节1角度: {controller.get_joint_angle(1)}")
    # logger.info(f"关节2角度: {controller.get_joint_angle(2)}")
    # logger.info(f"关节3角度: {controller.get_joint_angle(3)}")
    # logger.info(f"关节4角度: {controller.get_joint_angle(4)}")
    # logger.info(f"关节5角度: {controller.get_joint_angle(5)}")
    # logger.info(f"关节6角度: {controller.get_joint_angle(6)}")
    # logger.info(f"关节1 ID: {controller.get_joint_id(1)}")
    # logger.info(f"关节2 ID: {controller.get_joint_id(2)}")
    # logger.info(f"关节3 ID: {controller.get_joint_id(3)}")
    # logger.info(f"关节4 ID: {controller.get_joint_id(4)}")
    # logger.info(f"关节5 ID: {controller.get_joint_id(5)}")
    # logger.info(f"关节6 ID: {controller.get_joint_id(6)}")
    
    
    # controller.set_joint_angle_with_time_after_start(1, 100, 10000)
    # controller.set_joint_move_start(1)
    # time.sleep(2)
    # controller.set_joint_emergency_stop(1)
    # controller.set_joint_load_or_unload(1, 1)
    # controller.set_joint_temp_limit_range(1, 85)
    # controller.set_joint_led(3, 0)
    
    