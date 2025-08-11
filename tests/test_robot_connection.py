import unittest
import platform

import serial.tools.list_ports
from armpi_common.robot_arm_controller import RobotArmController
    
class ConnectTestCase(unittest.TestCase):
    """测试机械臂的连接"""
    
    def setUp(self):
        self.platform_version = platform.system()
        self.robot_port : str = list(serial.tools.list_ports.comports())[0].device
        self.baud_rate = 115200
        self.robot_controller = RobotArmController(device=self.robot_port, baudrate=self.baud_rate)
    
    def test_get_joint_ID(self):
        joint_id = self.robot_controller.get_joint_id(1).get("current_id")
        self.assertEqual(1, joint_id)
    
    def tearDown(self):
        self.robot_controller.close_connection()