import unittest
import platform

import serial.tools.list_ports
from armpi_common.robot_arm_controller import RobotArmController
    
class ConnectTestCase(unittest.TestCase):
    """机械臂的测试用例"""
    
    def setUp(self):
        self.platform_version = platform.system()
        self.robot_port : str = list(serial.tools.list_ports.comports())[0].device
        self.baud_rate = 115200
        self.robot_controller = RobotArmController(device=self.robot_port, baudrate=self.baud_rate)
        self.robot_controller.enable_reception(True)
        
    def test_get_joint_ID(self):
        recv_data = self.robot_controller.get_joint_id(1)
        current_joint_id = recv_data.get("current_id")
        self.assertEqual(1, current_joint_id)
    
    def tearDown(self):
        self.robot_controller.close_connection()


if __name__ == "__main__":
    unittest.main()