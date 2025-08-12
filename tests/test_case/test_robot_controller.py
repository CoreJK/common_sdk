import unittest
import ddt
import platform

from armpi_common.robot_arm_controller import RobotArmController
from configparser import ConfigParser

@ddt.ddt
class ConnectTestCase(unittest.TestCase):
    """机械臂的关节配置信息测试用例"""
    
    def setUp(self):
        self.platform_version = platform.system()
        self.config = ConfigParser()
        self.config.read("tests/test_case/robot_config.ini")
        self.robot_port : str = self.config.get("robot_config", "robot_port")
        self.baud_rate : int = int(self.config.get("robot_config", "baud_rate"))
        self.robot_controller = RobotArmController(device=self.robot_port, baudrate=self.baud_rate)
        self.robot_controller.enable_reception(True)
    
    def test_get_joint_angle_offset(self):
        """获取关节角度偏移"""
        pass
    
    def test_joint_angle_limit(self):
        """获取关节角度限制"""
        pass
    
    @ddt.data(1, 2, 3, 4, 5 ,6)
    def test_get_joint_id(self, ID):
        """测试获取关节当前 ID 编号"""
        recv_data = self.robot_controller.get_joint_id(ID)
        current_joint_id = recv_data.get("current_id")
        self.assertEqual(current_joint_id, ID, "ID 编号与预期不符")
    
    def test_get_joint_vin_limit(self):
        """获取指定关节的电压限制"""
        pass
    
    def test_get_joint_temp_max_limit(self):
        """获取指定关节的温度限制"""
        pass
    
    def test_get_joint_temp(self):
        """获取关节的当前温度"""
        pass
    
    def test_get_joint_input_voltage(self):
        """获取指定关节的当前输入电压"""
        pass
    
    def test_get_joint_position(self):
        """获取指定关节的位置值"""
        pass
    
    def test_get_all_joint_position(self):
        """获取所有关节的位置"""
        recv_data = self.robot_controller.get_all_joint_position()
        position_list = recv_data.get("position_list")
        if position_list is None:
            self.fail(recv_data.get("info"))
        self.assertEqual(len(position_list), 5, "关节位置数量与预期不符")

    
    def tearDown(self):
        self.robot_controller.close_connection()


if __name__ == "__main__":
    unittest.main()