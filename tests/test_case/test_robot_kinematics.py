import unittest
import numpy as np
from math import radians, degrees
import ddt
from spatialmath import SE3
from spatialmath.base import rpy2tr


from armpi_common.armipi_module import RobotArmModule


@ddt.ddt
class RobotArmKinematicsTestCase(unittest.TestCase):
    """机械臂的运动学测试用例"""
    
    def setUp(self):
        self.robot_module = RobotArmModule()
    
    @ddt.data(
        [0, 0, 0, 0, 0]
    )
    def test_fk(self, angle_list: list):
        """测试机械臂的正运动学"""
        translation_vector = self.robot_module.fkine(np.radians(angle_list))
        x, y, z = np.round(translation_vector.t, 3)
        Rx, Py, Yz = np.round(translation_vector.rpy(order="zyx"), 3)
        
        self.assertEqual((x, y, z), (0.195, 0.0, 0.065))
        self.assertEqual((Rx, Py, Yz), (-3.142, -0.0, 0.0))
    
    @ddt.data(
        [0.195, 0.0, 0.065, 0, -0.0, 0.0]
    )
    def test_ik(self, translation_vector: list):
        """测试机械臂的逆运动学"""
        x, y, z, Rx, Py, Yz = translation_vector
        
        R_T = SE3([x, y, z]) * rpy2tr([Rx, Py, Yz], order="zyx")
        sol = self.robot_module.ikine_LM(R_T, joint_limits=True)
        self.assertTrue(sol.success)
        
        
        
if __name__ == "__main__":
    unittest.main()