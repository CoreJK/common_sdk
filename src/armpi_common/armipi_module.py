from math import radians, pi, degrees
import numpy as np
from roboticstoolbox import DHRobot, RevoluteMDH

'''
Modified DH
----------------------------------------------
i | α(i-1) | a(i-1) |       θ(i)      | d(i) |
----------------------------------------------
1 |   0°   |   0    |  θ1(-120, 120)  |   0  |
----------------------------------------------
2 |  -90°  |   0    |  θ2(-180, 0)    |   0  |
----------------------------------------------
3 |   0°   | link1  |  θ3(-120, 120)  |   0  |
----------------------------------------------
4 |   0°   | link2  |  θ4(-200, 20)   |   0  |
----------------------------------------------
5 |  -90°  |   0    |  θ5(-120, 120)  |   0  |
----------------------------------------------
'''

# 连杆长度(m)
# 底座的高度，这里把第一个坐标系和第二个坐标的原点重合到一起了
base_link = 0.064605

link1 = 0.10048
link2 = 0.094714

# 计算tool_link时取值为link3 + tool_link，因为把末端的坐标系原点和前一个重合到一起了
# 这里的tool_link指实际上的夹持器长度
link3 = 0.05071
tool_link = 0.1126

# 各关节角度限制，取决于是否碰撞以及舵机的转动范围
# 多加0.2为了防止计算时数值的不稳定，会比设定值大一点点
joint1 = [-120.2, 120.2]
joint2 = [-180.2, 0.2]
joint3 = [-120.2, 120.2]
joint4 = [-200.2, 20.2]
joint5 = [-120.2, 120.2]

#         舵机脉宽范围，中位值，对应的角度范围，中位值
joint1_map = [0, 1000, 500, -120, 120, 0]
joint2_map = [0, 1000, 500, 30, -210, -90]
joint3_map = [0, 1000, 500, -120, 120, 0]
joint4_map = [0, 1000, 500, 30, -210, -90]
joint5_map = [0, 1000, 500, -120, 120, 0]

# 等比例映射
def angle_transform(angle, param, inverse=False):
    """用于映射脉冲宽度和角度值"""
    if inverse:
        new_angle = ((angle - param[5]) / (param[4] - param[3])) * (param[1] - param[0]) + param[2]
    else:
        new_angle = ((angle - param[2]) / (param[1] - param[0])) * (param[4] - param[3]) + param[5]

    return new_angle

def pulse2angle(pulse):
    """将脉冲宽度转换为角度"""
    theta1 = angle_transform(pulse[0], joint1_map)
    theta2 = angle_transform(pulse[1], joint2_map)
    theta3 = angle_transform(pulse[2], joint3_map)
    theta4 = angle_transform(pulse[3], joint4_map)
    theta5 = angle_transform(pulse[4], joint5_map)
    
    return radians(theta1), radians(theta2), radians(theta3), radians(theta4), radians(theta5)

def angle2pulse(angle, convert_int=False):
    """将角度转换为脉冲宽度"""
    pluse = []
    
    for i in angle:
        theta1 = angle_transform(degrees(i[0]), joint1_map, True)
        theta2 = angle_transform(degrees(i[1]), joint2_map, True)
        theta3 = angle_transform(degrees(i[2]), joint3_map, True)
        theta4 = angle_transform(degrees(i[3]), joint4_map, True)
        theta5 = angle_transform(degrees(i[4]), joint5_map, True)
        
        #print(theta1, theta2, theta3, theta4, theta5)
        if convert_int:
            pluse.extend([[int(theta1), int(theta2), int(theta3), int(theta4), int(theta5)]])
        else:
            pluse.extend([[theta1, theta2, theta3, theta4, theta5]])

    return pluse

class RobotArmModule(DHRobot):
    """幻尔Armpi-fpv机械臂"""
    
    def __init__(self):
        L1 = RevoluteMDH(
            alpha = 0,
            d = 0,
            a = 0,
            offset = 0,
            qlim = (radians(-120), radians(120))
        )
        L2 = RevoluteMDH(
            alpha = radians(-90),
            d = 0,
            a = 0,
            offset = 0,
            qlim = (radians(-180), radians(0))
        )
        L3 = RevoluteMDH(
            alpha = 0,
            d = 0,
            a = link1,
            offset = 0,
            qlim = (radians(-120), radians(120))
        )
        L4 = RevoluteMDH(
            alpha = 0,
            d = 0,
            a = link2,
            offset = 0,
            qlim = (radians(-200), radians(20))
        )
        L5 = RevoluteMDH(
            alpha = radians(-90),
            d = 0,
            a = 0,
            offset = 0,
            qlim = (radians(-120), radians(120))
        )
        
        super().__init__(
            [L1, L2, L3, L4, L5],
            name="Armpi-fpv",
            manufacturer="任伟明",
            keywords=("Armpi", "Armpi-fpv", "Armpi-fpv-Robot", "Armpi-fpv-Robot-Arm"),
        )
        
        self._MYCONFIG = np.array([1, 2, 3, 4, 5])
        self.qr = np.radians([0, 0, 0, 0, 0])
        self.qz = np.radians([0, 0, 0, 0, 0])
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        
    @property
    def MYCONFIG(self):
        return self._MYCONFIG
    
    
if __name__ == "__main__":
    robot = RobotArmModule()
    print(robot)
    
    # 六轴机器人初始位置
    q1 = radians(0)
    q2 = radians(0)
    q3 = radians(0)
    q4 = radians(0)
    q5 = radians(0)

    # 机械臂画图
    robot.teach([q1, q2, q3, q4, q5], block=True)
        
        