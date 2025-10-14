from math import radians, degrees
import numpy as np

#         舵机脉宽范围，中位值，对应的角度范围，中位值
joint1_map = [0, 1000, 500, -120, 120, 0]
joint2_map = [0, 1000, 500, -210, 30, 0]
joint3_map = [0, 1000, 500, 120, -120, 0]
joint4_map = [0, 1000, 500, -210, 30, 0]
joint5_map = [0, 1000, 500, -120, 120, 0]
joint6_map = [0, 1000, 500, -120, 120, 0]

joint_map = {
    'joint1': joint1_map,
    'joint2': joint2_map,
    'joint3': joint3_map,
    'joint4': joint4_map,
    'joint5': joint5_map,
    'joint6': joint6_map
}

# 等比例映射
def angle_transform(angle, param, inverse=False):
    """用于映射脉冲宽度和角度值"""
    if inverse:
        new_angle = ((angle - param[5]) / (param[4] - param[3])) * (param[1] - param[0]) + param[2]
    else:
        new_angle = ((angle - param[2]) / (param[1] - param[0])) * (param[4] - param[3]) + param[5]

    return new_angle

def pulse2angle(pulse_list: list):
    """将脉冲宽度转换为角度"""
    theta1 = angle_transform(pulse_list[0], joint1_map)
    theta2 = angle_transform(pulse_list[1], joint2_map)
    theta3 = angle_transform(pulse_list[2], joint3_map)
    theta4 = angle_transform(pulse_list[3], joint4_map)
    theta5 = angle_transform(pulse_list[4], joint5_map)
    theta6 = angle_transform(pulse_list[5], joint6_map)
    
    return radians(theta1), radians(theta2), radians(theta3), radians(theta4), radians(theta5), radians(theta6)

def angle2pulse(angle_list: list, convert_int=False):
    """将多组角度转换为脉冲宽度

    :param list angles: 期望角度列表, 单位为弧度
    :param bool convert_int: 输出的角度是否转换为整数, defaults to False
    :return _type_: _description_
    """
    pluse = []
    
    for angle in angle_list:
        pulse_1 = angle_transform(degrees(angle[0]), joint1_map, True)
        pulse_2 = angle_transform(degrees(angle[1]), joint2_map, True)
        pulse_3 = angle_transform(degrees(angle[2]), joint3_map, True)
        pulse_4 = angle_transform(degrees(angle[3]), joint4_map, True)
        pulse_5 = angle_transform(degrees(angle[4]), joint5_map, True)
        
        # print(pulse_1, pulse_2, pulse_3, pulse_4, pulse_5)
        if convert_int:
            pluse.extend([[int(pulse_1), int(pulse_2), int(pulse_3), int(pulse_4), int(pulse_5)]])
        else:
            pluse.extend([[pulse_1, pulse_2, pulse_3, pulse_4, pulse_5]])

    return pluse
    
def single_angle2pulse(angle, joint_name, convert_int=False):
    """将单个角度转换为脉冲宽度"""
    pulse = angle_transform(degrees(angle), joint_map[joint_name], True)
    if convert_int:
        return int(pulse)
    else:
        return pulse

def single_pulse2angle(pulse, joint_name):
    """将单个脉冲宽度转换为角度"""
    angle = angle_transform(pulse, joint_map[joint_name], False)
    return radians(angle)

if __name__ == "__main__":
    # 机械臂关节的角度
    new_angle = [0, 0, 0, 0, 0]
    pulse_list = [500, 500, 500, 500, 500]
        
    # 计算 角度 <--> 脉冲 的换算关系
    # new_angle = np.degrees(pulse2angle(pulse_list)).tolist()
    # print(f"脉冲 {pulse_list} 转换得到的角度 {new_angle}")
    
    new_pluse = angle2pulse([np.radians(new_angle)], convert_int=True)
    print(f"角度 {new_angle} 转换成脉冲 {new_pluse}")
    