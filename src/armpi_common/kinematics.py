# -*- coding: utf-8 -*-
# kinematics.py - 机械臂运动学计算模块

import numpy as np
from spatialmath import SE3
from spatialmath.base import rpy2tr
from typing import List, Optional, Dict, Any, Union

from armpi_common.armipi_module import RobotArmModule, angle2pulse, pulse2angle
from armpi_common._log import logger
from armpi_common.utils import is_flat


class ArmKinematics:
    """机械臂运动学计算类
    
    提供正运动学、逆运动学和轨迹规划功能，将运动学计算从控制逻辑中分离出来。
    支持独立实例化，可用于仿真和规划场景。
    """
    
    def __init__(self, robot_arm_module: Optional[RobotArmModule] = None):
        """初始化运动学计算模块
        
        Args:
            robot_arm_module: 机器人模型实例，包含DH参数和运动学计算方法。
                             如果为None，将创建默认实例。
        """
        if robot_arm_module is None:
            self.robot_arm_module = RobotArmModule()
        else:
            self.robot_arm_module = robot_arm_module
        
        logger.info("ArmKinematics module initialized successfully")
    
    def forward_kinematics(self, joint_position_list: List[Union[int, float]]) -> Dict[str, Any]:
        """计算正运动学
        
        根据给定的关节角度计算末端执行器的位姿。
        
        Args:
            joint_position_list: 关节角度列表，单位为弧度或脉冲值（根据输入类型自动判断）
                                长度必须为5
        
        Returns:
            Dict containing:
                - status (bool): 计算是否成功
                - fkine (List[float] or None): 末端位姿 [x, y, z, rx, ry, rz]
                - info (str): 状态信息
        """
        logger.info("正运动学计算")
        
        # 输入验证
        if not is_flat(joint_position_list):
            logger.error("关节角度列表不能嵌套, 只能是一维列表")
            return {
                "status": False,
                "fkine": None,
                "info": "关节角度列表不能嵌套, 只能是一维列表"
            }
        
        if len(joint_position_list) != 5:
            logger.error(f"关节角度列表长度为 {len(joint_position_list)}，预期为 5")
            return {
                "status": False,
                "fkine": None,
                "info": f"关节角度列表长度为 {len(joint_position_list)}，预期为 5"
            }
        
        # 检查是否为脉冲值（整数）还是角度值（浮点数）
        if all(isinstance(x, int) for x in joint_position_list):
            # 脉冲值转换为角度
            position_list = pulse2angle(joint_position_list)
        else:
            position_list = joint_position_list
        
        if not all(isinstance(x, (int, float)) for x in position_list):
            logger.error("关节角度列表中的元素必须是数值类型")
            return {
                "status": False,
                "fkine": None,
                "info": "关节角度列表中的元素必须是数值类型"
            }
        
        try:
            # 调用roboticstoolbox计算正运动学
            translation_vector = self.robot_arm_module.fkine(position_list)
            x, y, z = translation_vector.t  # 平移向量
            Rx, Py, Yz = translation_vector.rpy(order="zyx")  # 旋转角
            
            return {
                "status": True,
                "fkine": [x, y, z, Rx, Py, Yz],
                "info": "正运动学计算成功"
            }
        except Exception as e:
            logger.error(f"正运动学计算失败: {str(e)}")
            return {
                "status": False,
                "fkine": None,
                "info": f"正运动学计算失败: {str(e)}"
            }
    
    def inverse_kinematics(self, end_tool_coordinate_list: List[float]) -> Dict[str, Any]:
        """计算逆运动学
        
        根据给定的末端执行器位姿计算关节角度。
        
        Args:
            end_tool_coordinate_list: 末端位姿列表 [x, y, z, rx, ry, rz]
                                     位置单位为米，角度单位为弧度
        
        Returns:
            Dict containing:
                - status (bool): 计算是否成功
                - ikine (List[int] or None): 关节脉冲值列表
                - info (str): 状态信息
        """
        logger.info("逆运动学计算")
        
        # 输入验证
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
        
        try:
            x, y, z, Rx, Py, Yz = end_tool_coordinate_list
            
            # 构建目标变换矩阵
            R_T = SE3([x, y, z]) * rpy2tr([Rx, Py, Yz], order="zyx")
            
            # 使用Levenberg-Marquardt算法求解逆运动学
            sol = self.robot_arm_module.ikine_LM(R_T, joint_limits=True)
            
            if sol.success:
                # 将弧度结果转换为脉冲值
                inverse_result = np.round(sol.q, 6).tolist()
                joint_pulse = angle2pulse([inverse_result], convert_int=True)
                return {
                    "status": True,
                    "ikine": joint_pulse[0],  # 兼容关节转脉冲函数的返回格式
                    "info": "逆运动学计算成功"
                }
            else:
                logger.error("逆运动学求解失败")
                return {
                    "status": False,
                    "ikine": None,
                    "info": "逆运动学求解失败"
                }
        except Exception as e:
            logger.error(f"逆运动学计算失败: {str(e)}")
            return {
                "status": False,
                "ikine": None,
                "info": f"逆运动学计算失败: {str(e)}"
            }
    
    def plan_trajectory(self, start_coordinate_list: List[float], 
                       end_coordinate_list: List[float],
                       duration_ms: int = 2000, 
                       steps: int = 60,
                       mask: Optional[List[int]] = None) -> Dict[str, Any]:
        """规划轨迹
        
        在两个末端位姿之间生成平滑插值轨迹。
        
        Args:
            start_coordinate_list: 起点位姿 [x,y,z,roll,pitch,yaw]，单位 m / rad
            end_coordinate_list: 终点位姿 [x,y,z,roll,pitch,yaw]，单位 m / rad  
            duration_ms: 总时长（毫秒）
            steps: 轨迹离散步数
            mask: 逆解掩码，长度6，默认仅约束 [x,y,z,yaw] -> [1,1,1,0,0,1]
            
        Returns:
            Dict containing:
                - status (bool): 规划是否成功
                - trajectory (List[List[int]] or None): 轨迹点列表，每个点包含5个关节的脉冲值
                - steps (int): 实际生成的步数
                - info (str): 状态信息
        """
        logger.info("轨迹规划计算")
        
        # 输入验证
        for name, coord in (("start", start_coordinate_list), ("end", end_coordinate_list)):
            if not is_flat(coord):
                logger.error(f"{name} 坐标列表不能嵌套, 只能是一维列表")
                return {"status": False, "trajectory": None, "steps": 0, 
                       "info": f"{name} 坐标列表不能嵌套, 只能是一维列表"}
            if len(coord) != 6:
                logger.error(f"{name} 坐标列表长度为 {len(coord)}，预期为 6")
                return {"status": False, "trajectory": None, "steps": 0,
                       "info": f"{name} 坐标列表长度为 {len(coord)}，预期为 6"}
            if not all(isinstance(x, (int, float)) for x in coord):
                logger.error(f"{name} 坐标列表中的元素必须是数值类型")
                return {"status": False, "trajectory": None, "steps": 0,
                       "info": f"{name} 坐标列表中的元素必须是数值类型"}

        if mask is None:
            mask = [1, 1, 1, 1, 1, 0]
        if len(mask) != 6:
            return {"status": False, "trajectory": None, "steps": 0, 
                   "info": "mask 长度需为 6"}
        if steps <= 0:
            return {"status": False, "trajectory": None, "steps": 0, 
                   "info": "steps 必须 > 0"}
        if duration_ms < 0:
            return {"status": False, "trajectory": None, "steps": 0, 
                   "info": "duration_ms 不能为负"}

        try:
            # 拆包起止位姿
            sx, sy, sz, sR, sP, sY = start_coordinate_list
            ex, ey, ez, eR, eP, eY = end_coordinate_list

            # 先用起点作为初值求出初始关节解，提升后续迭代稳定性
            T_start = SE3([sx, sy, sz]) * rpy2tr([sR, sP, sY], order="zyx")
            sol0 = self.robot_arm_module.ikine_LM(T_start, joint_limits=True)
            if not sol0.success:
                logger.error("起点逆解失败，无法开始轨迹规划")
                return {"status": False, "trajectory": None, "steps": 0, 
                       "info": "起点逆解失败"}
            qk = sol0.q

            trajectory = []
            steps_done = 0
            
            for k in range(1, steps + 1):
                t = k / float(steps)
                # 线性插值位置与姿态
                x = sx + (ex - sx) * t
                y = sy + (ey - sy) * t
                z = sz + (ez - sz) * t
                Rr = sR + (eR - sR) * t
                Rp = sP + (eP - sP) * t
                Ry = sY + (eY - sY) * t

                T_goal = SE3([x, y, z]) * rpy2tr([Rr, Rp, Ry], order="zyx")
                sol = self.robot_arm_module.ikine_LM(T_goal, q0=qk, mask=mask, joint_limits=True)
                if not sol.success:
                    logger.error(f"第 {k}/{steps} 步逆解失败，提前结束")
                    break
                qk = sol.q

                # 角度(弧度) -> 脉冲
                joint_pulse = angle2pulse([qk.tolist()], convert_int=True)[0]
                trajectory.append(joint_pulse)
                steps_done += 1

            ok = steps_done == steps
            info = "轨迹规划完成" if ok else f"仅完成 {steps_done}/{steps} 步"
            return {"status": ok, "trajectory": trajectory, "steps": steps_done, "info": info}
            
        except Exception as e:
            logger.error(f"轨迹规划计算失败: {str(e)}")
            return {"status": False, "trajectory": None, "steps": 0,
                   "info": f"轨迹规划计算失败: {str(e)}"}