import sys
import time
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from armpi_common.armipi_module import RobotArmModule

# Swift 后端
try:
    from swift import Swift
except Exception:
    from roboticstoolbox.backends.swift import Swift  # fallback

# 可选：轨迹点可视化
try:
    from spatialgeometry import Sphere
    HAS_GEOM = True
except Exception:
    HAS_GEOM = False

def plan_joint_traj(robot, q0, qf, N=200):
    # quintic（平滑到加加速度），也可用 rtb.lspb 生成时间标尺再采样
    traj = rtb.jtraj(q0, qf, N)
    return traj.q  # (N, n)

def plan_cart_traj(robot, q_start, T_goal, N=200, mask=None):
    """
    先用 q_start 得到 T0，再用 ctraj 生成一串 SE3，逐步 IK（带 joint_limits）。
    对于 5 DOF，mask 需给 5 个约束（默认只约束位置和偏航 yaw）。
    """
    if mask is None:
        # 约束 [x, y, z, roll, pitch, yaw] -> 约束 x,y,z 和 yaw，放开 roll/pitch
        mask = [1, 1, 1, 0, 0, 1]

    T0 = robot.fkine(q_start)
    Ts = rtb.ctraj(T0, T_goal, N)  # SE3 trajectory
    qs = []
    qk = q_start.copy()
    for k in range(N):
        sol = robot.ikine_LM(Ts[k], q0=qk, mask=mask, joint_limits=True)
        if not sol.success:
            break
        qk = sol.q
        qs.append(qk)
    return np.array(qs)  # (K, n)

def animate(robot, qs, step=0.02, draw_trail=True, trail_stride=5):
    env = Swift()
    env.launch()
    env.add(robot)

    trail_color = (0.0, 0.4, 1.0, 0.25)
    tip_color = (1.0, 0.0, 0.0, 1.0)
    if HAS_GEOM:
        tip_marker = Sphere(radius=0.004, color=tip_color)
        env.add(tip_marker)

    for i, q in enumerate(qs):
        robot.q = q
        T = robot.fkine(q)
        if HAS_GEOM:
            tip_marker.T = T
            if draw_trail and i % trail_stride == 0:
                env.add(Sphere(radius=0.0035, color=trail_color, pose=T))
        env.step(step)

    # 悬停可交互旋转缩放
    try:
        while True:
            env.step(0.05)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "joint"  # joint 或 cart
    N = int(sys.argv[2]) if len(sys.argv) > 2 else 400

    robot = RobotArmModule()

    # 关节空间示例：从 q0 到 qf（单位：度 -> 弧度）
    q0_deg = np.array([0, 0, 0, 0, 0], dtype=float)
    qf_deg = np.array([0, -90, 0, -90, 0], dtype=float)
    q0 = np.radians(q0_deg)
    qf = np.radians(qf_deg)

    if mode == "joint":
        qs = plan_joint_traj(robot, q0, qf, N=N)
        animate(robot, qs)
    else:
        # 笛卡尔空间示例：目标位姿 = 当前位置平移+旋转
        T_curr = robot.fkine(q0)
        T_goal = T_curr @ SE3.Tx(0.06) @ SE3.Ty(-0.03) @ SE3.Tz(0.04) @ SE3.Rz(np.radians(20))
        qs = plan_cart_traj(robot, q_start=q0, T_goal=T_goal, N=N, mask=[1,1,1,0,0,1])
        if len(qs) == 0:
            print("IK failed along path")
            sys.exit(1)
        animate(robot, qs)