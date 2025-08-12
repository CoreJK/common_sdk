import sys
import time
import numpy as np

from armpi_common.armipi_module import RobotArmModule

# Swift 可视化后端的兼容导入
try:
    import swift  # noqa: F401
    from swift import Swift
except Exception:
    try:
        from roboticstoolbox.backends.swift import Swift  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "Swift backend not available. Install with: pip install swift-sim trimesh networkx"
        ) from e

try:
    from spatialgeometry import Sphere
except Exception as e:
    raise RuntimeError(
        "spatialgeometry not available. Install robotics-toolbox with extras or: pip install spatialgeometry"
    ) from e

from spatialmath import SE3


def sample_joint_space(robot: RobotArmModule, num_samples: int) -> np.ndarray:
    n = robot.n
    qlim = np.asarray(robot.qlim, dtype=float)
    if qlim.shape == (2, n):
        qlim = qlim.T
    elif qlim.shape != (n, 2):
        raise ValueError(f"Unexpected qlim shape: {qlim.shape}, expected (n,2) or (2,n) with n={n}")

    low = qlim[:, 0]
    high = qlim[:, 1]
    return np.random.uniform(low=low, high=high, size=(num_samples, n))


def show_interactive_workspace(
    num_samples: int = 5000,
    sphere_radius_m: float = 0.004,
    trail_stride: int = 20,
    step_seconds: float = 0.02,
    include_tool: bool = False,
):
    robot = RobotArmModule()

    # 如需把夹爪长度计入末端点位，启用 tool 偏置
    if include_tool:
        # 注意：具体方向取决于你的末端坐标系定义（此处假设沿 +x）
        try:
            from armpi_common.armipi_module import link3, tool_link
            robot.tool = SE3.Tx(link3 + tool_link)
        except Exception:
            pass

    env = Swift()
    env.launch()
    env.add(robot)

    # 末端动态标记点
    tip_marker = Sphere(radius=sphere_radius_m, color=(1.0, 0.0, 0.0, 1.0))
    env.add(tip_marker)

    qs = sample_joint_space(robot, num_samples)

    for i, q in enumerate(qs):
        robot.q = q
        T = robot.fkine(q)
        tip_marker.T = T

        # 每隔若干步留下轨迹点（半透明）
        if i % trail_stride == 0:
            trail = Sphere(radius=sphere_radius_m * 0.9, color=(0.0, 0.3, 1.0, 0.25), pose=T)
            env.add(trail)

        env.step(step_seconds)

    # 停留在场景中，可交互旋转/缩放浏览
    try:
        while True:
            env.step(0.05)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    # 可通过命令行参数调整采样数量，例如：python interactive_workspace_swift.py 8000
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 5000
    show_interactive_workspace(num_samples=N, include_tool=False)

