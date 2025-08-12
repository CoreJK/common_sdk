import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless backend for non-interactive environments
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from armpi_common.armipi_module import RobotArmModule
# 可选：若要把抓手（link3+tool_link）计入末端，可设置 tool 偏置（方向按你的 MDH 建系来）
# from spatialmath import SE3
# robot.tool = SE3.Tx(link3 + tool_link)  # 或 SE3.Tz(...)

robot = RobotArmModule()
n = robot.n                     # 关节数
qlim = robot.qlim               # 可能为 (n,2) 或 (2,n)，弧度
# 统一为形状 (n, 2)
qlim = np.asarray(qlim, dtype=float)
if qlim.shape == (2, n):
    qlim = qlim.T
elif qlim.shape != (n, 2):
    raise ValueError(f"Unexpected qlim shape: {qlim.shape}, expected (n,2) or (2,n) with n={n}")
N = 20000                       # 采样数量，按需要增减

# Monte Carlo 采样关节角（弧度）
low = qlim[:, 0]
high = qlim[:, 1]
qs = np.random.uniform(low=low, high=high, size=(N, n))

# 计算末端位姿并取位置
points = []
for q in qs:
    T = robot.fkine(q)          # SE3
    points.append(T.t)          # (x, y, z)
points = np.vstack(points)      # (N, 3)

# 可视化散点
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(points[:,0], points[:,1], points[:,2], s=1, alpha=0.1)
ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
ax.set_title('Reachable workspace (Monte Carlo)')
plt.savefig('workspace_scatter.png', dpi=150)
plt.close(fig)

# 可选：计算凸包近似外形（注意：非凸空间会被过度外插）
hull = ConvexHull(points)
print(f'Workspace convex-hull volume ≈ {hull.volume:.6f} m^3, vertices={len(hull.vertices)}')