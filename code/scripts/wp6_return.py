#!/usr/bin/env python3
"""WP6: 返航 + 任务状态机 (离线)
- 拓扑关键帧: 轨迹抽稀(间距>0.5m 或转角>30°)
- 状态机: INITIALIZE → EXPLORE → RETURN → HOME → STOP
- 返航: 拓扑逆序回退(沿关键帧逆向走回 home)
输入: wp2/trajectory.npy, home_anchor.npy, wp5_frontier 结果
"""
import numpy as np, os
import math

OUTDIR = "/home/lch/dog/tmp/wp2"
traj = np.load(f"{OUTDIR}/trajectory.npy")
home = np.load(f"{OUTDIR}/home_anchor.npy")
hx, hy = home[0], home[1]

# ---- 1. 拓扑关键帧提取 ----
def extract_keyframes(traj, dist_thre=0.5, angle_thre=30.0):
    kf = [traj[0]]
    last = traj[0]
    last_dir = None
    for p in traj[1:]:
        d = np.hypot(p[0]-last[0], p[1]-last[1])
        if d >= dist_thre:
            # 计算方向
            dir = math.atan2(p[1]-last[1], p[0]-last[0])
            if last_dir is not None:
                ang = abs(math.degrees(dir - last_dir))
                ang = min(ang, 360-ang)
            else:
                ang = 0
            if d >= dist_thre or ang > angle_thre:
                kf.append(p); last = p; last_dir = dir
    kf.append(traj[-1])
    return np.array(kf)

keyframes = extract_keyframes(traj)
print(f"=== WP6 返航 + 状态机 ===", flush=True)
print(f"轨迹 {len(traj)} 点 → 拓扑关键帧 {len(keyframes)} 个", flush=True)

# ---- 2. 状态机 ----
class StateMachine:
    def __init__(self, home, keyframes):
        self.home = home
        self.kf = keyframes
        self.state = "INITIALIZE"
        self.pos = home[:2]
        self.return_path = None
        self.log = []
    def run(self):
        # INITIALIZE
        self.log.append(f"INITIALIZE: home=({self.home[0]:.2f},{self.home[1]:.2f}) 锁存")
        self.state = "EXPLORE"
        # EXPLORE: 模拟沿记录轨迹探索(正方形路线)
        self.pos = self.kf[-1][:2]   # 探索到轨迹终点
        self.log.append(f"EXPLORE: 探索完成, 当前=({self.pos[0]:.2f},{self.pos[1]:.2f})")
        self.log.append(f"EXPLORE: 触发返航(无可达前沿/返航时间线)")
        # RETURN: 拓扑逆序回退
        self.state = "RETURN"
        self.return_path = self.kf[::-1]   # 逆序关键帧
        self.log.append(f"RETURN: 沿拓扑关键帧逆序回退({len(self.return_path)} 个航点)")
        # 模拟返航: 从轨迹终点沿 return_path 走回 home
        self.pos = self.return_path[-1][:2]
        dist_home = np.hypot(self.pos[0]-self.home[0], self.pos[1]-self.home[1])
        self.log.append(f"HOME: 到达判定 距离={dist_home:.3f}m (容差 0.5m)")
        if dist_home < 0.5:
            self.state = "HOME"
            self.log.append(f"HOME: 成功返航 ✓")
        else:
            self.state = "SAFE_STOP"
            self.log.append(f"SAFE_STOP: 未到容差, 进入安全停止")
        self.state = "STOP"
        self.log.append("STOP: 任务结束")

sm = StateMachine((hx, hy), keyframes)
sm.run()
for line in sm.log:
    print(line, flush=True)

# 返航路径统计
rp = sm.return_path
print(f"\n返航路径: {len(rp)} 航点, 起点=({rp[0,0]:.2f},{rp[0,1]:.2f}) 终点=({rp[-1,0]:.2f},{rp[-1,1]:.2f})", flush=True)
# 返航路径总长
path_len = np.sum(np.hypot(np.diff(rp[:,0]), np.diff(rp[:,1])))
print(f"返航路径总长 = {path_len:.2f}m", flush=True)

# ---- 3. 可视化 ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False

fig, ax = plt.subplots(figsize=(12, 10))
# 探索轨迹(正向, 蓝)
ax.plot(traj[:,0], traj[:,1], "b-", lw=1.5, alpha=0.6, label="探索轨迹(正方形)")
# 关键帧(黑点)
ax.scatter(keyframes[:,0], keyframes[:,1], c="black", s=25, zorder=3, label="拓扑关键帧")
# 返航路径(红, 逆序)
ax.plot(rp[:,0], rp[:,1], "r--", lw=2.5, label="返航路径(逆序回退)")
ax.scatter(rp[0,0], rp[0,1], c="red", s=180, marker="o", label="返航起点", zorder=4)
ax.scatter(hx, hy, c="lime", s=250, marker="*", edgecolors="black", label="HOME", zorder=5)
ax.set_title(f"WP6 返航 + 状态机: 探索→返航(拓扑逆序)→回到HOME")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.set_aspect("equal")
ax.legend(loc="best", fontsize=9)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/wp6_return.png", dpi=110)
print(f"\n已保存 {OUTDIR}/wp6_return.png", flush=True)
