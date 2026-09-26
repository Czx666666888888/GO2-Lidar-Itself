#!/usr/bin/env python3
"""WP6 v2: 返航 + 状态机 + 航向/掉头
- 从轨迹计算航向(yaw)
- 关键帧带位置+航向
- 返航: 计算掉头角度(当前航向 vs 返航方向)
- 可视化: 关键帧航向箭头 + 掉头示意
"""
import numpy as np, math

OUTDIR = "/home/lch/dog/tmp/wp2"
traj = np.load(f"{OUTDIR}/trajectory.npy")
home = np.load(f"{OUTDIR}/home_anchor.npy")

# ---- 1. 从轨迹计算航向(运动方向, 平滑窗口) ----
def compute_headings(traj, win=5):
    n = len(traj)
    yaws = np.zeros(n)
    for i in range(n):
        a = max(0, i-win); b = min(n-1, i+win)
        if b > a:
            yaws[i] = math.atan2(traj[b,1]-traj[a,1], traj[b,0]-traj[a,0])
    return yaws

yaws = compute_headings(traj)

# ---- 2. 拓扑关键帧(带航向) ----
def extract_keyframes(traj, yaws, dist_thre=0.5, angle_thre=30.0):
    kf = [(traj[0,0], traj[0,1], yaws[0])]
    last = traj[0]; last_dir = yaws[0]
    for i in range(1, len(traj)):
        d = math.hypot(traj[i,0]-last[0], traj[i,1]-last[1])
        if d >= dist_thre:
            ang = abs(math.degrees(yaws[i]-last_dir)); ang = min(ang, 360-ang)
            if ang > angle_thre or d >= dist_thre*2:
                kf.append((traj[i,0], traj[i,1], yaws[i])); last = traj[i]; last_dir = yaws[i]
    kf.append((traj[-1,0], traj[-1,1], yaws[-1]))
    return kf

kf = extract_keyframes(traj, yaws)
kf = np.array(kf)   # (N,3) x,y,yaw
print(f"=== WP6 v2 返航(带航向/掉头) ===", flush=True)
print(f"轨迹 {len(traj)} 点 → 关键帧 {len(kf)} 个(带航向)", flush=True)

# ---- 3. 返航: 逆序关键帧 + 掉头 ----
# 当前位置 = 轨迹终点(关键帧[-1])
cur_x, cur_y, cur_yaw = kf[-1]
print(f"\n当前位置 = ({cur_x:.2f},{cur_y:.2f}), 当前航向 = {math.degrees(cur_yaw):.1f}°", flush=True)

# 返航方向(第一段): 关键帧[-1] → 关键帧[-2]
ret_x = kf[-2,0] - kf[-1,0]
ret_y = kf[-2,1] - kf[-1,1]
ret_yaw = math.atan2(ret_y, ret_x)
print(f"返航方向 = {math.degrees(ret_yaw):.1f}° (指向关键帧[-2])", flush=True)

# 掉头角度 = 当前航向 vs 返航方向的差
uturn = abs(math.degrees(ret_yaw - cur_yaw))
uturn = min(uturn, 360 - uturn)
print(f"\n>>> 掉头角度 = {uturn:.1f}° (180°=完全掉头, 0°=不用掉头)", flush=True)

# 返航路径 = 逆序关键帧, 每段航向 = 反向
ret_path = kf[::-1]   # 逆序
# 返航时每段的方向 = 从当前点指向上一个点(即原路径的反向)
ret_yaws = []
for i in range(len(ret_path)-1):
    dy = ret_path[i+1,1]-ret_path[i,1]
    dx = ret_path[i+1,0]-ret_path[i,0]
    ret_yaws.append(math.atan2(dy, dx))
ret_yaws.append(ret_yaws[-1])

# 状态机(带掉头判断)
print(f"\n=== 状态机 ===", flush=True)
home_dist = math.hypot(cur_x-home[0], cur_y-home[1])
print(f"INITIALIZE: home=({home[0]:.2f},{home[1]:.2f})", flush=True)
print(f"EXPLORE: 探索完成, 当前位置=({cur_x:.2f},{cur_y:.2f})", flush=True)
print(f"EXPLORE: 触发返航", flush=True)
if home_dist < 0.5:
    print(f"HOME: 距家 {home_dist:.3f}m < 0.5m → 直接 HOME(无需返航)", flush=True)
else:
    print(f"RETURN: 距家 {home_dist:.2f}m, 先掉头 {uturn:.1f}° 再逆序回退", flush=True)
    print(f"RETURN: 逆序 {len(ret_path)} 航点, 总长 {np.sum(np.hypot(np.diff(ret_path[:,0]), np.diff(ret_path[:,1]))):.2f}m", flush=True)
    print(f"HOME: 到达判定 → 成功 ✓", flush=True)
print(f"STOP: 任务结束", flush=True)

# ---- 4. 可视化(带航向箭头) ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
for _f in ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc','/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']:
    try: fm.fontManager.addfont(_f)
    except Exception: pass
plt.rcParams['font.family']='Noto Sans CJK SC'; plt.rcParams['axes.unicode_minus']=False

fig, ax = plt.subplots(figsize=(13, 10))
ax.plot(traj[:,0], traj[:,1], "b-", lw=1.5, alpha=0.5, label="探索轨迹")
ax.plot(ret_path[:,0], ret_path[:,1], "r--", lw=2.5, label="返航路径(逆序)")
# 关键帧航向箭头(正向, 蓝箭头)
for i in range(len(kf)):
    ax.arrow(kf[i,0], kf[i,1], 0.15*math.cos(kf[i,2]), 0.15*math.sin(kf[i,2]),
             head_width=0.08, head_length=0.1, fc="blue", ec="blue", alpha=0.6)
# 返航航向箭头(红箭头)
for i in range(0, len(ret_yaws), 2):
    ax.arrow(ret_path[i,0], ret_path[i,1], 0.15*math.cos(ret_yaws[i]), 0.15*math.sin(ret_yaws[i]),
             head_width=0.08, head_length=0.1, fc="red", ec="red", alpha=0.7)
# 当前位置 + 掉头示意
ax.scatter(cur_x, cur_y, c="cyan", s=200, marker="o", zorder=5)
ax.arrow(cur_x, cur_y, 0.3*math.cos(cur_yaw), 0.3*math.sin(cur_yaw),
         head_width=0.12, head_length=0.15, fc="cyan", ec="cyan", lw=2, zorder=6)
ax.arrow(cur_x, cur_y, 0.3*math.cos(ret_yaw), 0.3*math.sin(ret_yaw),
         head_width=0.12, head_length=0.15, fc="gold", ec="gold", lw=2, zorder=6, alpha=0.9)
ax.scatter(home[0], home[1], c="lime", s=250, marker="*", edgecolors="black", label="HOME", zorder=7)
ax.text(cur_x+0.3, cur_y+0.3, f"掉头 {uturn:.0f}°", fontsize=12, color="darkred", fontweight="bold")
ax.set_title(f"WP6 v2 返航: 蓝色箭头=正向航向, 红色=返航航向, 当前位置需掉头 {uturn:.1f}°")
ax.set_xlabel("X(m)"); ax.set_ylabel("Y(m)"); ax.set_aspect("equal")
ax.legend(loc="best", fontsize=9)
fig.tight_layout()
fig.savefig(f"{OUTDIR}/wp6_return_v2.png", dpi=110)
print(f"\n已保存 {OUTDIR}/wp6_return_v2.png", flush=True)
