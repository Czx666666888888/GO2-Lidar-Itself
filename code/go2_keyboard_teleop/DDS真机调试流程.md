# Go2 DDS 链路真机调试流程

用 `go2_keyboard_teleop_dds` 走官方 DDS（CycloneDDS + 网线）控制狗。
**核心优势**：狗固件(sdk2)直接支持 DDS，无需在狗上部署 ROS2，网线直连即可。

---

## 0. 前置条件

- 狗：**Go2 EDU/PRO 版**（sdk2 固件，底层支持 DDS），上电
- 电脑：ROS2 Humble + unitree_ros2 消息包（`go2_ws` 已编译好）
- 网线一根

> 关键理解：DDS 链路**不需要** `ROBOT_IP`/`ROBOT_AES_KEY`（那是 WebRTC 的），
> 也不需要 WiFi——靠网线 + CycloneDDS 直接和狗固件通信。

---

## 1. 网络配置（一次性）

1. 网线直连狗和电脑
2. 查网卡名：
   ```bash
   ip addr    # 找到连狗的以太网卡, 如 enp3s0 / enx... / eth0
   ```
3. 给该网卡设静态 IP（官方约定狗在 `192.168.123.x` 网段）：
   ```bash
   # 图形界面: 网络设置 → 该网卡 → IPv4 → 手动
   #   地址: 192.168.123.99   掩码: 255.255.255.0
   # 或命令行(临时):
   sudo ip addr add 192.168.123.99/24 dev <网卡名>
   ```
4. 验证：`ping 192.168.123.161`（Go2 以太网口常见默认 IP，若不通查狗实际 IP）

---

## 2. 环境 setup（每个终端都要执行）

```bash
cd /home/lch/dog/go2_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="网卡名" priority="default" multicast="default" /></Interfaces></General></Domain></CycloneDDS>'
```

> 把 `网卡名` 换成第 1 步查到的实际网卡名。
> **注意：不要设 `ROS_LOCALHOST_ONLY=1`**（那会阻断和狗的通信，只用于离线隔离）。

---

## 3. 连接测试（先确认通信，再控制）

### 3.1 能看到狗的话题

```bash
ros2 topic list
```

**成功标志**：能看到 `/api/sport/request`、`/sportmodestate`、`/lowstate`、`/utlidar/cloud` 等。

### 3.2 能收到狗的状态

```bash
ros2 topic echo /sportmodestate
ros2 topic echo /lowstate
```

**成功标志**：数据持续刷新（SportModeState 含 position/velocity/imu_state）。

> 若 3.1 空列表：检查网卡名、IP、`ROS_DOMAIN_ID`（官方默认 0）、网线接触。

---

## 4. 键盘遥控（DDS）

```bash
cd /home/lch/dog/go2_ws && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS>...网卡...</CycloneDDS>'
ros2 run go2_keyboard_teleop go2_keyboard_teleop_dds
```

| 按键 | 动作 | api_id |
|---|---|---|
| `w/s/a/d/q/e` | 前后/平移/转向（长按运行松手停） | 1008 Move |
| `空格` | 急停 | Move 0 |
| `b` / `c` / `v` | 平衡站立 / 坐下 / 起立 | 1002/1009/1004 |
| `g` | 切换步态 | 1011 |
| `x` | 退出 | — |

---

## 5. 分步调试（建议按序验证）

1. **站立**：按 `v`（StandUp）→ 狗起立（若已趴下）
2. **低速前进**：`ros2 run go2_keyboard_teleop go2_keyboard_teleop_dds --ros-args -p max_vx:=0.15 -p max_vyaw:=0.2`，按住 `w` → 狗慢速前进，松手停
3. **转向**：按住 `q`/`e` → 原地转
4. **姿态**：按 `c` 坐下、`v` 起立、`b` 平衡站立
5. **急停**：移动中按空格 → 立即停

每步若狗无反应，另开终端监听确认指令已发出：
```bash
ros2 topic echo /api/sport/request   # 按 w 应看到 api_id=1008 + {"x":...}
```

---

## 6. 常见问题

| 现象 | 排查 |
|---|---|
| `ros2 topic list` 空 | 网卡名/IP 错、网线松、`ROS_DOMAIN_ID` 不一致、忘了去 `ROS_LOCALHOST_ONLY` |
| 有话题但狗不动 | 先按 `v` 起立；确认狗已上电且非低电量；速度参数 >0 |
| 指令发出但狗不执行 | 监听 `/api/sport/request` 确认 api_id=1008；查 `/sportmodestate` 的 error_code |
| ping 不通狗 | 电脑静态 IP 没生效；狗以太网口 IP 不是 192.168.123.161（查狗实际 IP） |

---

## 7. 与 WebRTC 链路对比（真机）

| | DDS（本流程） | WebRTC |
|---|---|---|
| 连接 | 网线直连 | WiFi + AES key |
| 前置 | 电脑静态 IP 192.168.123.99 | 狗 WiFi 模式 + ROBOT_IP |
| 节点 | `go2_keyboard_teleop_dds` | `go2_keyboard_teleop_node` |
| 命令 | 速度 + 姿态键 | 只速度 |
| 延迟 | 低（网线） | 较高（WiFi） |
