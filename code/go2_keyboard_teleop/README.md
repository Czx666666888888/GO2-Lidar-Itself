# go2_keyboard_teleop

纯键盘控制 Unitree Go2 移动的 ROS 2 包。**长按运行、松手停止**，只发布 `Twist` 速度指令，不含任何路径规划 / 建图逻辑。

## 依赖

- `rclpy`、`geometry_msgs`（RoboStack ROS 2 Humble 自带）
- 可选：`go2_robot_sdk`（提供 `/cmd_vel_out` 订阅，把速度指令经 WebRTC 下发到狗）

## 构建

```bash
cd /home/lch/dog/go2_ws
colcon build --packages-select go2_keyboard_teleop
source install/setup.bash
```

## 运行

```bash
ros2 run go2_keyboard_teleop go2_keyboard_teleop_node
# 或带参数
ros2 run go2_keyboard_teleop go2_keyboard_teleop_node \
  --ros-args -p max_vx:=0.8 -p max_vyaw:=1.0
```

## 按键映射（长按运行 / 松手停止）

| 按键 | 动作 | Twist 字段 |
|---|---|---|
| `w` / `s` | 前进 / 后退（满速） | `linear.x = ±max_vx` |
| `a` / `d` | 左移 / 右移（满速） | `linear.y = ±max_vy` |
| `q` / `e` | 左转 / 右转（满速） | `angular.z = ±max_vyaw` |
| `空格` | 急停 | — |
| `x` 或 `Ctrl-C` | 先清零再退出 | — |

**按住按键即朝对应方向满速运行，松开立即停止**（松手检测：约 300ms 无新按键即归零）。
支持组合按键，例如按住 `w`+`e` 即前进 + 右转。速度上限由参数控制（默认 `max_vx=0.6`、`max_vy=0.4`、`max_vyaw=0.8`），定时器按 `publish_rate=20Hz` 持续发布。

## 发布话题

| 话题 | 用途 |
|---|---|
| `cmd_vel_out` | 给 `go2_robot_sdk` 的 `go2_driver_node`（WebRTC 下发到狗） |
| `cmd_vel` | 标准话题，便于 `ros2 topic echo /cmd_vel` 调试 |

## 与 go2_robot_sdk 配合（真机）

先起驱动，再起本节点：

```bash
# 终端 1: 驱动(连狗, WebRTC)
ros2 launch go2_robot_sdk go2_driver_node.launch.py

# 终端 2: 键盘遥操
ros2 run go2_keyboard_teleop go2_keyboard_teleop_node
```

## 纯离线验证（无狗）

```bash
# 终端 1
ros2 run go2_keyboard_teleop go2_keyboard_teleop_node
# 终端 2: 观察速度指令
ros2 topic echo /cmd_vel
```

无 TTY 环境（如后台/CI）会自动降级为静默模式，仅持续发布零速度。

## 记录回传数据（IMU + 激光雷达）

`go2_sensor_recorder` 在控制狗的同时记录回传数据：

| 话题 | 消息类型 | 保存文件 |
|---|---|---|
| `imu` | sensor_msgs/Imu | `imu.csv`（时间戳/线加速度/角速度/四元数） |
| `point_cloud2` | sensor_msgs/PointCloud2 | `lidar.npz`（xyz/时间戳/intensity） |

```bash
# 终端 3: 记录数据(与控制同时进行), Ctrl-C 停止并保存
ros2 run go2_keyboard_teleop go2_sensor_recorder --ros-args -p outdir:=/home/lch/dog/go2_data

# 或定时自动记录 60 秒后自动保存退出
ros2 run go2_keyboard_teleop go2_sensor_recorder --ros-args -p outdir:=/home/lch/dog/go2_data -p duration:=60.0
```

激光雷达用 best_effort QoS 订阅（与 go2_driver_node 匹配）。数据累积在内存，停止时统一落盘，适合数分钟遥控记录。

## 节点总览

| 节点 | 功能 |
|---|---|
| `go2_keyboard_teleop_node` | 键盘遥控（长按运行/松手停止） |
| `go2_cmd_monitor` | 实时显示下发指令 |
| `go2_sensor_recorder` | 记录 IMU + 激光雷达 |
