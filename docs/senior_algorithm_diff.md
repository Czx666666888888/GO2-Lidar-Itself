# Senior Algorithm Package Comparison

调查日期：2026-09-28。除特别注明外，结论来自 Baseline A 与压缩包解包内容的静态逐文件比较；没有启动 ROS graph、离线回放或真机。

## Source Package

- 本地来源：仓库根目录未跟踪文件 `实时SLAM与自主导航源码.tar.gz`
- SHA-256：`d0ebf562d73955b3b070175ee027966abbd0d55e1da0e61240d8c495ceca4d5f`
- 压缩包顶层目录：`cp0904/`
- 包内自述版本：`Go2 自主导航代码包 (2026-09-04)`，声称是在 2026-08-31 打包版上的增量修改
- 内容形态：完整的 ROS 2 源码子集/部署包，不是完整 Git 工程。包含 24 个 package、`go2_keyboard_teleop`、脚本和包级 README；不包含仓库级 Git 历史及本项目的协作文档。
- 文件统计：忽略 `__pycache__`/`.pyc` 后 543 个文件；与 Baseline A 的 `code/` 比较，3 个新增、6 个仅 Baseline A 存在、9 个同路径文件内容不同，其余同路径文件相同。

## Baseline A

- 分支：`main`
- HEAD：`ace88b66f14951dcc5ee3c0b312f6a0ec690048b`（`docs: inventory ROS2 interfaces and data flow`）
- 含义：当前仓库旧 baseline，也是 `retest4/5/6/8/9` 所属 baseline。

## Baseline B

- 分支：`update/senior-algorithm-baseline`
- 来源版本：上述 `cp0904` / 2026-09-04 包及其 SHA-256
- 导入策略：原样导入算法增量（NBV 节点、离线比较脚本、FAR 参数、NBV console entry point）和包内 RViz 显示配置；保留 Baseline A 中较新的安全门、DDS sensor-data QoS、可移植启动/停止/预检脚本。
- 状态：`Imported / NOT YET EXPERIMENTALLY VERIFIED`。
- Git commit：本轮 `chore: import senior algorithm baseline` 提交；精确 hash 以该分支提交历史为准。

这不是对学长整包的无条件覆盖。未导入的旧集成文件列于 Compatibility Risks，目的是避免恢复默认真实运动、删除 Sport lease 或重新引入 `/home/lch/dog` 硬编码。算法文件本身未做自主优化。

## Package Changes

- package 数量仍为 24；没有新增或删除 ROS 2 package。
- `go2_keyboard_teleop` 新增 `nbv_explore_node` console script。
- Point-LIO、terrain、FAR C++、local planner、path follower、WP5、safety gate 的 package 名称均不变。

## Node Changes

- 新增可选节点 `nbv_explore_node`，用于替代 `wp5_explore_node` 的探索目标选择逻辑。
- 当前 `code/scripts/nav_launch.sh` 仍启动 `wp5_explore_node`；NBV 没有被切换为默认主链节点，也没有新增 launch/remap。
- 其余主链 executable/node 的实现未发生算法源码变化。

## Topic / TF Changes

NBV 沿用 WP5 的接口：

| Direction | Topic | Type | Source QoS |
|---|---|---|---|
| Subscribe | `/terrain_map` | `sensor_msgs/msg/PointCloud2` | depth 10 default |
| Subscribe | `/state_estimation` | `nav_msgs/msg/Odometry` | depth 10 default |
| Subscribe | `/far_reach_goal_status` | `std_msgs/msg/Bool` | depth 10 default |
| Publish | `/goal_point` | `geometry_msgs/msg/PointStamped` | depth 10 default |

- 没有新增 service 或 action。
- 没有修改 topic 名、消息类型、launch remap 或 TF edge。
- `/utlidar/cloud -> Point-LIO -> terrain -> exploration -> FAR -> local planner -> pathFollower -> safety gate -> GO2` 主接口链保持不变；仅当操作者显式以 NBV 替换 WP5 时，exploration 内部选点逻辑改变。
- 包内旧版 `transform_everything.py` 会把原生 UTLiDAR 订阅从 `qos_profile_sensor_data` 改回 depth 50 默认 QoS；Baseline B 未导入该倒退，当前 DDS 接口行为保持 Baseline A。

## Algorithm Changes

### Exploration

- WP5 文件逐字节相同：前沿提取、8 邻域聚类、簇质心观察点、BFS 可达性、冷却/降权、到达、超时、历史状态和转向进展判断均未修改。
- 新增 NBV 作为可选替代：从已知自由栅格随机采样至多 150 个候选；对每个候选执行 36 方向、5 m 量程 ray-cast，遇 occupied 截断并累计可见 unknown 栅格；再按 BFS 路径、返航距离和目标转角计算效用。
- NBV 复用 WP5 的网格累积、BFS、最小目标距离、障碍硬门槛、到达投票、无进展/墙钟超时、冷却、降权及 home fallback。
- NBV 使用 `numpy.random.choice`，没有固定随机种子；相同地图上的候选集合和目标可能不完全可复现。
- NBV 声明并读取 `info_radius`、`risk_radius`、`risk_weight`，但当前 `_select_goal()` 的实际评分没有使用它们；`risk` 固定为 0。这是源码事实，不作优劣判断。

### FAR Planner

- 唯一算法配置变化是 `robot_dim: 0.7 -> 0.8`。
- FAR C++ 图构建、图去重、waypoint 选择、convergence 和障碍处理源码逐字节相同；`converge=0.30` 不变。
- 包内说明推导 `kNavClearDist = robot_dim/2 + voxel_dim = 0.5 m`；实际行为仍需构建/运行验证。

### Local Planner

- `localPlanner.cpp`、`pathFollower.cpp`、launch 和路径资源均相同。
- obstacle inflation、collision check、path scoring、速度选择和 `stopDisThre` 没有源码或配置差异。

### Path Follower

- 角速度控制、heading alignment、速度限制和转弯行为源码逐字节相同。
- 因此不能把既有“机器人长时间原地转向”问题的变化直接归因于 path follower 修改。

### Point-LIO / Terrain / Localization / Sensor Transform / GO2 Interface

- Point-LIO 和 terrain analysis / ext 源码、launch、YAML 均相同。
- 学长包的 sensor transform 仅缺少 Baseline A 后加的 sensor-data QoS 兼容改动；Baseline B 保留现有版本。
- 学长包的 safety gate 缺少 Sport lease apply/renew/loss interlock；Baseline B 保留现有版本。
- 学长包的主启动脚本默认 `dry_run=false`、自动武装、速度上限 0.6 m/s、yaw 上限 0.5 rad/s，并硬编码学长环境；Baseline B 保留当前默认 dry-run 和显式 `--real` 的版本。

## Parameter Changes

### Existing parameter changed

| Module | Parameter | Baseline A | Senior package / Baseline B | Verification |
|---|---|---:|---:|---|
| FAR | `robot_dim` | 0.7 | 0.8 | Static config only |

### Existing parameters unchanged

| Module | Parameter | Baseline A | Baseline B |
|---|---|---:|---:|
| FAR | `converge` | 0.30 | 0.30 |
| Local planner | `obstacleInflate` | 0.2 | 0.2 |
| Local planner | `stopDisThre` | 0.2 | 0.2 |
| WP5 | `min_goal_dis` | 0.8 | 0.8 |
| WP5 | `arrive_dis` | 0.35 | 0.35 |
| WP5 | `risk_weight` | 0.3 | 0.3 |
| WP5 | `min_obs_dis` | 0.2 | 0.2 |

### New NBV parameters

| Parameter | Old | New default |
|---|---|---:|
| `nbv_sample_num` | absent | 150 |
| `nbv_ray_num` | absent | 36 |
| `nbv_ray_range` | absent | 5.0 m |

NBV 同时复制 WP5 的通用参数默认值，包括 `arrive_dis=0.35`、`goal_timeout=60.0`、`no_progress_timeout=8.0`、`progress_thre=0.1`、`min_obs_dis=0.2`、`min_goal_dis=0.8`、`path_weight=0.3`、`return_weight=0.3`、`risk_weight=0.3`、`turn_weight=0.5` 等；这些是新增节点参数，不是对 WP5 参数的修改。

## Files Added

- `code/go2_keyboard_teleop/go2_keyboard_teleop/nbv_explore_node.py`
- `code/scripts/nbv_offline_compare.py`
- 包内另有顶层 `README.md`，其来源信息已转录到本文档，未覆盖仓库根 README。

## Files Removed

Baseline B 未删除任何文件。学长包本身缺少下列 Baseline A 的仓库适配/验证脚本，因此它不是当前仓库的完整超集：

- `code/scripts/deploy_local.sh`
- `code/scripts/env_go2.sh`
- `code/scripts/go2_lidar_preflight.sh`
- `code/scripts/go2_preflight.sh`
- `code/scripts/offline_full_chain_audit.sh`
- `code/scripts/start_multi_terminal.sh`

## Files Modified

实际导入：

- `far_planner/config/default.yaml`：`robot_dim` 0.7 -> 0.8。
- `far_planner/rviz/default.rviz`：registered scan alpha 1 -> 0.1，point size 3 -> 2（仅显示配置）。
- `go2_keyboard_teleop/setup.py`：注册 `nbv_explore_node`。

学长包中不同、但因安全/可移植性原因未覆盖：

- `transform_everything.py`
- `safety_gate.py`
- `check_chain.sh`
- `nav_launch.sh`
- `nav_stop.sh`
- `test_safety_gate.py`

## Potential Behavioral Impact

- `robot_dim=0.8` 可能使 FAR 使用更保守的机器人包络/净空判断，从而改变图连接、可通行判断或 waypoint；没有实验能证明它更好或解决转向问题。
- NBV 可能产生与前沿簇质心不同的 `/goal_point`，因此可能间接改变 FAR waypoint、局部路径和转向占比；当前没有离线或真机证据。
- NBV 的转角惩罚可能影响目标方向偏好，但 path follower 本身未变，不能宣称已解决长时间原地转向。
- RViz alpha/point size 变化只影响可视化，不是传感器数据或算法变化。
- 当前主启动仍使用 WP5，所以仅构建 Baseline B 并按原命令启动，不会自动启用 NBV。

## Compatibility Risks

- 若同时运行 WP5 和 NBV，两者都会发布 `/goal_point`，会形成双目标源；必须二选一。
- NBV 随机采样无固定 seed，重复对比需记录目标和随机状态，或在后续经 ChatGPT 决策后设计可复现实验；本轮不改算法。
- `robot_dim` 增大可能降低窄通道可通行性；需要地图/rosbag A/B 验证。
- `nbv_offline_compare.py` 依赖 `rosbag2_py`、ROS 消息包、NumPy 和 Matplotlib，并要求包含 `/terrain_map` 与 `/state_estimation` 的 bag；未传第二个参数时仍默认写入学长路径 `/home/lch/dog/tmp/nbv_cmp`。
- 学长包旧启动/安全文件与当前仓库不兼容且安全边界更弱；它们已被明确排除，不能用包内 README 的旧命令替代当前安全启动流程。
- 学长包未提供 Git commit、依赖锁定或原始 NBV 实验记录；SHA-256 只能标识收到的归档内容。

## Runtime Verification Needed

- 当前主机 ROS 2 Humble 全量 build 已完成：24 packages succeeded；存在既有 deprecated/unused 编译警告，没有 build error。该结果不是干净环境复现。
- `nbv_explore_node.py`、`nbv_offline_compare.py` 和 `setup.py` 已通过 `/usr/bin/python3 -m py_compile`；构建后的 overlay 已由 `ros2 pkg executables go2_keyboard_teleop` 确认包含 `nbv_explore_node`。
- `colcon test --packages-select go2_keyboard_teleop` 完成，但该 package 实际发现 `0 tests`；因此不能作为算法行为验证。
- 用同一 Baseline A rosbag 分别运行 WP5/NBV 离线比较，记录随机性、候选/目标和失败原因。
- 验证 `robot_dim=0.8` 对 FAR 图、waypoint、窄通道和 convergence 的影响。
- 若未来做真机测试，先 dry-run/no-arm，再经明确授权执行低速测试；本轮真机状态为 `NOT VERIFIED`。
- 验证完整 TF、QoS 协商、连续 topic 数据和 `/goal_point -> /api/sport/request` 链；源码存在或 build 成功均不等于真机有效。
