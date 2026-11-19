# NX 接近阶段退出导航，保留多目标与抓取侧一致性

用户确认参考入口为以前 PC 的 `bash go2_test.sh --execute --mode full`：识别、靠近、趴下、S1 抓取。用户同时明确保留多目标和抓取侧一致性。本轮没有发送运动命令。

## 定位连续性异常的含义

对照 Git `a8f7c66` 的原 PC `approach_control.py` 与本次 NX 源码，旧控制器已经包含相同的相邻位姿检查：处理时间差不为正、位置增量大于 0.30 m、位置增量超过 `0.8 * dt + 0.02` m，或航向增量超过 `1.0 * dt + 0.05` rad，任一项触发共用错误 `localization jump or time regression`。这里的速度是定位推算值，并非直接证明机器人实际超速。失败样本没有记录，最近两轮具体触发分支仍待核对。

这次恢复的是原 PC 控制链的接近控制权边界，没有将整个 NX 工作区覆盖成早期 Git 快照；多目标、最新输入、历史两轴到位和现有左右侧参数保持当前用户选定版本。原 PC 控制器的定位检查仍然存在，因此停止导航不能被写成定位异常已经修复。

## 对照与最终行为

| 环节 | 原 PC 独立 FULL | 修改前 NX 融合 FULL | 本轮 NX |
| --- | --- | --- | --- |
| 搜索 | 原 SEARCH | NBV/FAR/局部规划探索 | 保留当前自主探索 |
| 目标准入 | 原接近控制器 | 协调器墙距、目标选择、停车交接 | 进入接近前保留当前准入与停车交接 |
| 接近运动 | 原接近/停靠控制器 | 原控制器外还检查导航定位、地形和规划端点 | 停稳后先结束本轮导航 launch，确认规划发布者退出，再启动原 STOP_CONFIRM/接近 |
| 速度出口 | 原控制器经 Sport gate | MotionArbiter 选择控制源，经 Sport gate | 保留唯一出口，接近只转发原控制器速度 |
| 接近时导航约束 | 无融合协调器约束 | 仍检查地形新鲜度、共享定位、路径/规划端点等 | 不执行这些导航约束；定位、相机和原控制器仍运行 |
| 多目标 | 原 FULL/S1 循环 | 完成记录、已访问过滤与下一轮循环 | 保留；未修改循环程序或目标过滤实现 |
| 抓取侧 | 原控制器确定 | 与选定侧/锁定侧保持一致 | 保留，同时保留选定目标 ID 检查；不匹配时明确报错 |

`full_nav.launch.py` 实际包含 NBV、FAR、地形分析与局部规划，Point-LIO、相机、识别、Sport gate 和原接近控制器不在该进程组内。停止只针对本轮由 `start_new_session=True` 启动的导航进程组，不匹配杀死其他任务。

停止期间持续处理 ROS 回调与零速心跳，避免等待导航退出时让唯一运动出口停止刷新。确认 `/mission/nbv_goal`、`/mission/far_way_point`、`/nav/path`、`/nav/cmd_vel_raw` 发布者消失后，日志打印 `NEAR_NAVIGATION_STOPPED`。下一目标由原 FULL 循环创建新一轮探索，接近时再停止该轮导航。多目标现场循环效果仍待实际验证。

## 部署与验证

已更新 NX `/home/unitree/go2_full_nx/releases/599c86c/source` 的：

- `camera_ws/scripts/run_map_anchor_experiment.py`
- `camera_ws/src/go2_science_perception/go2_science_perception/mission_manager.py`
- `.nx/install/go2_science_perception/lib/python3.8/site-packages/go2_science_perception/mission_manager.py`

初始文件备份：`/home/unitree/go2_full_nx/releases/599c86c/original_near_backup.8mxvieb9`。最终非阻塞停止版本安装前备份：同目录 `original_near_backup.16a8oqyg`。本轮中间兼容修正备份 `original_near_backup.a9r54qyn`。完整最终差异和原始/最终哈希见 [补丁](go2_nx_original_near.patch)、[清单](go2_nx_original_near_manifest.json)。生成及受哈希保护的安装脚本见 `scripts/nx_original_near/`，不包含机器人启动动作。

本机 8 项隔离回归通过；NX 最终安装版本共 173 项离线回归通过，覆盖导航交接隔离、目标/抓取侧不一致拒绝、多目标与完成过滤、真实模拟子进程停止且其他进程保留、退出期间回调持续、原接近、停靠、历史两轴到位和 S1 时序。首次接口回归的 3 项失败源于测试构造的对象没有 run 属性，已兼容修复并全量重跑通过。本机测试辅助 AST 构造曾发生解释器段错误，改为直接编译原方法文本后通过，未涉及控制程序运行。

纯 Python 文件已解析并读回哈希，没有 C++ 或接口变更。未运行传感器连接 dry-run、真实导航退出交接、运动、趴下或抓取；这些均为 `NOT VERIFIED`。尚不能确认最近定位连续性异常是否仍会复现。
