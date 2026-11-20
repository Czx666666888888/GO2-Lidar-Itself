# NX 最近五轮未抓取原因（只读诊断）

本轮依据 NX 原始运行记录、录像检测元数据、控制状态、协调器事件和 S1 日志；未启动运动、未修改控制算法。以下五轮均为 `latest_received` 输入模式。NX 时钟存在回退，不能仅按目录时间排序；持久终端最后关联运行是 `full_1783582592115311672`。

日志根目录：NX `/home/unitree/go2_full_nx/releases/599c86c/source/camera_ws/records/`。

| FULL 运行 ID | 识别证据 | 未完成抓取的直接原因 |
| --- | --- | --- |
| `full_1783581493286153880` | 329 份检测中 16 份有方块；存在 CONFIRMED 目标 | EXPLORE 约 30.2 秒后 `localization jump or time regression`，没有进入停车确认或接近。日志无直接抓取准入/拒绝事件，不能断定为何此前没有切换目标。 |
| `full_1783581726395596632` | 439 份检测中 94 份有方块 | 目标 3、左侧、墙距 0.288 m 已准入；STOP_CONFIRM 在 8 秒后以 `parking_unstable` 失败，采样数 0。尚未进入接近。 |
| `full_1783582012103494744` | 268 份检测中 124 份有方块 | 第一目标已完成停靠交接并进入 S1；腕部观测端点误差超过有界修正范围，原抓取序列返回 MISSED。随后第二目标停车确认阶段丢失通道或目标。 |
| `full_1783582292668606840` | 899 份检测中 870 份有方块 | 前期多次墙距超过 0.35 m 或无匹配 RGB-D；约 111.8 秒后准入。进入 DOCK 约 3.21 秒后交接一致性检查失败，未到抓取位。 |
| `full_1783582592115311672` | 94 份检测全部有方块 | 目标 1、左侧、墙距 0.192 m 已准入；停车确认通过。进入 DOCK 约 3.05 秒后定位连续性检查失败，未到抓取位。 |

## 定位连续性失败

第一轮与最后一轮均报 `localization jump or time regression`。最新输入模式的控制器仍检查相邻位姿的处理时间差、位置增量、推算速度和航向变化率；这是多个分支共用的故障文本，不能仅凭名称判定为源时间戳回退。

最新轮首次故障时，里程计接收年龄约 0.068 秒、Sport 反馈年龄约 0.037 秒；第一轮分别约 0.059 秒和 0.032 秒。没有证据指向此前已取消的 400 ms 源时间戳年龄门槛。失败位姿样本未记录，究竟哪项增量触发仍为 `Unknown / needs verification`。

最新轮到故障时两个轴到位记录均为 false，最后停靠误差约为前后 0.657 m、横向 -0.178 m，因此不是到位后漏调用手臂。

## 停车确认失败

`1726395596632` 在 6 秒执行原地复核、8 秒超时；阻塞项为 `parking_unstable` 与 `observations_insufficient`，未开始有效确认采样。故障时定位/相机估算速度约 0.053 m/s，而 Sport 平移速度接近零。说明停稳判断受到定位侧速度影响，不能据此断言机器人实际仍在移动。

## S1 实际尝试但未抓取

`2012103494744` 第一目标停车确认约 0.9 秒通过，历史两轴到位记录齐全并反馈停稳后进入 GRASP。结果文件记录 `task_motion_complete=true`、`handoff_ready=true`、`outcome=MISSED`；任务动作结束不代表物块被抓住。

S1 日志记录 `GRASP_FAILED_SAFE_RETURN`，原因：

`ABORTED: ObservationEndpointConvergenceError: observation endpoint error exceeds bounded correction range`

之后返回 `ZERO_HOLD`。对应腕部记录在 NX `/home/unitree/s1-arm-control-20261118/data/go2_visual_alignment_runs/alignment_20260709-152835/run.json`。端点误差的具体数值、图像及机械成因尚未确认。该轮统计 processed=1、completed=0、missed=1。第二目标则在停车确认时报 `UNREACHABLE: stop-confirm channel or target lost`，未再调用手臂。

## 交接一致性失败

`2292668606840` 最终错误为 `Original controller lost the selected handoff identity or side`。对应源码使用同一错误文本覆盖控制状态不属于近场状态、控制消息超过 0.55 秒未收到、目标 ID 不匹配或抓取侧不匹配。现有证据不足以区分这些分支，不能直接写成目标 ID 改变。

最后两个轴到位记录均为 false，最后停靠误差约为前后 0.455 m、横向 -0.101 m。后续 external stop 属于协调器失败后的收尾，不能倒置为首发原因。

只读复核后 NX 开始关闭 SSH 握手，未完成上述两个合并错误的进一步细分。修复效果和真机重试均 `NOT VERIFIED`。
