# S1 v2 自动回零与左右抓取集成（2026-11-14）

## 来源与状态

源包：用户交付的 `S1_arm_integration_20261114_auto_v2.zip`，
`BUILD_INFO.json.version=v2_auto_integration`。按包内 `docs/TEAM_INTEGRATION.md`
接入。GitHub 此分支只提交启动器/导航适配；**14 MB 原包没有写入 GitHub，也没有在
GO2 .18 上实际安装/执行。** 宿主安装与实机回零、左右抓取均待现场验证。

该包 `run_arm_integration.py` 启动时通过 `wait_for_valid_feedback()`
读取六关节实际角度，然后调用已有 `arm.move(q[:6], ZERO_Q, 'startup_zero')`，
完成后 `session.ready()` 发布 `ZERO_HOLD`。不会修改编码器零点；
不需要键盘 e/d/s 确认。仅当服务启动并且串口反馈有效时能够执行自动回零。

**接口边界：** v2 socket `/tmp/s1-arm-integration.sock` 只接受 `status` 和
`grasp`。不存在旧版的 `zero` RPC，也不存在运行时切换左右侧 RPC。
右入口 `scripts/run_arm_integration.py`，左入口
`scripts/run_go2_left_grasp.py`；它们不能同时占用串口。

## 一次性部署步骤

从对话保存完整 ZIP 到**PC 当前集成仓库根目录**，文件名保持
`S1_arm_integration_20261114_auto_v2.zip`。或者设置绝对路径环境变量：

```bash
export GO2_S1_V2_ARCHIVE=/absolute/path/S1_arm_integration_20261114_auto_v2.zip
```

下一次从PC启动时，脚本在**不存在S1服务**的前提下校验 ZIP、
通过 SCP 一次性上传并解压为 `/home/unitree/s1-arm-control-v2`。
它**不会修改或覆盖** `/home/unitree/s1-arm-control` 历史版本。

如果已有旧版服务（或任何其他所有者）占用串口，入口会明确阻断；
**不会热替换、不会杀死旧进程，也不会抢占串口**。
用户应先确认机械臂不在抓取、位置保持正常，再按原服务停机流程
退出旧版本，之后重新启动v2。

部署后使用GO2现有Python3.10环境 `/home/unitree/s1-arm-py310/bin/python`
启动S1。宿主SDK、RealSense运行环境与归档所需依赖的兼容性尚未经此版本
完整实机验收。运行期间复用健康的同侧 `ZERO_HOLD` 服务；异常时保留
具体状态，不把串口通信故障当作零位偏差，也不自动重启占用中的服务。

## 一键模式

默认仍然为右侧：

```bash
bash go2_test.sh --execute --mode dock
bash go2_test.sh --execute --mode full
```

显式选择左侧**停靠评估**（不进入趴下/抓取）：

```bash
bash go2_test.sh --execute --mode dock --grasp-side left
```

左侧完整抓取必须先由操作者验证左侧停靠、支撑和实际外参，
再显式授权 `--left-geometry-verified`：

```bash
bash go2_test.sh --execute --mode full --grasp-side left --left-geometry-verified
```

注意：左右侧服务切换**不是**行进中自动切换。若当前常驻服务
是右侧，要求左侧时会拒绝并提示先停止旧服务；自动选择侧别是
未来导航规划功能，目前未实现。

## MAP 停靠几何

- 右：当前停靠目标 `S1 Y=+0.41m`，使用现有右侧配置。
- 左：**仅为镜像初始候选** `S1 Y=-0.41m`，左侧抓取参考范围
  `[-0.33,-0.29]m`，仅适于早期停靠检查；实际安装/支撑/抓取可达性待验证。
- v2原包推荐抓取中心左右约 `±0.31m`，远至 `±0.40m` 存在侧翻/需手扶风险；
  不能把 `±0.41m` 停靠候选当作实际稳定抓取位置。
- 左侧配置使用 `science_grasp_docking_left.yaml`，右侧保持原配置；
  不改 Point-LIO、SEARCH、STOP_CONFIRM、vx/vy/wz控制和S1抓取轨迹。

**本轮没有实机运动或硬件回零。** 使用真实硬件前先检查
当前串口所有者、已装SDK、六关节反馈、零位回程路径以及侧别对应的
GO2物理支撑状态。先单独验右侧v2启动/抓取，再验左侧停靠和完整抓取。
