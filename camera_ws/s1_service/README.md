# 原S1唯一服务启动接口补丁

来源：`S1_arm_integration_20261111_auto_zero_hold_v1.zip`，SHA256
`a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e`。
本目录保留原服务与客户端结构，只扩展启动、状态及回零命令；不含/不替换D455抓取算法、
SDK、Arm运动实现、绑定配置或路径。`source_manifest.json`记录原文件hash。

一键入口仅在原服务未运行时安装，逐文件校验原件或本补丁hash并备份原件；发现其他版本保留并阻断。
已有新版ZERO_HOLD直接读取反馈并复用；旧服务没有关节反馈接口时保持现场并报告升级需要，
不为查询状态打开第二个串口控制进程，也不热替换运行中的服务。

状态保留原`state`，增加`readiness_state`/`position_state`、实测六关节、速度、零位容差和具体error。
`COMMUNICATION_ERROR`指设备缺失/SDK电机离线/总线或I/O故障；非零位单独为`NOT_AT_ZERO`。
`AT_ZERO`的零目标来自原`ZERO_Q`，容差沿用原`SafetyLimits.max_track_err_rad`；
SDK仅提供读取反馈，没有独立电机包时间戳，不把读取时间称为串口数据时间。

`zero`只排队到原服务的单工作循环；GRASPING/RETURNING_ZERO/QUEUED拒绝重复触发。
只有原plan的`start_safe_windows_rad`内，才使用原`arm.move(q, ZERO_Q, "startup_zero")`，
沿用S曲线/原速度与保护，返回后用原连续有效反馈检查并确认六关节零位，再进入保持。
超出该已验证窗口：保持`NOT_AT_ZERO`、`return_zero_allowed=false`，不启用、不猜测回零路径。
ZERO_HOLD的实时反馈不符或通信失败时禁止导航/新抓取。

本轮GO2没电：未部署、未读取实机六关节、未实际回零。必须在重新上电后以同一个入口验证；
`--startup-check`只读取已有服务，不创建/使能机械臂。`--execute`才允许原服务受控回零和GO2运动。


### 新任务的错误和结果字段

v4 适配版本 `v4_multitarget_dump_3` 在抓取或倒料请求被接受并进入 `QUEUED` 时，将 `error`、`last_result`、`last_exit_code` 和 `last_log` 一起设为 `null`。本轮完成时由服务写入本轮结果。拒绝的请求保留现有状态和结果。历史日志文件及硬件故障锁存继续保留。

部署沿用原有 v4 包校验与升级流程。检测到既有 S1 服务持有串口时，升级会等待后续安全切换。兼容旧版 `v4_multitarget_dump_1` 和 `v4_multitarget_dump_2`，保留原始归档与升级备份。任务状态、交付清单和协调器的 25 项离线回归通过。新版本实机部署与连续抓取为 **NOT VERIFIED**。

### 左侧抓取的轨迹跟踪容差

适配版本 `v4_multitarget_dump_4` 将左侧抓取流程的 `maximum_track_error_rad` 从 0.08 调为 0.12 rad。该参数用于六个运动关节的轨迹跟踪等待，沿用控制器原有的收敛阶段倍率。回零验收、右侧抓取配置、TCP 到位和夹爪力度继续使用各自原有参数。升级只接受校验通过的旧版交付，实机效果 **NOT VERIFIED**。
