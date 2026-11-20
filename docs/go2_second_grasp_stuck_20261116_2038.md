# 2026-11-16 20:38 FULL第二次左侧抓取卡住（只读诊断）

证据：camera_ws/records/full_1794832708501958436；NX alignment_20261116-204046/run.json、position_command_trace.jsonl、s1_v2_auto_full_1794832601046642714.log及kernel journal。
NX快照保存在camera_ws/analysis/second_grasp_20261116_2038/。未执行动作、重启、回零或修改抓取算法。

## 时序与直接原因

- 第一轮也是left，S1约45.582秒返回ZERO_HOLD，20:40:03起立反馈通过，三轮完成计数1/3。
- 第二轮20:40:38停靠STOP，位置误差62.954mm；20:40:46.592趴下反馈通过并接受left抓取。
- D455最终采图质量通过，最终高位平面误差4.758mm，converged=true；已规划213.49mm预抓取下降，并开始执行。不是卡在SEARCH、STOP_CONFIRM或首次取图。
- 约20:41:49.634（NX monotonic373.678）起同一关节目标不再推进，service最后一条跟踪信息为support_normal_adaptive_pregrasp WAIT，最大误差0.09105rad，阈值0.08rad。此时J6目标-0.878223rad、反馈-0.787175rad；主要阻塞是J6跟踪误差。
- 抓取源码s1_control/arm.py的_motion_feedback/move/move_waypoints在pause_on_tracking_lag开启时等待误差降到阈值以下才推进轨迹；没有该等待分支的独立时间上限。快照同一目标保持至少209.149秒。trace phase仍写adaptive_pregrasp_planning，但服务日志证明已进入support_normal_adaptive_pregrasp下降执行，不能仅凭phase名称判断是规划计算卡住。
- 20:42:31～32 PC任务日志终止，恢复相机被拒绝：S1仍GRASPING/busy=true/at_zero=false，未发起第二次StandUp。没有TASK_COMPLETE，不能算第二次完成。
- 20:42:44 kernel明确记录CH341 ttyUSB0 urb -71和USB disconnect；20:42:56重枚举为ttyUSB1。其发生在最初固定目标约54秒之后，不能作为最初等待的原因。之后旧进程仍指向ttyUSB0，出现大量Read error: Success，状态为COMMUNICATION_ERROR/device missing，继续恢复被阻断。

## 边界及后续

S1 status中的last_result/last_exit_code=0/last_log仍指第一轮203915，不能当作第二轮成功证据；busy和本轮204046原始记录优先。
两轮left证明自动左侧请求已进入S1；第二轮故障不证明左侧整体不可用。J6为何无法进一步收敛（负载、摩擦、力矩/控制或机械限制）Unknown / needs verification，USB断连具体物理原因也未确定。

后续应单独评审S1跟踪等待的有界故障退出和反馈新鲜度检测；不能通过直接放宽0.08rad或自动切换端口后续跑来掩盖问题。设备恢复/回零须另行授权和确认当前物理姿态。本轮无生产代码修改，恢复效果NOT VERIFIED。
