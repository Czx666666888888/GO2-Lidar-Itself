# YOLO成功轮次之间解除目标锁（2026-11-16）

基线40afb95。用户要求修复首轮抓取起立后YOLO仍锁旧目标的问题；诊断原始证据为full_1794834675902500879：第二轮71帧深度有效，但MAP无新增、candidate/history全空。

## 修改及边界

run_three_grasps仍仅在task_motion_complete=true（原抓取回零+StandUp反馈通过）后计数；第二/三轮run_full_round在原S1 ZERO_HOLD检查后，把已有成功结果的累计MAP XY列表发给现有camera_target_to_map节点的SetParametersAtomically服务，参数completed_targets_json。服务3秒发现/5秒响应、子进程12秒上限；失败或拒绝则不启动下一轮SEARCH。不修改S1、姿态、相机交接和计数条件；第一轮、HSV、dock、独立approach不调用该接口。

adapter参数回调仅接受列表增加一项且旧前缀一致；非法/缩短/重排拒绝，重复提交幂等、不清当前锁。新成功列表生效后清initial/current与latest，将源时间水位提升到当前ROS时间，拒绝轮间排队旧RGB消息重新锁旧物块；不因漏帧/丢失自动解除锁定。

选择前排除距已完成MAP XY小于0.15m的候选，沿用现有任务链已完成目标半径。未完成候选仍执行原近邻歧义检查、最近目标选择及initial0.20m/current0.15m单轮关联；没有修改深度、模型、MAP转换或SEARCH停稳条件。已完成物块不参与剩余候选歧义检查，原Manager历史不清除。

该参数操作仅在父协调器成功轮次边界调用，不是故障后自动重试动作；外部运维不应手工在活动抓取中调用。新接口使用节点自带参数服务，无新增topic或自定义service消息。

## 验证

50项定向离线测试通过：成功边界计数、失败不推进、默认轮/HSV不重置、重置确认先于下轮任务、确认失败阻断、两轮解锁及已完成排除、剩余候选歧义、丢失不换锁、非法列表不变更、重复请求不再次解锁、旧时间戳消息拒绝，以及服务发现/响应超时/拒绝和原相机交接回归。首次ROS测试环境因覆盖PYTHONPATH缺geometry_msgs，修正继承ROS环境后通过；无真机测试。

单包colcon build成功（1分30秒），安装版selector/adapter与源码SHA256一致；py_compile及git diff --check通过。

未启动相机/ROS图或运动/抓取，未重启现有服务；在线轮间行为NOT VERIFIED。上轮定位跳变和S1跟踪等待/USB断连问题不在本轮修复范围。
