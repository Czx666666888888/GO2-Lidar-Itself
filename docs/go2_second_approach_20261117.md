# 2026-11-17 第二轮远处停车趴下诊断

仅离线只读复核，未运行机器人、重启服务或修改控制参数。用户报告第二轮距离仍远就停车趴下，物理失败反馈保留；日志估计值不作为物理到位证明。

## 固定运行与证据

- 最新运行：camera_ws/records/full_1794902087861996506，manifest为8c35804f2007c3740f8e95887a4c513e29939c60、YOLO V3、S1 20261117_final_fusion_v4。
- 第二轮：science_approach_1794902214485695764.jsonl、run_1794902213672865075.json、1794902213672865075_control.log、handoff_1794902213672865075.jsonl、round_02_coordinator.log。
- 视觉：approach_20261117_155654_565844.jsonl及其raw.mp4。
- 姿态/机械臂：s1_task_20261117T075706Z.jsonl、round_02_result.json。
- 对照实际运行源码：/tmp/go2_onekey_s1/camera_ws/src/go2_science_perception/go2_science_perception/{approach_control,grasp_docking}.py；该工作目录HEAD与manifest相同，tracked文件无修改。

## 已确认的因果链

1. 第二轮直接选择记忆target_id=1，左侧目标，MAP锚点(2.707194907,0.840627142)。最后观测1794902127212921875；开始接近时visual_age=89.279s，停止时94.979s。memory_reconfirmed=0、anchor_updates=0，目标坐标全程未更新。第二轮记录82份detection全部targets=[]，410份selection全部no_color_block；抽查raw视频0/55/110帧均只见地面，不能据此量出真实物块距离。
2. 15:56:56.492开始APPROACH_TO_GRASP_POSE，至记忆停车位的估计误差29.07cm。当前实际左右横向目标为±0.32m（非历史±0.36m），位置容差8cm、arrival_latch_enabled=true。
3. 15:57:00.192一次估计误差7.937cm，触发docking_arrival_latched，停止平移进入STOP_WAIT。源码将arrival_ok设为position_ok或历史到位锁存；锁存后不会因当前误差退出范围而恢复平移。
4. 15:57:02.192估计误差已回到11.513cm，docking_pose_in_tolerance=false、predicted_in_grasp_region=false，但历史到位仍有效。航向合格且有效停稳累计0.500367s，状态STOP、grasp_docking_complete=true。此为相对旧MAP停车位的估计，不代表真实物块到位。camera_distance_xy=0.538315m是前侧相机至旧锚点的平面距离，也不能视作还应前进53.8cm。
5. 15:57:00.592起localization_untrusted=true；最终窗口化LIO路径0.470088m、Sport速度积分0.161559m，差0.308529m；motion_disagreement_stop_enabled=false，未阻断STOP/交接。两种运动估计明显分歧已确认，但无法据此断言LIO单独漂移30.9cm或把路径差直接当终点误差。
6. 15:57:08.178开始StandDown RPC，返回0；15:57:11.224高度/模式/静稳反馈确认。说明是导航宣告到位后正常协调器交接趴下，非因超时/故障触发的趴下。
7. S1随后返回VISUAL_NO_TARGET_SAFE_RETURN，ZERO_HOLD后起立；第二轮outcome=MISSED。three_grasps.json实际processed=3、completed=2、missed=1，placement_confirmed=null；不能称三次均抓取成功。

## 结论及未知

直接停止/趴下原因是：未重新观测的历史MAP目标 + 一次进入8cm范围即永久锁存到位 + 停稳后允许交接；运动估计差异保护当前关闭，因此已标记不可信的定位仍用于完成判定。该组合允许真实目标仍远时报告到位。
真实物块与停车点的精确偏差、第一轮趴下/起立前后MAP一致性、旧目标是否准确及运动估计分歧的底层原因：Unknown / needs verification。前侧第二轮无目标，无法用其录像恢复真实物块坐标。修复方案和修复后实机效果NOT VERIFIED，本轮未更改算法或保护配置。

## 用户指出横向问题后的分轴复核

用户明确指出主要为横向距离。对第二轮58个停靠控制样本分轴：

| 时刻 | 至停车位车体前后误差 | 至停车位车体横向误差 | 预测目标S1横向坐标 |
| --- | --- | --- | --- |
| 开始接近 | +24.30cm | +15.95cm | -48.02cm |
| 首次锁存到位 | -0.29cm | +7.93cm | -40.08cm |
| 最终STOP | +6.68cm | +9.38cm | -41.49cm |

本轮选择left，S1横向目标-32cm；负号代表S1左侧，对应车辆左侧物块。首次锁存时前后已基本对齐，合成误差7.94cm几乎全部为横向误差，仍被8cm合成位置容差接受。最终预测目标横向仍距S1底座41.49cm，比目标32cm多9.49cm。车体至停车位的横向误差9.38cm与S1横向差9.49cm因当前航向与停车目标航向略不同而不完全相同。

因此直接问题具体为：横向还剩约8cm时就因合成8cm阈值锁存停止，随后横向残差回到约9.4cm，仍不恢复横移便趴下。并非日志中没有横移命令：gate记录Move vy=+0.034、+0.089、+0.128、+0.134m/s等，方向为车体左侧；这些输出证明软件发出了横移请求，不证明真实横移量达标。按控制样本梯形积分，Sport有符号车体横向速度积分约+5.06cm，仅为机身反馈估计，不能当作真实位移或据此断言横移死区。

上述均对旧MAP锚点而言。真实横向偏差可能更大；旧锚点误差与定位偏差仍缺独立真值，不能将9.49cm写成实测距离。后续若修复，应单独评审横向到位条件以及锁存后横向偏差超限的处理；本轮未改参数或算法。
