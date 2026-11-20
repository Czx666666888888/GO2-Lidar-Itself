# 第一轮起立后第二轮FAULT（2026-11-16 18:50运行）

来源full_1794826213729797107（45e6b2f/HSV/full）、s1_task_20261116T105051Z.jsonl、
post_arm_camera_1794826311976395355.json、第二轮run_1794826326351885363.json及control/gate日志，
当前go2_full_test.py/run_map_anchor_experiment.py/science_target_approach.py源码。仅只读，无动作或参数修改。

## 已确认时序

- 18:51:48.639：S1 GRASP_RETURNED_ZERO_HOLD，last_exit_code=0、at_zero=true、通信OK。
- 18:51:48.640：StandUp RPC返回0；18:51:51.323起立反馈confirmed=true，body_height约0.3299m，随后TASK_COMPLETE。
- 相机恢复后的12秒probe最终ready=false、camera_ok=true、detector_ready=true，但slam_ok/body_tf/camera_tf=false。
  RGB58/Depth57/检测59且fresh=true；cloud44、odom/base44、imu500，末次收取年龄分别约9.081/9.078/9.021秒，均fresh=false。
  这是转换后点云/IMU与里程计链停止更新证据，scope=chain没有重新观测原生雷达，不能直接断言雷达硬件停止。
- 18:52:04.973 CAMERA_HANDOFF_RESUME仍ack ok=true；三轮父任务计数1/3，随后启动第二轮。
- 18:52:07.192第二轮控制器IDLE，robot/camera=null、odom_age/camera_age=null；Sport反馈持续新鲜。
- 18:52:09.392 START_REJECTED: lock/pose/TF/feedback not ready。
- 18:52:11.492控制器FAULT/manual emergency stop，安全门记录独立停止。第二轮未进入SEARCH。

## 实现缺口与证据边界

resume_front_camera调用完整chain probe，但只检查report.camera_ok，忽略ready=false和slam_ok=false，
所以相机恢复被误当作下一轮全部输入已就绪。下一轮Experiment.run的SEARCH准备主要检查IDLE/录像帧数，
未确保当前控制器已取得有效robot/camera位姿，随后start被拒绝。
run_map_anchor_experiment.py的2秒start-rejected看门狗异常会进入finally close，close发送/stop。
该时序与本轮从START_REJECTED到FAULT约2.1秒吻合，自动收尾为有源码支持的推断；
/stop消息本身没有发布者ID记录，不能独立证明发送者，也不能仅凭manual字样认定人按急停。

第一轮软件动作完成和起立反馈成立；物块真实放置仍placement_confirmed=null。
本轮故障不是第一轮抓取/起立失败，而是下一轮缺定位数据仍尝试启动。

后续应补齐：相机恢复后整条chain ready检查、下一轮控制器自身pose/TF新鲜度准备检查；
同时分段记录原生LiDAR/IMU→转换→LIO/TF，定位起立恢复窗口的数据中断原因。
不放宽保护，不自动复位或重复运动。本轮未修改生产代码，具体数据中断根因Unknown / needs verification。
