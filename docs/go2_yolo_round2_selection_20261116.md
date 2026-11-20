# 2026-11-16 YOLO第二轮有框但不进入STOP_CONFIRM

证据full_1794834675902500879（40afb95/V3），原始run/control/视频JSONL/slam/S1日志，以及集成源码与安装版color_block_selection.py一致的SHA256。

第一轮21:12:58.652起立反馈通过并计数1/3。轮间probe ready/camera_ok/slam_ok/body_tf/camera_tf均true，不能套用早前轮间定位未就绪的诊断。
第二轮21:13:13.377进入SEARCH，24.915秒后21:13:38.277因localization jump or time regression进入FAULT；未出现STOP_CONFIRM。
第二轮视频JSONL有298份检测，76份有框，71份深度accepted_central_roi，另3份RGB-depth时间不匹配、2份混合/噪声深度。290份控制状态均无candidate、无search_observations、first_seen=null。
slam日志MAP_TARGET最后一次在第一轮1794834706.510，adapter退出时input=100/transformed=100，第一轮后没有正式目标MAP更新。

源码确认：color_block_target_adapter启动时创建一次BlockSelection，首次commit保存initial/current；之后只允许距initial≤0.20m且距current≤0.15m的唯一目标更新。其生命周期跨三轮；pause/resume只停启NX前相机，保留PC adapter及Manager，没有轮间解除选择的接口/调用。新轮仅新建控制器并传completed_targets，无法解除上游旧物块锁。因此存在YOLO单目标适配与三目标持续链的衔接缺口；新物块即使2D和深度有效，也不能自动成为新轮正式候选。

日志未录制/science/color_block/selection逐帧拒绝reason，无法把全部71份逐一归类；不能排除部分帧另有TF等待/年龄等拒绝。但固定旧目标锁是源码和生命周期确认的缺陷，与第一轮后MAP完全无更新、候选全空吻合，为本轮主要诊断。
后续应在一轮真正完成后显式解除本轮YOLO选择锁，结合已完成目标排除后重新选择，单轮内仍保留唯一关联；不得在每帧丢失时自动解除。另查最后定位跳变，不能称为STOP_CONFIRM停稳超时。
本轮只读分析，无生产代码修改、重启或动作；修复及实机效果NOT VERIFIED。
