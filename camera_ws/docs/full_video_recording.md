# Full 前侧自动录像

2026-11-15用户要求移除新增D455录像，当前版本已删除腕部录像及其首帧检查、
共享pipeline借用和PC取回逻辑。原抓取回调直接执行，D455按原始抓取程序采集RGB-D。
这不代表D455底层断连已经修复，原抓取程序仍有自身的图像/深度质量检查。

入口不变，view仅控制预览，不控制前侧保存：

```bash
bash go2_test.sh --execute --mode full --view
```

PC主仓库 `camera_ws/records/full_<时间戳>/` 下保留：

- `approach_*.mp4`：前侧D435搜索至停靠结束的识别/控制叠加画面。
- `approach_*_raw.mp4`：对应原始RGB；JSONL保存图像时间和检测/控制记录。
- `recordings.json`：前侧录像索引，wrist.status=DISABLED。

既有录像、故障证据及远端data/grasp_videos原件不删除。

兼容层v2_usb01_feedback_no_video_4支持从video_3升级：仅在没有S1串口所有者时，
核验原文件/已知兼容hash，恢复原capture_rgbd.py和直接抓取回调，备份并移除已知录像模块。
运行中的旧服务不停止、不热替换；需在机器人停稳、机械臂有支撑后正常停服，
下次原入口才应用新版。不会自动清除FAULT_HOLD、启动抓取或起立。

72项离线检查及静态编译/lint/离线计划通过；含实际v2原ZIP对照和本机模拟video_3升级。
本轮未部署、未运行真实相机/机械臂或运动，真机效果NOT VERIFIED。
