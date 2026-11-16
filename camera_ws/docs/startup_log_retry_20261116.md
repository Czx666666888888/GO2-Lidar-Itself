# 定位重建日志冲突修复（2026-11-16）

基线：远端最新00983d0165b89fda97cadd552d2ca9bbd5ec1cca。保留主视觉分支的全部既有脏改动。

## 实际触发

full_1794822629924990770在原有final_live_readiness检查失败后进入一次定位重建分支。
pre_motion.json中ready/slam_ok=false，30秒内收到RGB111、Depth1、detector0、base407/cloud411/odom407，
body/camera TF false。final_live_readiness总耗时32.930秒FAILED。
旧定位正常退出后日志slam.log保留，而Workflow.spawn仍用open('x')创建同一路径，故Errno17。
第二次定位进程尚未启动，也未生成pre_motion_reinitialized.json，不能说定位重建已失败或通过。

## 修改

仅调整scripts/go2_full_test.py的spawn日志分配：

- 首次仍用slam.log；后续按slam_2.log、slam_3.log等寻找下一个可排他创建的文件。
- 使用open('x')处理实际占用，不先exists再覆盖，不删除/截断/追加原始日志。
- 子进程逻辑标签仍为slam，原健康检查、停止匹配、进程组及清理策略保持。
- 同标签子进程仍在运行则显式拒绝新启动，避免日志改名意外容许重复定位。
- Popen启动失败仍关闭句柄，失败文件保留，下次使用新编号。
- 相机/viewer重复启动同样适用独立日志；不会因此主动触发新启动。

原预检阈值、一次重建次数、相机/Point-LIO启动命令、MAP/SEARCH确认和完整抓取链不改。
该修复只是让原有重建路径能够执行，不解决深度/TF/硬件数据根因。

## 验证

`python3 -m pytest -q camera_ws/test/test_full_spawn_logs.py camera_ws/test/test_camera_handoff.py camera_ws/test/test_startup_interfaces.py`

57 passed；其中新测试使用实际短Python打印子进程复现退出→停止清理→重复标签启动，
确认首份日志字节不变、新日志完整、逻辑标签不变；还覆盖已存在多个编号、Popen异常和活进程阻断。
静态编译、测试文件flake8和git diff --check通过。full HSV离线计划通过（不带--execute）。
仅入口脚本，无需ROS包重建。本轮未运行full、真实运动/抓取、相机采流或远端服务重启。
完整定位重建和真机流程：NOT VERIFIED。
