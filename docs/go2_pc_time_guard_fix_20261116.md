# PC校时包装器故障阻断修复（2026-11-16）

用户现场输出：systemd1激活等待25000ms超时后，入口仍打印“快速启动”和time_context 0.000s。旧包装器把任何非零退出码当作inactive，且计时在初始查询之后开始。

主仓库go2_test.sh现在在PC full/dock的execute转交前进入本仓库camera_ws/scripts/pc_time_check.py；继续通过GO2_PC_TIME_HELD=1调用原集成入口，不修改/tmp/go2_onekey_s1或其更新/构建策略。命令仍是bash go2_test.sh --execute --mode full --view。原approach和aarch64/local分支保持。

- systemctl is-active限3秒，只有退出0且active、或退出3且inactive并无stderr可接受；bus错误、failed、unknown、空响应和超时均阻断，不启动子任务。
- timedatectl详情查询限3秒且检查退出码；systemctl stop/start限10秒。sudo密码交互仍由操作者完成。
- 计时从初始查询前开始，查询失败记录time_context_failed_sec及error；report.json在失败和收尾失败时仍写入，不再伪报就绪0秒。
- 运行中监测失败走原停止顺序：向子任务请求SIGINT，等其退出，才恢复校时；不会为追求快速退出而杀死控制进程。
- 初始状态无法确认时不改服务、不二次查询；报告状态Unknown。恢复失败保留cleanup_error和非零退出，不写成恢复成功。

验证：python3 -m pytest -q tests/test_pc_time_check.py，15 passed。全部系统命令、子任务、sudo为模拟；shell转交测试用无动作参数记录器。另完成shell语法、Python编译与git diff --check。纯Python/脚本改动，不涉及ROS包构建。

未执行full、未重启/停止本机或NX服务、未部署机械臂、未发运动命令。修复只处理入口错误分类、等待边界和日志；本机systemd/D-Bus底层超时原因与真实运行效果NOT VERIFIED。雷达健康保护、检测器、抓取侧别和控制算法未改；先前发现的auto侧别透传问题另行处理。
