# 一键FULL自动左右抓取透传修复（2026-11-16）

来源：主入口go2_test.sh/pc_time_check.py、45e6b2f集成源码、最近full运行记录和离线测试。

## 为什么之前全部右侧

左侧和auto侧别选择已在集成分支5e818cf加入：使用同一S1服务、原左右轨迹，
按当前朝向下左右停车位的平面距离选择更近目标/侧别，同距选右侧。
但规范PC入口先进入主仓库camera_ws/scripts/pc_time_check.py；它只接受right/left，
默认right且总传--grasp-side right，覆盖集成go2_full_test.py原有full默认auto。
因此最近full日志grasp_side=right且side_candidates为空，表示未启用左右候选比较，
不能解读为已经比较后右侧更优。此为外层透传缺口，不是左侧轨迹未加入。

## 本轮修改及范围

仅主入口包装器接受right/left/auto，未指定时full默认auto，dock保持right；显式left/right保持。
主包装器继续设置GO2_PC_TIME_HELD=1，转交集成目录后不再进入其旧PC时间包装器，
新auto参数会直接进入原集成FULL→自动侧别停靠→原S1请求侧别链。
直接使用/tmp/go2_onekey_s1内旧时间包装器不是本项目规范入口；本轮不改该独立分支的旧包装器。

不修改左右停车位/轨迹、目标选择算法、ROS接口、S1服务、相机交接或三轮计数。
auto选择更近的停车位，不保证左右轮换；同距按原实现选右侧。

原命令继续使用：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute --mode full --detector hsv --view
```

本轮未执行该真实动作命令。主入口也可显式传--grasp-side auto/left/right；本轮仅离线验证。

## 验证

23项tests/test_pc_time_check.py通过；8个新增参数组合覆盖full默认/auto/left/right及dock默认/auto/left/right，
确认启动命令侧别、HSV/view和GO2_PC_TIME_HELD环境正确透传；其余保留校时故障阻断及恢复回归。
全部外部服务和硬件调用均模拟，无systemctl/sudo/SSH/ROS节点/机器人动作。
静态编译与git diff --check通过。真实左右自动选侧/左侧可达性仍NOT VERIFIED。
本轮不处理已记录的起立后定位数据中断/轮间就绪检查问题。
