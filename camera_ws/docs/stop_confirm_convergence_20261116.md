# STOP_CONFIRM目标位置收敛（2026-11-16）

来源：用户本轮明确要求位置收敛优先，允许机身不稳定；本地源码、历史run JSON和离线测试输出。

## 本次行为

现有SEARCH配置设置`stop_confirm_require_stationary=false`，覆盖dock/full的搜索确认阶段。
STOP_CONFIRM及确认切换周期仍输出零速，但目标样本从本次请求零速之后立即采集：
不再等待机身/相机定位速度、Sport速度和偏航速度满足旧停稳门槛；这些速度波动也不清空样本。
保留源时刻晚于本次确认开始、视觉新鲜度、不同时间戳、唯一空间匹配、manager CONFIRMED，
以及1秒窗口至少5份、跨度至少0.60秒、散布严格小于8cm、源帧最大间隔0.30秒。
6秒总预算/超时冷却保持。原安全门、定位/TF/Sport新鲜度、定位跳变和最终停靠/趴下/抓取/起立保护未改。

Settings默认仍为true，旧入口及直接构造保留原行为。恢复当前SEARCH旧模式可将此参数改回true。
日志新增`confirmation_mode=position_convergence`及`confirmation_sample_count/span`、`confirmation_spread`；
旧`stationary_sample_*`字段为兼容现有预览仍提供采样数据，不应解读为机身已停稳。
位置模式的`settled_at`、`stopped_after_sec`、`stop_stable_range_m`、`stationary_confirmation_sec`为null。

## 历史依据与边界

- full_1794819797246482768/run_1794819818752971803.json：前8次确认超时，第9次4.400秒确认。
  超时阶段反复等待停稳/候选缺失或歧义；前8次有5次采样峰值0，第7次峰值4份/0.403秒。
- full_1794820061800141575：第一轮3.400秒确认；第二轮连续3次超时，随后定位跳变FAULT。
- 这些历史日志说明旧判据阻断，不能通过修改后重解释为新策略当时已成功。
- 收敛仅指同一空间候选的MAP观测一致，不证明HSV识别身份正确、绝对定位准确或机身实际静止。

## 验证

- 157项定向离线测试通过：SEARCH/普通接近/MAP/抓取停靠/运动窗口/S1序列/连续三目标。
  新测试覆盖机身平移+偏航且目标收敛可通过；不收敛、重复/旧帧、稀疏/歧义/错误位置与过期定位仍拒绝。
- 测试需source /opt/ros/humble/setup.bash和deploy_ws/install/setup.bash，并将集成源码绝对路径置于PYTHONPATH首位。
  最初缺失ROS消息导致导入失败；一次测试收集出现Python段错误，环境补齐后同一目标集合157项通过。
- 单包colcon构建成功（约90秒）；安装模块路径与SEARCH配置false读回通过。
- 修改测试文件flake8通过；控制模块仍有基线已有的E501/E128各一处，未新增此类问题。
- git diff --check通过；full HSV离线计划通过（无--execute）。
- 无ROS在线/真实目标收敛验收，无运动、远端部署或服务重启；实机NOT VERIFIED。
