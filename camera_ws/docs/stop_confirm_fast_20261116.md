# SEARCH快速确认与实际停稳（2026-11-16）

## 基线与范围

已fetch远端最新f9245bcd9bac49a7cb2efb8a12b64617c5821cea；用户列出的5e818cf9aa35是其父提交。
本轮新要求覆盖上一轮取消停稳的策略：恢复实际GO2速度/角速度以及0.20秒停稳。
仅修改SEARCH→STOP_CONFIRM→LOCK_TARGET的确认，不改Manager实现、MAP停靠、Point-LIO、
YOLO模型、S1抓取、相机互斥交接、持续三目标链或原停靠参数。
用户本轮报告单目标自主抓取已连续成功；本轮沿用该现场结论，不将导航日志统计冒充新的物理抓取验收。主视觉分支脏改动保留。

## 旧日志统计

固定8个最近运行目录、9份run JSON；均为HSV，不是YOLO真机漏检验收。
统计脚本：`camera_ws/scripts/analyze_stop_confirmation.py`；逐文件路径/原始SHA256和attempts
保存在`camera_ws/analysis/stop_confirm_fast_20261116/summary.json`。
7份成功LOCK_TARGET，2份未锁定；LOCK成功不等于本轮独立确认了物理抓取成功。
first_seen到LOCK平均23.703秒，逐次17.385、12.209、12.157、24.673、21.221、72.023、6.253秒。
first_seen为本轮SEARCH首次有效发现，可能包括候选更换；不能当作最终同一实物的专属识别时间。

27次已结束确认尝试：7次CONFIRMED、20次TIMEOUT。超时末次原因10次waiting for measured stop、
7次candidate missing or spatially ambiguous、3次collecting new stationary observations。
旧日志将缺失和歧义合并，不能事后捏造拆分数量。成功单次确认均值4.143秒，停稳耗时3.229秒、
停稳后复核0.914秒。17:03这次前8次超时、第9次成功，是23.703秒均值中的主要长尾。

重复等待包括Manager的5次确认、SEARCH自己的3次触发、STOP_CONFIRM停稳后重新至少5帧和
0.60秒跨度以及再次要求最新Manager CONFIRMED。速度波动或0.30秒断档清空采样，会重复这些等待。

## 新触发条件和回退

- `stop_confirm_fast_enabled`通用默认false，现有SEARCH配置true。
- `stop_confirm_require_stationary=true`：原Sport、机身定位和相机定位速度≤0.05m/s，
  Sport/定位yaw≤0.10rad/s、原数据新鲜度全部保留，连续0.20秒才允许复核锁定。
- SEARCH已Manager CONFIRMED候选只需2份不同源观测，保存ID、MAP中位数和有效前史；
  其余候选保留3份触发。自动左右侧/目标排序不改。
- 快速前提：前史至少2份且散布<8cm，停车前Manager已确认；当前同一ID唯一空间匹配。
- 停稳后3帧有效RGB-D，源时刻晚于最新一次连续停稳、不同且递增时间戳，与冻结种子≤15cm，
  MAP两点最大散布<8cm。默认3帧，可配置2或3；无需0.60秒跨度、不再次等待Manager确认。
- 不具备快速前史或ID发生变化时，回退完整至少5帧/跨度≥0.60秒、最新Manager CONFIRMED。
  仅关fast开关且保留require_stationary=true即可恢复5e818cf的原成功确认路径。
- 新配置最大帧间隔0.75秒，1秒观测窗口和0.5秒视觉新鲜度保持；缺帧时不能锁定。
  短时空快照保留SEARCH历史最多0.75秒。速度抖动保留目标历史，但重新计算连续停稳；
  旧停稳周期观测不能替代新停稳周期的3帧。候选歧义时清空复核样本且禁止锁定。

## 超时和多目标

每周期记录confirmation_path、confirmation_blockers、样本数/跨度/散布、停稳起点及原地复核状态。
具体原因：parking_unstable、target_lost、observations_insufficient、position_spread_excess、
target_ambiguous、candidate_position_inconsistent；原有stop_inputs仍完整记录速度/角速度/数据年龄。

6秒到期时若仍有新鲜、唯一、与种子一致的可靠目标，保持零速原地追加最多2秒一次。
追加期结束仍可见但无法确认则FAULT零速，避免同一可见目标反复SEARCH停车；
目标丢失或候选歧义则恢复原冷却SEARCH。没有无限延期或自动新增抓取轮次。
多目标排序/识别不改；快速路径比单纯位置关联多要求原ID一致，近邻多候选时拒绝快速锁定。
ID变化可经原空间范围内完整确认回退，不可直接按新ID快速锁定。

## 既有样本的针对性时序对比

只比较每份成功run最终确认尝试中，旧控制器已经接受的静止样本。
检查同ID、唯一源时间戳、<8cm散布、≤15cm种子一致性和原停稳输入；取最早具有3份样本的周期
与旧LOCK时刻相减：7份估算节省均值1.000秒、中位0.400秒、范围0.200～3.000秒。
5份在0.20～0.40秒附近，2份较长（2.30/3.00秒）包含旧流程后续停稳重判/额外等待。

这是采样时序条件的反事实估算，不是完整RGB-D/Manager闭环回放：日志没有完整的每周期Manager
原始轨迹快照及停车前确认状态，无法保证所有历史尝试都具备快速前提，不能承诺相同端到端节省。
可预期的直接收益是正常复核少等约0.3～0.4秒；减少重复停车的收益需要下一次实机验证。
没有调整实际停稳要求，因此原停稳耗时3.23秒不会被本轮直接消除。

## 验证与下一次实机

26项针对性SEARCH/快速确认测试通过，覆盖实际停稳、运动重判/历史保留、3帧、重复时间戳、
旧模式回退、缺帧、关联歧义、ID变化、散布和有限原地复核；未运行大型离线测试或训练。
新模块/测试/统计脚本flake8通过；原控制模块历史E128保留。
单包构建成功；安装的approach_control.py和search_confirmation.py与源码哈希一致，
SEARCH参数读回fast=true、require_stationary=true、stable=0.20、新帧数3；HSV full离线计划通过。
沙箱内构建首次在setuptools正则解析失败，重试等待子进程未结束；取消本轮等待后沙箱外构建0.54秒通过。
本轮新增效率、多目标快速关联及三轮完整full效果NOT VERIFIED。
本轮未执行真实GO2运动/抓取、远端部署、相机采集或服务重启。

下一次唯一full入口沿用成功日志的HSV检测器，使用新配置：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute --mode full --detector hsv --view
```

该命令会真实运动、趴下和抓取，需下一次现场授权与原急停/路线/机械臂支撑条件；本轮未执行。
PC systemd校时问题没有在本轮修复，不自动复用先前单次绕过校时授权。
