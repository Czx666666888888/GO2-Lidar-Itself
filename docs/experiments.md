# Experiment Record

## 2026-11-14 停稳计时：异常持续1秒才清零

用户针对22:21日志要求延迟归零。右/左抓取停靠配置新增
`dock_stationary_reset_sec=1.0`；默认0保持其他使用者的立即清零行为。
STOP_WAIT中位置/航向/实际停稳检查暂时不合格时，保持零速度并暂停累计，
不合格时间不计入停稳时间，不允许在不合格帧判成功；恢复后接着累计合格间隔。
异常连续达到1秒才清零之前累计；恢复后的首帧不把中间无效时段补算进去。
成功仍需当前<=8cm/10度且实际停稳，合格时间累计>=0.5秒；
此处按用户新指令从“严格连续0.5秒”改为“允许短暂暂停的合格时间累计0.5秒”。
停车等待的越界退出同样至少等待这1秒，防止提前恢复横向控制使计时失效。
真正退出STOP_WAIT开始修正、任务复位或FAULT时清空计时；定位/TF/反馈超时、
LIO/Sport矛盾及急停仍即时生效，不因计时防抖而延后。
JSONL补充good_sec、paused、bad_age_s、reset_sec及blockers；reset_reasons只记录真实归零。
78项针对性离线检查通过，无硬件动作；实机效果NOT VERIFIED。


## 2026-11-14 用户要求：关闭前后微调，8cm验收，保留横向修正

取代此前6cm精度和前向短步试用：右/左配置dock_position_tolerance=0.08，
零速等待退出带0.10m（仅滞回，不是成功精度），航向10度/实际停稳0.5秒不变。
新增dock_fine_forward_correction_enabled=false；首次进入最后15cm后锁定横向微调，
前后目标vx置零，沿用原速度出口和减速限制制动，不瞬间切断正在执行的运动。
随后只保留原vy/wz修正；即使定位波动回到15cm外，也不自动重新启动前后修正。
前向短步和死区补偿在左右停靠配置关闭，原短步实现保留但本模式不调用。
最后8cm指原MAP停车位平面距离；若停车后前后误差大于8cm，横向修正无法补足，
仍不能完成STOP/趴下，不能把“哪里停下”解释为任意误差都成功。
失视觉3秒停车保持关闭；5m预算、LIO/Sport矛盾停车、定位/TF/反馈新鲜度、急停保留。
75项针对性离线检查通过，覆盖前进/后退修正关闭、横向保留、平滑制动、漂出不重启、
7.5cm停稳成功与8.5cm不成功；未发硬件命令，实机效果NOT VERIFIED。


## 2026-11-14 用户要求：抓取停车位阶段关闭视觉年龄停车

取代上一条3秒视觉超时及到位后额外3秒等待策略：右/左抓取停靠配置
`dock_visual_loss_stop_sec=0.0`。只在完成SEARCH/STOP_CONFIRM并锁定目标后，
APPROACH_TO_GRASP_POSE使用原MAP锚点和Point-LIO继续接近、短步微调和STOP_WAIT，
不因视觉年龄超过3秒报FAULT；有可信新观测仍按原关联策略更新锚点。
不关闭视觉检测/相机，不改SEARCH、STOP_CONFIRM、普通approach或S1轨迹。
原5m失视觉续行预算、目标跳变/定位异常、TF/反馈新鲜度、唯一速度出口与急停保留；
位置<=6cm、航向<=10度、实际停稳持续0.5秒后才交接StandDown/抓取。
依据最新21:55 full_1794664520619960754（5b746ea）：21:56:01.769在42.104cm处
因视觉年龄3.032秒停止，未进入精调/STOP_WAIT，故用户明确要求取消该阶段视觉年龄停车。
71项针对性离线检查通过，覆盖左右实际配置、42cm失视觉继续、5m预算阻断和失视觉停稳完成。
本轮未发真实硬件命令；取消超时后的实机停靠/趴下/抓取NOT VERIFIED。


## 2026-11-14 到位后的视觉超时改为有界零速停稳

来源：21:45 full_1794663918545257953及science_approach_1794663940261429147.jsonl。
21:46:03.669误差5.804cm、航向4.97度，视觉年龄3.038秒先触发FAULT；
Point-LIO速度0.154、Sport速度0.056m/s，尚未满足停稳。未进入STOP/StandDown。
修改：3秒失视觉时，只有已满足当前位置/航向容差才进入arrival_zero_settle；
仅发零速，复用原STOP_WAIT/实际停稳证据，不再用新视觉作为成功必要条件。
其他位置仍失视觉FAULT；失视觉末端不再恢复修正，持续越界/航向超限要求重新确认。
有界期限=原settle_wait_sec+settled_sec+stable_window_sec（当前2+0.5+0.5=3秒），
未能停稳则FAULT；原5m预算、定位/TF/反馈新鲜度及异常检查保持。
成功仍<=6cm/10度且实际停稳持续0.5秒；不改速度、短步、S1轨迹或Point-LIO。
68项针对性离线检查通过，未发送任何硬件命令；本次末端交接实机NOT VERIFIED。


## 2026-11-14 YOLO贯通原S1完整流程（软件接入）

最新Push基线45e1391。此前approach独立YOLO接口绕过S1，而full仍使用launch默认HSV；
用户要求“这些都串起来”，本轮将YOLO接入原full编排，不以approach的10cm停车替代抓取停靠。

当前PC默认full/dock使用yolo，可显式--detector hsv；approach仍为仅视觉接近、无S1动作。
本地V2权重/venv路径和SHA校验提取为共用函数，full在S1准备前校验同一份模型，
实核模型e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1，未下载新权重。
PC时间包装器完整转发detector/view/model-path/python-executable；full正常支持--view，
沿用原录制器preview-image和debug_image，关闭可选viewer不触发控制任务错误。
--local-camera仍只属于approach，full/dock沿用伴随主机D435；本机aarch64流程未改。

full启动及失败后原有定位重建两条路径均显式传detector/yolo_model/yolo_python，
仅class0方块、同源RGB-D/实测ROI/精确时间TF→原目标Manager→原SEARCH/STOP_CONFIRM→
抓取位姿停靠→STOP→导航控制权释放→原StandDown反馈确认→S1右侧抓取→ZERO_HOLD→StandUp。
S1启动/复用、串口唯一所有者、姿态反馈、原抓取轨迹与串口协议均保留，未改Point-LIO、时间映射。
停靠仍是用户批准<=6cm/10度/实际停稳0.5秒，继续原明确未验证的估算短步参数；
不会因YOLO接入而改成approach的camera XY10cm或绕过STOP/导航释放条件。

启动探针扩展已有就绪检查：核对实际detector节点、连续新鲜有效输出、YOLO运行时V2模型hash。
健康会话仅在detector一致时复用；混合HSV/YOLO或请求不一致保留现场并要求先退出旧会话，
不热切换、不并行开两套正式目标发布者。full SEARCH可在无目标时开始，但检测器必须已有有效帧输出；
dock仍需原新鲜CONFIRMED目标。独立响应检查不加载YOLO或执行抓取。
复用full原连续定位/TF预检，不使用approach固定3秒等待作为full就绪依据。
伴随冷启动对时成功后再次严格verify_clock，不放宽50ms/200ms检查，也不把网络错误当时钟回退。

验证：45项入口/转发/复用接口/时钟模拟检查及9项目标选择/适配/S1序列检查通过，
共54项针对性检查；Python/Bash语法、关键lint/diff通过。
full--view离线计划及S1任务离线计划通过，明确包含StandDown/抓取/ZERO_HOLD/StandUp；
真实V2本地文件hash已验证（未加载推理或硬件）。原感知单包构建到full使用的.onekey overlay成功，
YOLO/选择器/适配器/停靠/短步/S1序列及launch安装文件逐字节一致，launch--show-args解析通过。
原full构建指纹已更新，不因文档提交重复构建；未构建全仓库。
本轮未启动相机/定位运行节点、未连接机器人、未使能S1、未行走/趴下/抓取。
本轮结论为软件串联完成，YOLO+停靠+机械臂完整实机效果NOT VERIFIED。


## 2026-11-14 approach一键入口对时恢复

最新Push基线ea1b127（独立YOLO/HSV方块接近模式）。用户日志offset=-132031297.091764秒、
RTT=0.000473秒；本轮SSH只读确认伴随主机date=2022-09-08、uptime约4分钟，没有相机/S1进程。
主因是重启后伴随时钟回退，不是该次SSH高延迟。approach新入口原来只有verify_clock，
缺少full/dock已有的调用sync_go2_clock.py恢复步骤，故无法继续到--view或运动。

修复：先只读确认远端相机所有者，再检查时钟；仅无相机且偏差超50ms时调用原交互对时脚本，
由用户在终端输入GO2 sudo密码，不保存密码；对时退出后再次执行原5次50ms/200ms RTT检查。
已有相机且时钟失配时保留会话，不修改运行中的时钟；SSH错误/RTT高/盘点失败不触发对时。
新增ClockOffsetError只区分时钟偏差与传输异常，不改变原阈值/时间映射；full原RuntimeError捕获兼容。
对时复核失败或取消密码仍停止，不启动相机、感知、预览、控制或S1。
原approach --view完整转发，显示器在相机/感知启动后创建；模式仍是camera XY10cm接近，
不进入S1/full抓取，不能与full的6cm停车参数混为一谈。

11项入口/时钟模拟检查及2项原PC时钟转发检查通过，Python/Bash语法、关键lint/diff通过。
仅改Python入口，无需ROS包构建。真实SSH只读时间/进程已核对；未执行对时写入、
没有启动D435/S1/GO2运动或实时预览。修复后真实对时到运动全链路NOT VERIFIED，用户自行执行。


## 2026-11-14 用户要求估算短步直接接入full（取消继续独立标定）

用户明确表示不追求精细标定、以微调后趴下为主要目标，随后要求“不测试了直接接入完整流程”。
按最新要求，不再以完整实测profile作为软件启动前提；不再安排本轮额外运动或标定试验。
已生成并安装config/forward_step_estimated_trial.json，类型estimated_trial，
reviewed=false、physical_motion_confirmed=false；不是已完成标定，不伪造审核/实测标志。

估算参数：正向参考0.20m/s（已观察实际出口达到），原模型减速度0.30m/s²、延迟0.30s，
模型停车距离0.20²/(2*0.30)+0.20*0.30≈0.1267m，文件向上取0.127m；
测得位移触发步长1～2cm，保留2cm不确定余量，MOVE/SETTLE各3秒为安全看门狗，
不是按速度乘时间确定运动量，不是持续0.20达3秒的固定脉冲。
该模型不是制动距离的实测保证。很近的误差如果原制动包络容纳不下，仍会明确停止，
没有为了实现趴下而删除包络、扩大6cm终验或强行制造成功。

左右dock配置均显式dock_allow_estimated_forward_profile=true并指定估算文件，
普通full已使用同一dock overlay，无需新脚本或额外CLI选择。估算加载须显式opt-in，
测量profile仍保留原物理审核/证据hash检查；日志区分profile_available=true、
calibration_ready=false、profile_kind=estimated_trial。
复用原ForwardStep MOVE/SETTLE、真实位置增量、唯一速度出口与加减速限制。
仍需<=6cm、<=10度、实际停稳0.5秒及控制权交接，随后执行原StandDown/S1/StandUp。
SEARCH、vy/wz计算、底层定位/识别/时间映射、S1轨迹/协议不改。

用户停止测试指令前已完成35项针对性检查与单包构建；之后仅核对安装配置/源码字节一致、
估算文件可由full的实际参数加载、更新构建指纹及文档，没有新增实机动作。
本轮接入已完成；估算参数下的独立dock/full物理效果NOT VERIFIED，用户选择直接用原full入口。
唯一完整入口仍为bash go2_test.sh --execute --mode full，未由助手执行。


## 2026-11-14 第2步一次受控测量结果（20:39:29）

本轮实际前向运动仅一次：run_1794659964588482850、full_1794659950602237662，代码af89596。
此前外层参数、密码、重启后S1缺失、Humble回调及原生DDS归属问题均未发前进指令；
对应记录保留，不能把准备失败计成运动重复试验。
用户已授权0.20m/s、35cm保护、8秒/横偏3cm/航向3度；另授权恢复S1及原有对时。
S1恢复ZERO_HOLD后复用，未抓取；本轮无搜索/趴下/起立。

最终出口按本次gate租约收到46份记录，峰值0.200m/s，达到参考速度时间14403.644722217。
随后在20:39:29因heading边界FAULT：Point-LIO yaw=-3.20918度（阈值3度），
同段Sport角速度积分约-3.98986度；操作者报告已停稳、未看到明显偏转，未提供实际距离。
不能以肉眼未见明显偏转认定角度为零，也不能把两路估计直接当独立物理角度真值。
控制器零速请求14403.944696005，出口StopMove观察时间14404.046034735，间隔约0.10134秒；
末尾Sport速度约8.63e-8m/s，定位体速约0.00777m/s；用户物理确认停稳。
零速时原始定位前移0.122307m，首次FAULT记录0.129005m；均为定位值，非独立尺量。

本轮完成了0.20出口与归零观测验证，但航向保护打断正常停车流程，不能将故障StopMove
对应距离当作正常制动profile。calibration_accepted=false，未生成标定文件、未改审核标志。
按一次运动授权到此结束，不自动重复；第2步正常停车验收仍未通过，第3～5步未执行。
下一步需核对固定方向参照/录像及真实位移，确认小角度偏转和测量可信度，再讨论新测试授权；
没有擅自放宽3度、停车精度或其他保护。分析与原始hash见
camera_ws/analysis/forward_response_20261114_203929/summary.json。


## 2026-11-14 出口记录兼容原生DDS（测量前修正）

run_1794659738325666609仍未发前进命令，因出口归属过严在IDLE停止。
只读ROS图核实/api/sport/request在安全门退出后仍有9个_CREATED_BY_BARE_DDS_APP_原生发布者。
它们不能当作第二个ROS速度控制器。修正沿用原审计排除原生参与者的逻辑，
同时从本次实验自有gate日志读取实际租约ID，按Request.header.lease.id筛选本轮门的输出。
Humble无逐包GID，故明确记录租约+唯一当地ROS门归属，不伪称GID可用；其他ROS控制者仍阻断。
24项针对性检查通过，独立本机ROS域87实传Move/StopMove并模拟原生endpoint共存验证通过；
单包构建/安装读回通过，无额外速度发布者。此前失败均在前进指令之前，尚未物理测量。


## 2026-11-14 实测前Humble出口回调兼容修正

本轮授权的一次测量尚未发前进指令。先后记录：外层PC时间入口未透传35cm参数（已修复）；
本机密码需交互；远端重启导致S1/Socket消失（用户另授权恢复，已恢复ZERO_HOLD并原对时）。
随后run_1794659502828542193仍在IDLE、仅StopMove时节点退出：
TypeError: ScienceTargetApproach._sport_output() missing required positional argument info。
当前安装的Humble executor只传消息正文，take_message元信息仅含source/received时间，没有GID。
修正为单参数回调，以当前graph唯一go2_safety_gate发布者归属记录；有其他发布者即明确停止。
不伪称逐包GID验证，不改ROS执行器，不加起步基线门槛。32项针对性检查及新增单参数回归，
独立本机ROS域87/localhost真实Move和StopMove回调测试通过；该域未连接GO2。
单包构建与安装一致性通过，随后才继续尚未执行的同一轮授权测量；未增加物理测量次数。


## 2026-11-14 前向标定测量修正（第1步，未运动）

最新Push基线7e9c865。复核20:15:13的run_1794658509091411854：最高控制请求约0.1800m/s，
窗口3.4763cm触发零速时原始定位已约7.8cm，后续触及20cm原始位移保护；
stop_extra_m=0.1212m混入窗口迟滞，不能当作真实制动距离，更不能生成0.20标定profile。

本次只修正受控响应测量，未增加起步基线门槛、未改导航架构/原5m预算/SEARCH/S1动作：
- 只读订阅既有/api/sport/request，按DDS publisher GID只接收go2_safety_gate的输出，
  API1008速度/API1003零速逐条写原JSONL；不增加任何速度发布者。
- 实际输出达到参考0.20后，记录该时刻原始vehicle pose为测量段起点；
  再前移2cm即请求零速，使用原始位姿而非延迟的移动中位数触发停车。
- 原始/窗口位移并列，独立记录控制器zero_requested、outlet_zero、实际停稳证据和确认时刻，
  记录请求/出口峰值、原始pose采样时间、明确的越界原因。
  观察到DDS请求不等于电机实际执行，物理位移仍需现场标记/固定录像核对。
- 未达到参考速度、未观察到出口归零、未完成停稳均不算完整响应测试。
  physical_review_required=true、calibration_accepted=false，不自动填审核标志。
- 默认20cm/8秒、横偏3cm/转角3度保护保留。新增仅响应测试可用的
  --response-distance-limit参数（0.10～0.50m），不影响普通dock/full或5m续行预算。
  离线计划明确打印所选边界，普通dock/full禁止使用该覆盖。

原模型a_up=.15、a_down=.30、latency=.30，达到0.20的理想加速路程.1333m，
再测2cm，模型减速.0667m加延迟.06m，合计约.28m。
这只是原模型估算，非已测制动值；因此准备35cm边界的下一轮明确测试请求，
未经新运动授权不执行，不自动扩张默认20cm限制来绕过失败。
建议现场前方至少1m空旷、急停在手、有人监护、S1健康ZERO_HOLD，
准备地面标记及固定录像/卷尺确认实际位移；本轮最多先做一次。

32项针对性检查（响应阶段/出口解析/原停靠接口）、语法、关键lint、单包构建与安装读回通过。
启动器既有F841未使用变量告警已识别为历史代码，没有混入无关清理；其余关键lint通过。
原速度/位置/时间/反馈失效保护保留；本轮未启动GO2或S1动作。
真实响应、制动profile和独立dock均NOT VERIFIED，尚未宣称完成标定。


## 2026-11-14 移除响应测试额外的单次基线门槛

最新run_1794658282823146261、full_1794658268393168074，20:11:25首次故障为
forward response check requires stable trusted pose baseline。安全门只有StopMove、没有前向Move。
该门槛是前轮增加的起步瞬间定位窗口判定；未记录具体blocker，不能从10Hz快照反推
完整内部高频窗口。故障时体速约0.0187、相机速度0.0175m/s，Sport速度近零；
保留快照重算窗口体/相机速度约0.0244/0.0230m/s、半径约1.41/1.44cm均通过，
不足以确定启动回调瞬间是哪个内部指标不通过，不能说成0.20m/s执行失败。
按用户明确要求删除begin_response_check中的额外一次性稳定基线拦截，不新增等待流程。
保留原新鲜度/TF/Sport反馈、唯一速度出口、加速度、20cm原始位姿/8秒、3cm横偏和3度航向保护。
测试参考0.20，窗口位移记录和实际停稳判据不改；数据有歧义仍不能生成可信标定profile。
21项针对性检查（含新增不完整额外窗口不再起步FAULT回归）、语法/diff与单包构建通过；
安装源码逐字节一致并更新内容指纹。本轮未发送任何运动指令，实际响应仍NOT VERIFIED。


## 2026-11-14 19:48:07停靠失败分析与最小控制修复

基线：fetch最新Push为0444d33cbfdc77296f766bf024ea5db49ab90de1。本轮无实机运动。
用户后续明确：0.15m/s不能移动，0.20m/s可稳定移动；这是用户现场反馈，不是本轮控制实验。
用户最后批准停车距离4～6cm均可：本轮配置验收改为<=6cm，航向10度及停稳0.5秒不变。
SEARCH、STOP_CONFIRM、vy/wz计算、Point-LIO/时间映射/D435算法、S1轨迹/协议未修改。

### 原始证据及可确认边界

本轮失败run_1794656817517152442，full_1794656799004878859；
控制/安全门/定位相机日志、视频检测sidecar已读取，原始文件哈希见
[camera_ws分析证据](../camera_ws/analysis/docking_20261114_194807/evidence.json)。
原记录里程4.993814666m；可恢复的10Hz转换后vehicle位姿折线4.614532627m，
同段净位移0.498057865m，Sport速率积分0.530736006m。
原始逐帧Point-LIO Odometry/完整高频pose回调未保存，不能把10Hz快照冒充原始Point-LIO。
新的0.5秒移动中位数、2cm分辨率保留锚点估计，在同份10Hz记录上为2.541488015m；
相对4.9938m减少2.452326651m，但它包含采样差异、滤波及未分辨往返，**不是确认的噪声米数**。
Sport积分也不是独立真值。没有外部物理测量，实际位移/定位抖动/漂移各占多少无法定量确认；
不能写实际只走了0.53m或余下全是噪声。新证据一致性检查在同份记录失视觉后约6.20秒
已可检出路径证据持续矛盾；这是离线检查结果，不是改控制后的物理轨迹预测。

### 六次STOP_WAIT完整序列

逐点vehicle/target MAP XY、航向误差、原始Sport vx/vy/wz、控制指令、
body/camera/Sport速度与停稳计时阻塞原因，见
[完整CSV](../camera_ws/analysis/docking_20261114_194807/stop_wait_series.csv)。
六次退出均为当前距离超过原6cm退出阈值，没有航向超限、2秒等待超时或目标锚点变化：
- 第1次：3.019cm → 6.363cm，持续0.5s，退出航向误差7.44度，目标MAP变化0。
- 第2次：3.816cm → 6.859cm，持续0.4s，退出航向误差7.13度，目标MAP变化0。
- 第3次：3.906cm → 6.325cm，持续0.5s，退出航向误差7.09度，目标MAP变化0。
- 第4次：3.838cm → 6.557cm，持续0.4s，退出航向误差6.98度，目标MAP变化0。
- 第5次：3.885cm → 6.987cm，持续0.6s，退出航向误差7.37度，目标MAP变化0。
- 第6次：3.733cm → 7.202cm，持续0.4s，退出航向误差7.16度，目标MAP变化0。

六段共28个STOP_WAIT状态样本，仅13个在原4cm内；只有一个样本启动过停稳计时，
未持续0.5秒。目标锚点在这些段固定，波动来自vehicle MAP位姿；Sport几乎静止，
支持检查定位抖动/漂移和姿态摆动，现有证据不能独立排除真实超调。
精调483个样本中479个正vx<0.15m/s、443个Sport合速度<0.01m/s。
配置0.18只是平面请求下限；safe=sqrt((a*latency)^2+2*a*clearance)-a*latency，
随后还受实测速度制动、方向分解和向量加减速限制。例旧4cm容差下、a=.30/latency=.30，
6cm距离时safe约0.089m/s；斜向运动的vx会更低。不能把请求下限当作最终速度保证。

### 失视觉

最后有效检测色彩图戳1794656834492360840，中心(621.845,452.285)，在640x480画面的
右下边缘，深度约0.389m。之后442份检测，343份no blue candidates，99份聚类后无有效目标。
原始录像抽查100/200/300/600/950帧，后段目标已不在可见画面内；相机仍连续出帧。
不能把目标失效说成USB掉线，也不能只改识别阈值假装恢复。
旧逻辑没有dock视觉时效停止，允许追旧MAP直到5m路径预算耗尽；最终首次故障时误差
5.8819cm、视觉年龄53.2203秒、剩余预算6.19mm，未STOP/未释放导航，所以没有StandDown。

### 改动及验收边界

1. 新dock-only docking_motion时间窗估计：移动中位数、保留锚点累计慢速持续运动和
   可分辨往返；保留raw/windowed/Sport/未分辨差额。Sport为零不删除LIO位移，
   两个审计窗口累计路径矛盾超过8cm时停止并报localization untrusted，5m预算不变。
   2cm以下/窗口内快速往返不能保证分辨，明确标未分辨；本轮不宣称精确真实里程。
2. STOP_WAIT使用0.5秒位姿趋势及2cm包络估计实际停稳，仍要求新鲜Sport/TF/定位及
   原速度/角速度阈值。单帧越界不立即重启：退出还需窗口中心超外圈并持续0.3秒。
   终验必须当前<=6cm、<=10度且实际停稳持续0.5秒；外圈8cm仅零速滞回，不是成功容差。
   超6cm持续2秒仍恢复必要修正。新增每次退出原因和计时器阻塞原因JSONL字段。
3. 用户要求短步打开：左右dock overlay的dock_forward_step_enabled=true。
   正向参考速度改为用户确认有效的0.20m/s；不把它当全局速度下限，停车可为0。
   仍复用ForwardStep MOVE/SETTLE和实际投影位移、原制动/加减速出口。
   缺少真实制动profile时，必要正向精调直接零速故障并提示标定，不再持续无效低vx。
   左右/后退不加死区补偿；vy/wz原控制计算保留。profile速度必须匹配当前0.20，
   旧0.15记录不能冒充有效数据。已启用不等于已经完成制动标定。
4. 有效视觉超过3秒即停车并要求重新确认目标，不无限追旧MAP；不自动重新SEARCH。
5. 复用唯一入口的forward-response-check，按配置缓升至0.20，窗口测得2cm请求零速，
   原20cm/8秒、横偏3cm/航向3度保护保留。起步需稳定可信定位基线，测量不等于v*t；
   保留原始20cm位姿边界，不能因滤波延迟漏掉越界。无稳定基线不发送测试速度。

103项针对性检查（其中8项新增回归）、Python语法/关键lint/diff通过；
第一次未带ROS PYTHONPATH检查有1项导入失败，补齐原环境后全部通过。
第一次沙箱colcon执行器阻塞后仅中断本轮构建；正常环境顺序单包构建成功，
安装的6份控制源码及左右配置逐字节一致，已更新内容指纹，后续不重复构建。
本轮没有连接新SDK、机械臂使能、GO2行走、趴下或抓取。
**下一步只具备0.20m/s受控响应测试的软件条件；完整dock仍待制动实测profile和定位证据验收。**
当前forward_step_profile_file为空，不能承诺普通dock能完成全部末端动作。


## 2026-11-14 修复零位恢复后导航许可锁存

### 本轮授权后的实际部署结果

用户确认可靠支撑、空间无干涉、急停在手，授权仅S1升级及安全重启。
旧PID3365已空闲/零位/通信正常，按原SIGTERM流程正常退出；确认USB0释放后升级。
兼容层v2_usb01_feedback_2已部署，新右侧PID5633唯一占用USB0。
启动时已在原容差内，日志STARTUP_ZERO_SKIPPED_ALREADY_AT_ZERO；未执行非零位回零。
连续12次真实Socket采样均ZERO_HOLD、busy=false、at_zero=true、navigation_allowed=true、
communication_state=OK；最大关节绝对偏差0.0268944rad。旧状态许可false的阻塞已解除。
保留新版保持服务；未启动GO2行走、趴下或抓取，full仍未实机验收。
证据本机camera_ws/records/s1_nav_recovery_9448301/（before.json、after_samples.json），
运行日志camera_ws/records/s1_nav_recovery_9448301_run.log。
采样SHA256：`b327eeddf782d5af1c64ef5e81eb8a763458d59c039e4df181f30c1c2af948e1`。超限后再次恢复的连续硬件过程未人为制造，已由离线回归覆盖。


来源：用户异常输出、原Socket只读状态和适配源码。J1先为-0.0803009rad（超过0.08rad），
随后已回到-0.0268944rad，at_zero=true、readiness_state=ZERO_HOLD且通信正常，
但navigation_allowed仍false。根因为snapshot只在超限时写false，恢复后不重算true。
修复为每次有效反馈按ZERO_HOLD、busy=false、at_zero及无服务故障重算许可；
通信异常、忙碌、真实FAULT、仍超限均不放行。原0.08rad容差、轨迹、协议、GO2验收不变。
兼容层版本v2_usb01_feedback_2，升级只接受原版/已部署上一版/本版精确hash，
按hash保留升级备份；运行中不改服务文件或创建第二所有者。
51项针对性离线检查通过，覆盖超限→恢复、忙碌/故障/通信异常阻断及上一版hash升级；
Python语法、关键lint、diff检查通过。本改动不涉及ROS包构建。
远端原服务仍运行旧代码，升级后真实状态恢复NOT VERIFIED，需支撑确认后安全重启S1。
本轮未发送回零、抓取或GO2运动指令。未把软件修复写成硬件部署通过。

## 2026-11-14 修复S1进程清单误报历史版本

用户full日志在connection_and_inventory阶段拒绝/home/unitree，未进入导航/姿态/抓取。
源码确认原pgrep按整条命令行匹配，而SSH检查代码自身包含左侧脚本名称，可能匹配其外层shell；
同时原版本判断用cwd，绝对路径启动的真实v2服务也可能被误报。
现按/proc读取Python实际脚本操作数，排除shell、python -c/-m和其他脚本参数中的名称；
相对入口结合cwd解析，绝对入口直接解析，按真实scripts父目录判断v1/v2。
仍拒绝真实旧版/多所有者/不一致端口，未删除安全检查，未停止任何进程或重启机械臂。
42项针对性离线检查、语法/关键lint/diff通过；远端执行完全相同的只读清单返回
arm=[]、task=[]、camera=[]、USB0唯一存在、D435/D455枚举正常，无历史S1进程。
原始清单：本机camera_ws/records/s1_v2_inventory_fix_20261114/inventory.json。
本轮没有运行dock/full，运动、自动回零及抓取效果仍NOT VERIFIED。

## 2026-11-14 S1 v2 独立部署、兼容层及真实零位保持

- 基线：fetch后本地/远端ce20ba909dbecc99a6858ede143e9c79b8cf4c73。
- 用户原ZIP：S1_arm_integration_20261114_auto_v2.zip，14,098,177 bytes；
  SHA256 `892c7be031fd52bceaf335caeb189567de364afecaa20d21de1a2f047afee3d9`。
  CRC、路径、BUILD_INFO和原SHA256SUMS通过；不提交ZIP或大日志。
- 实际安装：GO2伴随主机.18 `/home/unitree/s1-arm-control-v2`；
  保留旧目录，部署前后旧版scripts/s1_control/config/calibration共111文件哈希一致。
  原先没有S1服务/串口所有者，故没有停止旧保持服务或抢占。
- v2专用适配`v2_usb01_feedback_1`：默认auto兼容唯一USB0/1，全服务锁+原设备锁；
  设备inode/rdev改变即通信异常。原v2文件备份`.v2-compat-upstream/`，
  `S1_V2_COMPAT.json`核对适配哈希，其余使用原清单；不修改抓取算法、轨迹或协议。
- 用户本轮明确授权右侧启动及原startup_zero，排除GO2运动/趴下/抓取。
  只读反馈有效：q约[-0.037194,-0.023460,0.000572,-0.005150,0.026894,0.049020]rad，
  温度17～20°C；原0.08rad零位容差内。服务日志正确输出
  `STARTUP_ZERO_SKIPPED_ALREADY_AT_ZERO`，随后ZERO_HOLD。PID3690、USB0唯一所有者。
  8次连续Socket采样以及只读定位检查后均busy=false、navigation_allowed=true、at_zero=true、
  communication_state=OK。最后最大绝对关节角约0.0132rad。保留该服务，不为清理而撤保持。
- Python3.10复用已有环境及SDK，没有重装依赖。右侧服务真实启动；
  左右`verify_handoff.py`离线预检和包完整性通过，仅右侧访问机械臂。
  D455序列231622302209，15次彩色/深度frameset返回，启动初期有重复帧/编号重置，
  随后双流递增；这是短时接口证据，不代表抓取识别验收。D435序列213622070598。
- 原生LiDAR只读5秒：74云/1250 IMU样本、25个LidarState，健康检查PASS。
  伴随时钟原为2022年，操作者通过原sync_go2_clock.py终端完成对时；
  未改时间映射算法。短时链检查首次因动态导入缺少scripts路径失败，修复后重跑。
  8秒有效采集：D435、Point-LIO、连续输入、时间映射、时间戳限制、精确base/camera TF全通过；
  119个track快照，0个coarse/map_point/selected，没有CONFIRMED目标；
  整体为`NOT VERIFIED: exact_target_map, confirmed_selected_target, fresh_confirmed_target`。
  无数据不足以判断目标未检出的原因，未改感知算法。检查自有相机/定位进程已正常收尾。
- 软件交接核对：只用status/grasp，没有zero RPC；同侧健康服务复用，不重新初始化；
  STOP释放导航→新鲜停稳反馈→StandDown反馈确认→原抓取→新结果ZERO_HOLD→StandUp逻辑保留。
  41项针对性离线检查通过（实际ZIP适配分支、串口/状态接口、原姿态序列）；
  Python/Bash语法、diff检查通过；按内容缓存仅构建变化单包成功，非全仓测试。
- NOT VERIFIED：非零位startup_zero实物运动、USB1换号、导航运动中ZERO_HOLD、真实dock、
  StandDown/抓取/StandUp。SDK未提供独立电机包时间戳；关节反馈不等于独立物理测量。
  本轮无dock/full运动授权，未运行。右侧Y=0.41m与v2建议约0.31m仍需实际位置和支撑验收。
- full目前只具备S1服务和软件交接条件，未具备本场景完整验收：仍需有效MAP目标、
  新授权独立dock通过、机械臂路径/机身支撑/抓取位置确认，再由用户执行full。
- 原始记录本机`camera_ws/records/s1_v2_deploy_20261114/`；其中right_startup.log、
  zero_hold_samples.json、readonly_hardware.log、final_remote_verify.log。
  定位证据`camera_ws/records/time_20261114_191821_s1_v2_readiness_retry/report.json`，
  SHA256 `e82f6ef5852108726b33a2ac2e2985e2fb69329734049aae8fa8c3bbb35b4541`。本轮失败首次报告及重试日志均保留。


The entries below are a structured transcription of the root `README.md`. No rosbag was replayed and no metric was recalculated during creation of this document.

## Evidence Rules

- **Recorded observation:** text already present in the repository README.
- **Interpretation:** only the interpretation stated in that README; no new causal claim is added.
- **Raw data:** the local `rosbags/` directory is ignored by Git. Raw rosbag stored locally / not tracked in Git.

## retest4

- **Experiment ID:** `retest4`
- **Configuration:** Current README associates the representative runs with the 2026-08-31 rollback code, but does not provide a per-run parameter manifest.
- **Observed result:** Duration 78 s; moved 0.82 m; 94% of the time was spent turning.
- **Interpretation recorded in existing documentation:** The robot could move, but turning dominated the run.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest4_20260901_153430/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest5

- **Experiment ID:** `retest5`
- **Configuration:** Per-run details are not present in tracked documentation.
- **Observed result:** Duration 258 s; recorded as the day's farthest run at 2.96 m; 67% of the time was still spent turning.
- **Interpretation recorded in existing documentation:** Best recorded distance that day, but excessive turning remained.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest5_20260901_160622/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest6

- **Experiment ID:** `retest6`
- **Configuration:** README identifies a corridor-check variant but does not provide a complete parameter manifest.
- **Observed result:** Duration 219 s; almost no movement.
- **Interpretation recorded in existing documentation:** Corridor checking falsely rejected motion (“走廊检查误杀”).
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest6_20260901_173037/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest8

- **Experiment ID:** `retest8`
- **Configuration:** README says this run was after rollback; no independent configuration manifest is tracked.
- **Observed result:** “能走一点” (some movement); no duration or distance is documented in README.
- **Interpretation recorded in existing documentation:** Movement returned after rollback.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest8_20260901_175103/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## retest9

- **Experiment ID:** `retest9`
- **Configuration:** README says this run was after rollback; no independent configuration manifest is tracked.
- **Observed result:** “能走一点” (some movement); no duration or distance is documented in README.
- **Interpretation recorded in existing documentation:** Movement returned after rollback.
- **Raw data availability:** Raw rosbag stored locally / not tracked in Git (`rosbags/full_chain_a_retest9_20260901_180538/` observed during repository import).
- **Verification status:** `NOT VERIFIED` in this documentation cycle.

## Missing Evidence

- The referenced `today_representative.md` and `today_analysis_20260901.tar.gz` are not tracked in this repository.
- Exact metric calculation methods, per-run commit hashes, parameter dumps and environment metadata are not recorded in the root README.
- Future experiment entries should include commit hash, parameter snapshot, command, host/robot configuration, bag checksum, safety conditions and analysis command.


## 2026-11-14：YOLO单类别方块MAP接口（仅代码与软件检查）

- 来源：远端abb83f5/caf6ab6、两分支实际源码、12项针对性检查、集成感知单包构建30.3秒、venv ROS/YOLO导入与launch参数/互斥检查。
- 固定V2 best e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1；仅class0，完整帧结合Aligned Depth/CameraInfo，ROI实际有效像素投影，精确源时间MAP转换。
- 新适配器一次选最近可测方块并空间锁定，缺深度/歧义不更新，仅单目标兼容PointStamped进入原Manager/Approach；近邻≤0.40m拒绝。HSV/YOLO互斥。
- 原控制器、Manager、转换器实现及camera_origin/0.10m配置未改，hash证据见camera_ws/docs/evidence/color_block_interface/unchanged_integration_controls.json。新增集成approach分支，不启动S1/抓取，不改full/dock功能。
- 用户改为先完成代码，本轮不连接设备/预览/运动；没有相机/MAP实测坐标、停车距离或物理停止结论。上述真机项均NOT VERIFIED；后续先静态误差检查再另授权单方块、多方块。
- 完整命令、数据路径和失检ODOM行为：camera_ws/docs/color_block_map_approach.md。早于build的参数检查及默认只读ROS日志目录检查失败，build完成并指定/tmp日志后解析通过。
