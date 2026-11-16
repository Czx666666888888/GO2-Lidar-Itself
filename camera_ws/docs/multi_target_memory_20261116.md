# YOLO V3多目标MAP记忆与连续处理（2026-11-16）

## 基线、范围与证据

开始fetch远端feature/map-approach-s1-grasp-integration，HEAD/FETCH_HEAD均7c170210fbe1968e9dc22d0e899879f3edd62073。
用户要求保留已成功单目标链，接入多目标记忆；本轮补充答复明确“障碍接口后续提供，先完成多目标识别和抓取”。因此本轮没有新增障碍点云模块或避障：通行判断/绕障仍NOT VERIFIED，下一次只能在人工确认空旷场地测试。
既有诊断full_1794834675902500879中71帧深度有效却无MAP新候选，为旧单目标锁与持续任务不兼容的证据；本轮不把这些帧当作新方案实机通过。
V3权重、640/.25阈值、深度有效规则、Point-LIO、MAP停车几何/速度/停稳、原成功抓取轨迹均保持。

## 数据流及记忆

仅规范YOLO/full启用multi_target_memory；HSV/dock/独立approach参数默认false。YOLO继续检测color_block，本帧每个深度有效、精确源时间TF可用的方块全部送入原PointStamped MAP观测topic。单个无效深度方块不阻断其它有效方块；不把它伪造为3D目标。
沿用Manager按源时间戳批处理及TargetTracker一对一ID关联/最近窗口中位数融合；增加重复/倒退源帧拒绝。只在启用记忆时保留已确认STALE轨迹；未确认候选仍按原超时删除，原默认模式生命周期不变。
目标原始ID、融合位置、末次实际观测时间、计数、位置散布发布在原/science/target_tracks，并每0.5秒原子保存到本次full目录target_memory.json。
COMPLETED/MISSED等处理结果在round_NN_result.json及three_grasps.json，按target_id/MAP位置对应几何记忆。几何轨迹状态CANDIDATE/CONFIRMED/STALE与任务处理结果是两个维度；不把STALE等同MISSED。仅本次MAP会话使用，不跨SLAM重启盲目恢复旧文件。

## 选择和接近

首轮沿用SEARCH→STOP_CONFIRM→原MAP停靠。多目标由已有左右停车位函数比较平面距离，固定选侧则只比较该侧；距离/原运动预算不满足者不执行。相近目标无法唯一关联时不强选。其余目标只在移动观测中建立各自历史，不逐个停车确认。
后续轮次收到原任务成功/安全MISSED且起立确认的已处理位置列表，排除15cm内位置；优先选择300秒内曾确认、历史散布<8cm且在5m原范围内的最近停车位目标。没有合法记忆则走原SEARCH。
记忆目标直接初始化原MAP锚点并进入LOCK_TARGET/原GraspDocking，不伪造新RGB帧或新的observed_ns，不重新进行完整SEARCH/停车采样。原实际定位/TF/Sport反馈、控制周期、定位跳变和5m里程续行保护仍有效。
途中重新看见目标时，只接受初始20cm/当前15cm内唯一空间匹配；收集3个不同源时间戳的新观测、间隔≤0.75s且散布<8cm后用原融合更新。重复帧、歧义或不一致不更新，也不切换其它目标。
无视觉时仍沿历史锚点执行原MAP停靠；定位/TF/反馈失效或原运动边界超限停车。不能宣称这能识别障碍；障碍接口按本轮用户答复留待后续。

## 到位后的D455、MISSED与起立

仅记忆目标或当前已确认多个目标时开启allow_missed；场景始终只有一个物块的首轮保留原S1成功及拒绝路径。到位仍原StandDown/相机交接，单一串口所有者运行原D455观测部署。
S1兼容版本6在原上游脚本“第一次抓取前拍摄”后增加窄分支：只有有效捕获、NO_TARGET且支撑平面quality_pass=true、同时本轮allow_missed才触发MemoryTargetMiss。不会下发抓取下降/夹取；使用原已存在、碰撞检查的部署逆向返回与保持路径，完全返回后结果VISUAL_NO_TARGET_SAFE_RETURN/exit3。采图失败、无效平面、恢复失败不伪装无目标。
原接触未命中VISUAL_ADAPTIVE_CONTACT_MISS_SAFE_RETURN、夹持丢失VISUAL_ADAPTIVE_SECURE_LOST_SAFE_RETURN也是明确安全MISSED结果。只有本轮新日志、明确exit3/枚举、busy=false、ZERO_HOLD、at_zero=true及通信OK/readiness=ZERO_HOLD，才允许原StandUp；姿态反馈仍必须通过。
未知非零结果、串口/机械臂FAULT、回零失败、心跳/姿态异常均中止，禁止自动起立/下一目标。软件动作完成标COMPLETED，实际入盒仍placement_confirmed=null；安全无目标/接触未命中/夹持丢失标MISSED，两类都排除后续选择。
默认最多处理3个不同目标（含MISSED）；processed_count、completed_count、missed_count分别记录。记忆用完且不足3个恢复SEARCH，达到3个确认起立后停止。

## 结果保存及运行边界

起立结果先保存round_NN_result，再恢复D435；恢复阶段要求完整ready（RGB-D+定位+TF），失败保留已处理计数并中止。已处理不等于允许继续运动。协调器stdout/stderr另存round_NN_coordinator.log；心跳写/关闭与进程退出相遇的BrokenPipe会等待子进程退出并验证本轮RESULT，不能吞掉其它故障。
原D435/D455释放/恢复顺序、S1唯一串口所有权及左右参数复用保持。旧单目标adapter的轮间解锁参数接口留作兼容；本模式不再使用它筛掉其它目标。

S1版本6由本地用户原始ZIP按源哈希生成，所有已改文件保留原来源/备份，verify_handoff校验同步；旧版正在运行时拒绝热替换。下一次full前须在安全状态结束旧S1服务；本轮没有停止、升级、重启或动作。不得为升级而在机械臂未回零/支撑不明时强杀后启动。

## 定向验证

- 127项定向离线检查通过（约1.4秒）。覆盖多目标ID/暂停后记忆/源帧唯一性、最近停车位/已处理排除、无视觉续行与定位超时、3帧复核、全有效方块MAP发布、MISSED/故障禁止起立、计数/收尾保存、原SEARCH/快速确认/相机交接及S1源绑定兼容做短离线检查；不运行大规模视频回放。
- 实际用户原始S1 ZIP（SHA256 892c7be031fd52bceaf335caeb189567de364afecaa20d21de1a2f047afee3d9）已用于兼容补丁生成/compile及升级备份模拟；无导入执行其SDK/机械臂动作。
- 单包go2_science_perception构建成功，安装版approach_control/Manager/adapter/sequence与源码SHA256一致；静态编译、diff检查和FULL离线计划通过。
- 尚未验证真实三物块识别连续性、目标身份、实际停车/无视觉行进、无目标返回和StandUp、S1版本6部署；全部NOT VERIFIED。
- SSH只读读取NX原脚本两次被远端关闭，后续源核查使用本地原始ZIP；未写NX。现有定位跳变、USB异常的物理根因未在本轮修复。

唯一full测试命令（仅提供，未执行）：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute --mode full --detector yolo --view
```
