# 指定21:48录像：YOLO到MAP阻断审计与证据记录修复

## 对象与版本

仅分析用户指定的`full_1794836905838133718/approach_20261116_214843_509039.mp4`。
故障发生于第一轮抓取之前，three_grasps.completed_count=0，不是轮间解锁问题。
2026-11-16 fetch/ls-remote核对远端集成最新Push为7c170210fbe1968e9dc22d0e899879f3edd62073，
与该轮manifest一致。主视觉远端为1785c9a416ae8c2df1fcb94e8a9d54a5243ce4df。
既有/tmp/go2_onekey_s1内有另一项未提交的多目标记忆工作，本次使用独立干净clone，未覆盖或提交它。
不以最新时间目录中的另一轮替代本轮证据。

## 六项核对结果

| 检查 | 指定轮次证据 |
|---|---|
| YOLO与MAP | 584份检测报告，139份非空，134份至少一个有效深度；137个accepted深度目标。正式MAP发布6次，全部转换成功。最后MAP之后仍128份有效深度检测。 |
| selection reason | 未录制/science/color_block/selection，分布Unknown；不能伪造0拒绝。 |
| 20cm/15cm拒绝 | 未记录每帧candidates_map/initial/current，次数Unknown。存在锁机制不等于该机制已被证明触发。 |
| RGB-D/精确TF | 4个检测目标的depth reason为rgb_depth_time_mismatch，1个为insufficient_valid_depth。原始depth_stamp/delta和adapter TF异常未保存，精确TF失败次数Unknown。FINAL_MAP_TRANSFORM failed=0不能覆盖发布前检查。 |
| Manager | slam.log记录NEW_TRACK 1/2/3；ID1/2各一份，ID3四份后均CANDIDATE过期，无CONFIRMED。已证明收到6个MAP点。 |
| SEARCH | 收到6个不同源时间MAP观测，但candidate始终null、stop_candidate为空、没有STOP_CONFIRM；SEARCH约53.400秒后定位跳变/时间回退FAULT。candidate字段仅展示CONFIRMED轨迹，并不代表完全没收到MAP或CANDIDATE。 |

源视频867/867帧完整解码；同名JSONL有867 frame/584 detection/576 control/1 summary。
日志有136份单框、3份双框；这只是检测输出，不能据此否定用户现场只有一个物块的描述。
这3帧是否重复框/误框未做人工真值判定。

## 第一个可确认阻断点与因果边界

`yolo_detector.py:_tick`深度有效后发布独立YOLO检测；
兼容detection_debug的accepted只复制depth.valid，未承诺正式MAP或身份关联通过。
`color_block_target_adapter.py:_process`在调用`_transform_and_publish`之前检查
0～0.3秒年龄、唯一发布者、完整方块帧深度、精确源时间TF及空间身份。
历史版本仅在这些检查全部通过后才input_count+=1。
因此本轮input=6/transformed=6/failed=0证明正式输出只有6次，
而不是其余128份检测都没有TF失败或都成功进入转换器。

最后MAP源时间为1794836958612522949（视频源时间约35.146秒），
Manager最后ID3于1794836959.628以CANDIDATE移除。
首次正式观测距离2.640826m，超1.8m触发上限。
SEARCH `_near_detections`使用源时间/原始MAP点匹配的camera XYZ三维范数，
不能用画面的ROI轴向depth替代。`_search_observations`与`_begin_stop_confirm`
要求同一唯一目标、≤1.8m、有效新鲜度、不同源帧、最大间隔和≤15cm一致性；
CANDIDATE需要3帧，已CONFIRMED的快速路径可以2帧。
所以“Manager未CONFIRMED”本身不是CANDIDATE路径不能停车的充分原因。
旧输出没有trigger_history及逐轨camera_range，无法从六个点总数反推哪些触发条件逐点失败。

134份有效检测在录像节点的接收年龄为0.151～0.227秒，均未超过adapter的0.3秒；
这降低了“检测刚产生就已经过期”的解释，但不能排除adapter排队/等待TF后过期。

**明确结论：第一个可确认阻断环节为adapter发布前检查；具体首个拒绝分支无法由现存日志唯一确定。**
不能将20cm/15cm、定位跳变、TF或RGB-D时差任一项写成已经证实的唯一根因。
本次没有原始完整YOLO XYZ/TF包，不能真实离线重放其选择结果。

## 最小修改

只修复已证实的证据缺口，不在缺少条件证据时修改身份关联/恢复：

1. 现有录像JSONL额外记录完整YOLO、selection、map_measurement_debug和target_tracks，沿用原topic与消息。
2. selection保留观测年龄、initial/current、候选到两锁的距离、各阈值是否拒绝、真实发布计数及TF异常文本。
3. 控制status附带实际trigger_history和逐轨新鲜度/三维距离，区分未匹配距离与距离超限。
4. 离线audit脚本按attempt和独立源帧分别统计，缺失证据返回null而非0；附本轮统计及原日志SHA256。

未修改20cm/15cm条件、目标锁生命周期、模型、RGB-D/TF接受条件、Manager/SEARCH判据、
成功单目标抓取、三轮计数、左右选择或停靠参数。诊断代码不构成锁恢复修复。
若后续证据确认空间锁卡死，才以身份连续证据设计有界恢复；若为TF/时序，则只改对应分支。

## 验证与待办

111项定向离线回归通过，覆盖selector/adapter、SEARCH与快速确认、停靠、左右连续抓取、三轮计数、
新证据保存和audit缺失/重试计数。首次测试因未source Unitree消息overlay有1项导入失败，
补齐已有overlay后全部通过。单包colcon build成功（2分钟），3个修改模块的源码与独立install
SHA256一致。新增工具/测试flake8通过，三个修改模块严重错误检查与py_compile/diff检查通过。
首次多进程flake8挂起，改用--jobs=1通过，并仅清理本轮卡住的lint工作进程。
未运行ROS节点/相机、未连接设备、未执行真实运动或抓取。
已知原始incident根因分支仍Unknown / needs verification；新日志在线完整性与实机效果NOT VERIFIED。

复核命令（仅本地读文件）：

```bash
python3 camera_ws/scripts/audit_yolo_handoff.py /home/czx666/today_full_20260901/today_full/camera_ws/records/full_1794836905838133718
```

统计快照：`camera_ws/docs/evidence/yolo_handoff_20261116/full_1794836905838133718.json`。
