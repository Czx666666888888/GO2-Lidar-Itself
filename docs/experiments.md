# Experiment Record

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

## 2026-11-10 — blue_cube_pilot_v1（PC真实GPU，小样本）

- 来源：用户CVAT First 30.zip；20图20框。按4个视频14 train/6 val，无test；同lab场景。
- 原annotation_v1 181图哈希未变，161张未标注图未当负样本。
- 既有RTX4060/venv，YOLO11n 640/batch4/epochs50/patience15/device0；实际20轮早停。
- 独立val P=0.003333、R=1、mAP50=0.00996661、mAP50-95=0.00712589。
- conf0.25：6张val与12张未标注观察图均0框，模型不可用，无成功案例；后者不算定量测试。
- 90次预热后GPU推理均值6.385ms、同步predict墙钟8.969ms；不代表持续FPS/ROS。
- 本机证据vision_training/runs/blue_cube_pilot_v1，报告及参数/版本/权重哈希见
  [pilot报告](../vision_training/reports/blue_cube_pilot_v1.md)。18项针对性检查通过。
- 未全仓构建、未重装框架、未启动ROS/GO2；真实部署和跨场景性能NOT VERIFIED。

## 2026-11-10 — 已有pilot权重失败分析（无新训练）

- 基线b7a2e46，20图真实GPU四阈值、类别及IoU≥0.5的一对一真值匹配。
- best conf0.01：train TP/FP/FN=3/973/11，val=1/427/5；conf0.05/0.10/0.25均0框。
- 最高候选分数约0.0128–0.0132，多为背景；不能用候选框总数当正确识别。
- 20图GT裁剪复核无明显错对象；验证目标更小/更靠上；同视频高度近似，有效多样性不足。
- 训练cls损失5→20轮下降52.4%，但best未可靠拟合train；last高分输出全为背景FP。
- 固定源码默认预热100batch，本次仅80batch；推算17个optimizer_step调用点，非实测有效更新数。
- 主要支持A，建议先补多样标签、从官方权重建新基线并适配更新预算；唯一根因未证实。
- 报告：[失败分析](../vision_training/reports/blue_cube_pilot_v1_failure_analysis.md)。
  本机完整证据vision_training/runs/blue_cube_pilot_v1/analysis_v2；21项检查通过。
- 未训练、未改框架/ROS/HSV/MAP/控制、未全仓编译或启动GO2；改进效果NOT VERIFIED。

## 2026-11-10 — blue_cube_pilot_v2（一次80轮配置修正）

- 基线3ea9fbc；同20图14/6、同官方YOLO11n与增强；nbs64→4，epochs50→80，patience15→80。
- warmup3仍有100batch下限；实际320batch累积均1，optimizer post-step hook313次，AMP跳步7次。
- conf0.25、IoU≥0.5：V1 best train/val=0/0/14、0/0/6；V2 best=13/0/1、4/0/2；last=14/0/0、4/0/2。
- conf0.05：V2 best train/val=14/1/0、5/0/1；last=14/1/0、6/0/0，FP1为重复框。
- best第70轮，last第80轮；训练调用87.85秒，CSV累计74.69秒（口径含监测等，非纯GPU时间）。
- 第24–27轮10项验证loss为NaN，后期有限；原CSV保留，分析显式null并记事件，不声明全程稳定。
- 结论：已具备训练图拟合能力；同场景val仍漏边缘小目标，不证明跨场景泛化。
- 236个V1/数据保护文件和库trainer源码hash未变；30项检查通过，无全仓构建/GO2运行。
- [对比报告](../vision_training/reports/blue_cube_pilot_v2.md)；本机完整证据vision_training/runs/blue_cube_pilot_v2。


## 2026-11-12：BlenderProc科学目标合成数据（非真机）

- 来源：实际Blender4.2.1/BlenderProc2.8.0 CPU渲染、原STL/发布ZIP核对、mask与原STL世界坐标独立检查。
- 50mm立方体/0.5mm倒角实际创建；五个已知材质+第六色未确认占位，后者不采样。
- 本地原STL默认scale1.0，单位和实物尺寸未知；原始文件未改，哈希/拓扑/导入尺寸见合成assets审计。
- 600张：每类主目标200；实际框数[392, 368, 444]；300场景，每场景2视角，train480/val120，无test。
- 全600张自动标签/掩码/来源/场景泄漏检查通过；原STL接地812次通过。
  小批量发现并修复坡面悬空及间距问题，历史悬空样本回归拒绝；最终预览抽查通过。
- 数据位置：`vision_training/datasets/science_targets_synthetic_v1/`；原始日志在合成runtime/batch600.log。
  统计/哈希：`vision_training/synthetic_generation/previews/batch600_quality.json`；详见该目录REPORT。
- 证据等级：合成渲染与离线数据检查，不是真实D435或GO2运行。synthetic_camera，实物尺寸、
  第六色、真实CameraInfo/Depth/GPU/泛化为NOT VERIFIED。最终test必须独立真实D435；未训练/未改ROS或运动。


## 2026-11-12：合成生成器采样审查（0张新渲染）

- 源码审查：移除目标尺寸触发的隐式距离放大及失败换段；新增三段距离、七区域、稳定岩石姿态。
- 实际检查：11项离线测试通过；Blender原STL多姿态支撑/投影检查，不调用渲染器。
  compact 16组合中14通过、2因坡面支撑不稳定被过滤；spiky 16通过，均非实物姿态验收。
- 不覆盖与固定far失败路径实际执行：3次仍在far内，无降门槛、无COMPLETE、0次渲染调用。
- 新版检查器只读回归原600张通过，输出写临时视图；原812次STL接地核对通过。
- 2455个既有数据/正式池/模型资产文件的hash及集合不变；既有600张不变。
- 证据：vision_training/synthetic_generation/GENERATOR_REVIEW.md与reports/sampling_*.json。
- NOT VERIFIED：新采样RGB/Mask实际效果/全分布覆盖、实物尺寸/重心、D435实际CameraInfo。
  未导正式池、未训练、未操作机器人；最小预览命令仅提供，未执行。


## 2026-11-14：第二轮CVAT映射与科学目标V2（离线GPU）

- 远端/本地起点454d544。原CVAT类顺序block/spiky/compact；按名称实际重写标签为block/compact/spiky，原ZIP SHA256 c3e1be9f4f4cc0321ead37b700cdd12be5ad91506998b9898c11e3b2b4e61c8a。
- 用户明确确认161张已标全/类别正确：新增框56/0/122；19张缺TXT继续pending，负样本0。规范化train.txt仅161张，pending.txt单列19张，不补空标签；隔离导入池与审核表保留。
- 数据V2 758train/177val，真实train278（263positive/15negative），仍同57张B；原600图480/120划分不变，原V1和源池不动，6523文件核对未变。
- YOLO11n以V1 best微调640，batch8/nbs64/warmup3，100轮1958.46秒，9500batch/1233 optimizer step，best第100轮；显存日志约1.26–1.27GB，无OOM。
- 真实B的640逐类P/R/AP50：block .8666/.6958/.8178；compact 0/0/.0189；spiky .2810/.3333/.2318。conf.25 TP/FP/FN=139/21/85、0/17/33、14/27/46。
- 960为同一640权重推理对照，非960训练：逐类P/R/AP50=.6964/.8705/.8294、.1613/.4545/.1215、.1622/.5000/.3039；conf.25 compact仍0TP，总TP/FP/FN182/82/135。
- 小目标spiky TP/FN：V1 4/22、V2 640 6/20、V2 960 8/18；44张多目标图分别TP/FP/FN131/50/175、148/65/158、178/81/128。小目标固定为缩至640后短边<16px。
- 新122个spiky仅1个小目标，B60个中26个小目标；真实compact训练依然0框。目标未达标，保留HSV，优先补真实训练场景，不追加盲目调参/960训练。
- 49项离线检查、静态编译/dry-run、GPU训练/评估通过；未整仓ROS build、未运动。首次未约束CPU的比较进程PIL/enum导入报错，限定线程/亲和性后重跑成功，原因未知，未改环境。
- 报告与Git证据：vision_training/reports/science_targets_mixed_v2/；完整本地训练/逐图对照：vision_training/runs/science_targets_mixed_v2/；best SHA256 e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1。
- 证据等级：文件/标签检查、用户审核确认、离线GPU训练与B开发验证；B非最终test。D435实时RGB-D、类别/框/三维点/时间戳/frame关联、导航和抓取均NOT VERIFIED；未替换HSV。
