# 第三轮人工标注导入、YOLO11n V3与真实开发验证

2026-11-15。开始fetch后远端/本地均为9c16c09634dec5362f888f66635f47365cd6a34f。
本轮仅离线视觉数据、训练和评估；现有YOLO V2→Depth→MAP→Approach程序、Point-LIO、Manager、Approach和机械臂流程未修改或启动。

## 人工标注导入

两份原始ZIP从微信本地文件目录找到并按字节保存于annotations/science_targets_round3/，原件不改。
Batch001 job_4728822：153图，127 positive / 26 negative，246框，39张多目标。
Batch002 task_2764079：152图，132 positive / 20 negative，259框，40张多目标。
合计305图，259 positive / 46 negative，505框：color_block365 / rock_compact0 / rock_spiky140。

46张负样本来自本会话用户明确人工确认；不是程序根据缺TXT或模型无输出推断。
独立confirmed_negatives ZIP仅为这46张补空TXT，复用import_cvat.py按data.yaml类别名称映射；
两份原始导出均为0 color_block / 1 rock_compact / 2 rock_spiky，实际映射0→0、1→1、2→2。
原ZIP SHA256、规范化副本SHA256、审核CSV、框数见annotation_audit.json及batch_00*_review.csv。
规范化副本保留源类顺序，导入池中的标签由原导入器按名称映射为正式类别；原始标签bytes也保留。

检查ZIP CRC/唯一性/路径、原送标批次文件名集合、所有标签坐标、图片尺寸/hash、跨池源视频+帧号/hash；
79张多目标图均保留多框。已抽查叠框图中的运动模糊、小目标及多目标对应关系，未声称替代人工语义审核。
source_video、video SHA256、frame_index、capture_group及历史RGB时间戳保留于dataset_version.json。

## V3数据版本

独立目录datasets/science_targets_mixed_v3/。原600张BlenderProc沿用480/120，V2全部935图原样沿用，再加第三轮305图到train。

| split | 合成 | 真实 | 总图片 | color_block框 | rock_compact框 | rock_spiky框 |
|---|---:|---:|---:|---:|---:|---:|
| train | 480 | 583 | 1063 | 830 | 292 | 652 |
| val | 120 | 57 | 177 | 314 | 109 | 148 |

真实train为522 positive / 61 confirmed negative，框数528/0/296；真实val55 positive / 2 negative，框数224/33/60。
历史125张GO2图全部归lab_floor_20261108_09；所有采集组整体划分，没有相邻帧随机拆分。
原视频B的同57张图及标签与V1/V2逐hash一致，B是开发验证且用于模型选择，不能称为未触碰最终test。
新视频3的50张与新视频4的86张仍为独立验证/最终测试候选，没有进入train、训练val或模型选图推理。
这些留出图片无合法人工标签，本轮不能计算新场景独立P/R/AP、远距/spiky/普通岩石背景误检率，均NOT VERIFIED。
新视频1/2的训练图预测不能代替它们。25个新增方块小目标、11个spiky小目标，定义为长边缩640后框短边<16px；详见round3_gt_sizes.json。

第二轮19张缺标签继续pending，不因本轮负样本授权而补空。
历史20张blue_cube人工标签已核查，未包含于V2，也与本轮305张无源帧重复；因仅单类审核、缺其它类别完整性审核，本次不追加，不重复计数，原标签保留。
原V1/V2、合成源集及旧20图/权重hash保护见preservation.json；保留全部旧权重和本地训练记录。

## 训练

YOLO11n，V2 best.pt初始化（pretrained.json核对SHA256），原3类保留；不改网络，不加CLAHE。
RTX4060、Ultralytics8.3.40、torch2.5.1+cu124，640 / batch8 / nbs64 / warmup3 / epochs上限100 / patience30 / seed42。
1063训练图每轮133 batch，预热边界399 batch，预热后目标梯度累积8；实际step记录见runtime_result.json与本地batch_audit.jsonl。
使用原--pilot入口明确仅train/val开发协议；名称沿用框架，不表示305图是未审核pilot数据，也不冒充有独立test。
线程1、CPU affinity16–19；未升级训练环境。

实际完成100轮，用时43.35分钟；13300 batch、1729 optimizer steps。无OOM。best轮次及权重hash见best_epoch.json/run_summary.json。

## 同一真实视频B对比（640）

P/R为Ultralytics完整曲线选定工作点；AP使用完整置信度范围。
固定阈值TP/FP/FN另按conf0.25、同类别IoU≥0.5一对一匹配，不能与曲线P/R混成同一工作点。
模型/图像/标签hash与全部57个文件名已核对。comparison.json保留完整指标，evaluation_metrics_640.json保留V3逐图GT、预测坐标、confidence、匹配与错误。

| 版本 | 类别 | P | R | AP50 | AP50-95 | TP / FP / FN (conf.25) |
|---|---|---:|---:|---:|---:|---|
| V1 | color_block | 0.6996 | 0.8214 | 0.8374 | 0.5755 | 122 / 15 / 102 |
| V1 | rock_compact | 0.0820 | 0.3030 | 0.0538 | 0.0208 | 0 / 16 / 33 |
| V1 | rock_spiky | 0.1306 | 0.4833 | 0.2108 | 0.1102 | 12 / 19 / 48 |
| V2 | color_block | 0.8666 | 0.6958 | 0.8178 | 0.5785 | 139 / 21 / 85 |
| V2 | rock_compact | 0.0000 | 0.0000 | 0.0189 | 0.0037 | 0 / 17 / 33 |
| V2 | rock_spiky | 0.2810 | 0.3333 | 0.2318 | 0.1176 | 14 / 27 / 46 |
| V3 | color_block | 0.7646 | 0.8702 | 0.8736 | 0.6072 | 187 / 52 / 37 |
| V3 | rock_compact | 0.3506 | 0.0606 | 0.1200 | 0.0449 | 1 / 1 / 32 |
| V3 | rock_spiky | 0.8231 | 0.3000 | 0.4664 | 0.2765 | 11 / 2 / 49 |

小目标为同一GT子集（长边缩640后短边<16px），多目标为GT数>1，以下均conf.25：

| 版本 | 方块小目标TP/FN | compact小目标TP/FN | spiky小目标TP/FN | 多目标图 TP/FP/FN |
|---|---|---|---|---|
| V1 | 12/60 | 0/25 | 4/22 | 131/50/175 (44图) |
| V2 | 28/44 | 0/25 | 6/20 | 148/65/158 (44图) |
| V3 | 46/26 | 0/25 | 1/25 | 195/54/111 (44图) |

## 历史GO2运动录像与人工审核包

原始10段raw共3117帧，三版本均逐帧处理，保存所有conf≥.10框、confidence、原始rgb_stamp_ns、帧号与控制日志，presence阈值.25。
使用原始RGB时间戳核对连续/严格递增，容器15fps不冒充实际帧时间。
go2_prediction_comparison.json和r3_go2_07/08_confidence_timeline.png描述预测输出与confidence变化；
class0预测不等于已正确识别蓝方块；无class0预测可能是不可见/出画/错类/低置信度，不能直接当真实漏检。
空档时长用最后无预测帧减首个无预测帧的RGB stamp，一帧空档观测跨度为0；下一次输出延迟不是GT证实恢复速度。
全图Laplacian和日志速度只作诊断，不证明单一模糊原因。

按用户本轮选择，已建立90张连续帧人工审核包，稍后人工补标：
`vision_training/annotations/go2_continuity_v3_review/go2_continuity_90_images.zip`。
包含r3_go2_07帧100–144及r3_go2_08帧150–194，原RGB/逐图hash/源视频hash/原始时间戳/review.csv/三页预览均保留。
5帧为V3训练精确帧，全部属于同lab训练采集组，manifest明确标注；它只能作同场景连续性诊断，不能作独立泛化验收。
需人工确认每帧蓝方块身份、可辨认性、全部目标框和negative/uncertain，再计算IoU匹配的逐帧成功、最长连续漏检和恢复时间。
本轮没有完整连续人工GT，因此真实最长漏检帧数/时间、模糊持续漏检及恢复速度均NOT VERIFIED。
真正泛化需另采不进入训练的独立GO2连续片段；不从训练帧或模型输出伪造GT。

## 交付与验证边界

- best.pt：`vision_training/runs/science_targets_mixed_v3/baseline/weights/best.pt`，last.pt同目录。
- 全训练日志/依赖和数据快照/逐batch审计/权重/三版本全视频JSONL/所有原尺寸GT预测对照均保留在本地runs/science_targets_mixed_v3。
- 原ZIP和规范化ZIP在annotations/science_targets_round3，原数据与权重不入Git；Git仅代码/config/审核来源与评估摘要、曲线及代表图。
- 54项视觉工作流测试、py_compile、训练dry-run、GPU训练和离线评估；flake8未安装，未执行该工具。没有ROS代码改动，未做ROS/全仓build。
- 不切换现有V2部署权重，不启动相机/ROS/GO2运动。新权重部署、RGB/Depth匹配、相机frame/MAP输出与实际连续识别均NOT VERIFIED。

复现（输出已存在会拒绝覆盖，应使用新目录/新版本）：

```bash
vision_training/.venv/bin/python vision_training/tools/prepare_science_mixed_v3.py \
 --archives /Batch001原ZIP /Batch002原ZIP --confirm-user-reviewed-259-positive-46-negative
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 16-19 \
 vision_training/.venv/bin/python -m vision_training.train \
 --config vision_training/configs/science_targets_mixed_v3.yaml --pilot
bash vision_training/tools/evaluate_science_v3.sh
vision_training/.venv/bin/python vision_training/tools/summarize_go2_v3.py
```


## 960输入对照与是否替换V2

960是同一640训练权重推理，不是960训练。B的方块P/R/AP50/AP50-95为.6416/.9509/.9068/.6247，spiky为.3333/.3833/.3714/.1924，compact为.1786/.1515/.1049/.0266。
conf.25方块TP/FP/FN=203/59/21、compact=0/4/33、spiky=16/5/44；方块小目标TP/FN=59/13，spiky=2/24；44张多目标图TP/FP/FN=215/65/91。
V3 640同阈值方块TP139→187、FN85→37，但FP21→52；spiky FP27→2，同时TP14→11。
640的方块曲线P/R=.7646/.8702，未同时≥.90；spiky R=.30未达.80。compact真实训练仍0框，B中1个TP不证明可用。

**本轮不建议直接用V3替换V2。** V3更有利于方块召回和小目标覆盖，但方块误检增多、spiky召回不足，且独立GO2连续性真值未完成。
后续960训练值得作为单独、固定数据和初始化的受控对照：960推理带来方块小目标46→59 TP、AP50 .8736→.9068；但precision下降、spiky AP下降，不能保证960训练解决。
本轮只完成640训练和960推理对照，不追加调参训练；先补连续GT/留出验证、真实compact与困难背景，再决定高分辨率实验和部署。
切换前仍需类别映射、推理输入尺寸、RGB-Depth配对、相机frame和MAP输出及实际GO2连续识别检查；现有V2适配流程保持。

历史全3117帧中有conf≥.25 class0输出的帧：V1 651、V2 636、V3 1799。这不是检出率，可能含FP，也包含训练来源。
重点两段最大**预测输出空档**（帧数/原RGB时间戳观测跨度秒）：

| 视频 | V1 | V2 | V3 |
|---|---|---|---|
| r3_go2_07 | 327 / 21.898 | 216 / 14.448 | 102 / 6.778 |
| r3_go2_08 | 141 / 9.408 | 459 / 30.779 | 121 / 8.065 |

以上可能包含目标不在画面/不可辨认/低置信度时段，不能据此宣称连续漏检缩短；等待人工逐帧真值后再作判断。

性能计时仅为本轮离线运行记录，部分评估与其它GPU离线任务重叠；不作为独占GPU延迟或GO2实时吞吐验收。
