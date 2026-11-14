# 第二轮科学目标混合训练与真实验证

2026-11-14。远端起点和本地起点均为 `454d5440f1c0633cbbbdebee263d2c4f688eee52`。
仅离线视觉训练/评估；不操作机器人，不修改 HSV、Point-LIO、MAP、导航、抓取或 ROS 接口。

## 标注来源和审核边界

原导出从本地下载目录 `job_4728349_annotations_2026_11_14_11_20_48_ultralytics yolo detection 1.0.zip`
按字节复制至 `vision_training/annotations/science_targets_round2_priority/round2_annotations.zip`。
ZIP 的 train.txt 包含180个图片条目，161份非空TXT，19张缺TXT，178框；导出本身不含图像，
图像来自原选图manifest关联的原尺寸抽帧文件，已逐张核对SHA256。

原CVAT顺序0 color_block / 1 rock_spiky / 2 rock_compact。
复用 `import_cvat.py` 根据名称实际重写标签：0→0、1→2、2→1；坐标不变。
原ZIP保留；独立规范化ZIP为同目录 `round2_reviewed_normalized_v2.zip`，规范化标签实际为0/2，
未为19张缺标签图补空TXT；规范化train.txt只列161张已审核正样本，其余19张仅列pending.txt。原始和规范化SHA256见 `annotation_audit.json`。

本会话用户明确确认“是，161张已全部审核通过”，针对“所有可辨认目标均已标全，类别正确”。
人工完整性/类别正确性来自此确认，程序只验证配对、格式、坐标、哈希及映射，不能替代逐图人工语义审核。
第二轮有效正样本161、确认负样本0、pending19；新增color_block56框、rock_compact0框、rock_spiky122框。

`round2_review.csv` 是本轮审核记录；`pending_19.csv` 与 `pending_19.jpg` 提供19张缺TXT的完整清单/缩略图。
19张均保持pending，不纳入V2，CSV框数留空表示未知。缩略图外观不能替代“无目标”确认；以后只有人工确认后才能在新版本中补空标签。
独立导入池为 `vision_training/datasets/science_targets_round2_reviewed_v2/`，不更改第一轮或原视频池的标签/审核状态。
后续选图必须同时排除本独立池的161张reviewed图片，不能仅按原视频池pending状态重新送标。

## V2数据版本

独立根 `vision_training/datasets/science_targets_mixed_v2/`。
原600张Blender图全部沿用原480/120分组；第一轮174张全部沿用；第二轮161张仅加入A/C训练。

| split | 合成图片 | 真实图片 | 总数 | color_block框 | rock_compact框 | rock_spiky框 |
|---|---:|---:|---:|---:|---:|---:|
| train | 480 | 278 | 758 | 465 | 292 | 512 |
| val | 120 | 57 | 177 | 314 | 109 | 148 |

真实train为263 positive/15 confirmed negative，真实val为55 positive/2 confirmed negative。
真实train框数163/0/156，真实val框数224/33/60。
训练中compact292框全部来自合成图；真实compact依然0个训练框，不声称compact问题解决。
原数据和来源仍为phone_real；相邻帧增加不等于独立训练场景增加。

视频B所有图片继续保留验证用途，本轮没有添加新B标签或将B混入训练。
实际用于V1/V2对比的仍为第一轮同57张B图片；“完整B留出”不等于B339张均有标签可评估。
V2 val图像集合及标签与V1逐个哈希相同，跨split capture_group和图片哈希无交叉。
数据 `preparation.json` 保存来源、视频hash、帧号、capture_group、标签hash与拆分依据；Git报告副本为 `dataset_version.json`。
`preservation.json` 保存准备前后的原合成集、原视频池、V1混合集/权重/评估的文件清单及SHA256保护核验。

## 训练方案和验证协议

使用既有YOLO11n/Ultralytics8.3.40、RTX4060 Laptop 8188MiB、V1 best.pt初始化；不重装环境。
640输入、batch8、nbs64、warmup3、100轮上限、patience30、seed42；增强沿用V1。
每轮95 batch，warmup边界285 batch，warmup后目标累积8；实际调度由runtime_audit记录。
仅train/val开发实验，使用现有`--pilot`入口；没有独立最终test。
训练配置 `vision_training/configs/science_targets_mixed_v2.yaml`；本地输出 `vision_training/runs/science_targets_mixed_v2/`。
CPU affinity16–19，OMP/OPENBLAS/MKL线程1，避免先前探测的无约束线程异常。

```bash
# 输出已存在时拒绝覆盖，不直接重复运行。
vision_training/.venv/bin/python vision_training/tools/prepare_science_mixed_v2.py \
 --archive vision_training/annotations/science_targets_round2_priority/round2_annotations.zip \
 --confirm-161-complete-positives
vision_training/.venv/bin/python -m vision_training.train \
 --config vision_training/configs/science_targets_mixed_v2.yaml --pilot
```

P/R/AP50来自Ultralytics完整验证曲线；P/R的曲线选定工作点与conf0.25计数协议不同。
固定阈值统计为conf0.25、同类别IoU≥0.5、一对一匹配，分别报告TP/FP/FN。
单框confidence≥0.90不是90%检测准确率。
小目标定义为原图长边缩至640后框短边<16px，跨分辨率对比固定同一GT子集。
多目标定义为GT目标数>1的图片，报告其总TP/FP/FN。
B是训练模型选择使用的开发验证场景，不能称为独立未触碰最终test或D435验收。

## 部署边界

本轮首先训练和验证，不切换HSV。三类检测、对应深度/三维点、时间戳和相机frame仍需一致关联，
不能把无类别PointStamped混流当作多目标关联完成。
D435实时RGB-D、三维粗定位、独立真实test与连续视频最长漏检时间均 NOT VERIFIED。

## 实际训练和同一视频B对比

完成100轮，best为第100轮，用时1958.46秒（32.64分钟），9500 batch、1233 optimizer step。日志显存约1.26–1.27GB，无OOM。

P/R为Ultralytics曲线选定工作点，不能与下表固定conf0.25计数混为同一阈值。

| 版本/输入 | 类别 | Precision | Recall | AP50 |
|---|---|---:|---:|---:|
| V1_640 | color_block | 0.6996 | 0.8214 | 0.8374 |
| V1_640 | rock_compact | 0.0820 | 0.3030 | 0.0538 |
| V1_640 | rock_spiky | 0.1306 | 0.4833 | 0.2108 |
| V2_640 | color_block | 0.8666 | 0.6958 | 0.8178 |
| V2_640 | rock_compact | 0.0000 | 0.0000 | 0.0189 |
| V2_640 | rock_spiky | 0.2810 | 0.3333 | 0.2318 |
| V2_960_inference | color_block | 0.6964 | 0.8705 | 0.8294 |
| V2_960_inference | rock_compact | 0.1613 | 0.4545 | 0.1215 |
| V2_960_inference | rock_spiky | 0.1622 | 0.5000 | 0.3039 |

固定conf=0.25、同类IoU≥0.5、一对一匹配：

| 版本/输入 | 类别 | TP | FP | FN |
|---|---|---:|---:|---:|
| V1_640 | color_block | 122 | 15 | 102 |
| V1_640 | rock_compact | 0 | 16 | 33 |
| V1_640 | rock_spiky | 12 | 19 | 48 |
| V2_640 | color_block | 139 | 21 | 85 |
| V2_640 | rock_compact | 0 | 17 | 33 |
| V2_640 | rock_spiky | 14 | 27 | 46 |
| V2_960_inference | color_block | 162 | 39 | 62 |
| V2_960_inference | rock_compact | 0 | 16 | 33 |
| V2_960_inference | rock_spiky | 20 | 27 | 40 |

## 小目标和多目标

小目标固定为原图长边缩至640后短边<16px，960仍统计同一GT子集。下表为conf0.25的TP/FN。

| 版本/输入 | color_block | rock_compact | rock_spiky | 44张多目标图 TP/FP/FN |
|---|---:|---:|---:|---:|
| V1_640 | 12/60 | 0/25 | 4/22 | 131/50/175 |
| V2_640 | 28/44 | 0/25 | 6/20 | 148/65/158 |
| V2_960_inference | 37/35 | 0/25 | 8/18 | 178/81/128 |

真实训练新122个spiky框只有1个小目标，而B的60个spiky框中26个小目标；具体尺寸统计见real_box_sizes.json。此为覆盖差异证据，不是已证明的单一漏检原因。

640的spiky固定阈值TP12→14，FP19→27；960增至TP20但FP仍27，未达到Recall≥0.80或减少误检的目标。compact在640/960的conf0.25均为0TP，960曲线的Recall0.4545/AP50 0.1215来自更低置信度范围，不能据此宣称可用。

方块640曲线Precision提高但Recall/AP50降低；960的Recall回升至0.8705，Precision仅0.6964，两者均未满足P/R同时≥0.90。

44张多目标图在640的TP131→148、FN175→158，但FP50→65；960 TP178、FP81、FN128，仍有错类/漏框。

960只是同一640训练权重的输入对照，未训练960模型；整体AP50从0.3562升至0.4182，但conf0.25总FP65→82。鉴于目标仍未达标、真实compact为0且小目标覆盖不足，本轮不继续追加960训练或盲目调参；先补充独立真实训练场景。


## 真实岩石与截图核对

- 已目视抽查640两类岩石GT/预测拼图：video_B_f0000240、f0000096的spiky可正确匹配；video_B_f0002642、f0002594、f0001487的compact漏检。不是全部岩石逐图人工验收。

- video_B_f0002236：640固定阈值TP0/FP5/FN8，远处多目标存在错类/错位；不把每个FN都解释为完全没有预测框。

- 三类640/960典型难例与较易例见本目录*_examples.jpg；640_case_*.jpg保留原尺寸对照，左GT、右预测。

- 全57张原尺寸GT/预测对照保存在本地runs/science_targets_mixed_v2/evaluation_640/phone_val_cases及evaluation_960/phone_val_cases；逐图GT、预测框、confidence、匹配GT、错类/漏框和尺寸保存在evaluation_metrics_640/960.json。


## 合成验证、训练拟合和证据限制

| 640来源 | 图片 | Precision | Recall | AP50 |
|---|---:|---:|---:|---:|
| synthetic_val | 120 | 0.9936 | 0.9956 | 0.9927 |
| phone_val | 57 | 0.3825 | 0.3430 | 0.3562 |
| phone_train_fit | 278 | 0.9992 | 0.9908 | 0.9949 |

合成和A/C拟合高分不能覆盖B的失效；训练拟合没有compact真值，不能把该类0/0计数当识别成功。

全部49项视觉工作流测试、3个脚本py_compile、训练dry-run、100轮GPU训练、640三来源评估、960同B推理对照通过；未做ROS/整仓build，未改ROS代码。6523个原文件的集合与SHA256再次核对一致。

首次未约束CPU的比较进程在PIL/enum导入报ValueError；按既有线程1/CPU16–19约束重跑成功，底层原因未知，没有重装或修改环境。

建议保留当前HSV，不切换V2。D435实时RGB-D、类别/框/深度三维点/时间戳/frame关联、导航和抓取均NOT VERIFIED。后续优先补真实compact、近远/低对比小目标和多目标新训练场景；完整B继续留验证，最终test另采。


## 交付位置

- 本地best.pt：`vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt`。

- best.pt SHA256：`e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1`；初始化权重hash见pretrained.json，与原V1 best一致。

- training.log、batch_audit.jsonl、环境/配置/数据快照、完整评估日志和全部截图在本地runs/science_targets_mixed_v2；Git保存配置、审核表、来源/hash、指标、曲线、混淆矩阵和代表截图，不上传图片数据集/ZIP/权重。

- comparison.json核对三组完全相同的57个B文件名；V1数据/hash、指标来自已保存的原评估，V1小目标/多目标计数由原missed_gt和未变GT重算，没有重写V1评估。


复现评估（原输出已存在，换新的output目录）：

```bash

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 taskset -c 16-19 \
 vision_training/.venv/bin/python vision_training/tools/evaluate_science_mixed_v1.py \
 --dataset vision_training/datasets/science_targets_mixed_v2 \
 --weights vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt \
 --output /新评估目录 --imgsz 640

# 960对照将imgsz设为960，增加 --groups phone_val，并另选output目录。
```
