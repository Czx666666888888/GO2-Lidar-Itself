# 第一轮科学目标混合训练

2026-11-14。本轮仅视觉训练，不修改ROS2、Point-LIO、MAP、导航或抓取。
远端起点83f1627fa242d67fbf4929ba5309ee7015c10475，使用现有Python环境与YOLO11n框架。

## 人工标注来源

用户提供下载目录的job_4727595_annotations_2026_11_13_18_18_51_ultralytics yolo detection 1.0.zip。
原件复制到vision_training/annotations/batch_001_annotations.zip，原ZIP未改。
原ZIP含157份非空标签，17张缺标签，0份空标签；缺标签起初不作为负样本。
用户明确确认“是：157张标全正确，17张全部无目标”后，规范化副本为17张添加空标签，
更新原batch_001_review.csv为157 positive、17 negative，使用现有import_cvat.py成功导入174张。
原ZIP和规范化ZIP哈希见annotation_audit.json。未向其余869张未标注图赋予负样本含义。

类顺序0 color_block / 1 rock_compact / 2 rock_spiky；所有坐标、类别、边界、图片哈希和文件名匹配检查通过。
真实框数331/33/94，总458框；83张多目标。标全/类别正确来自用户确认，程序只验证格式；
confirmed_negatives.jpg及本地annotations/batch_001_overlay_preview提供复核图，不能据抽查声称独立逐图语义验收。

## 数据版本与分组

新数据根vision_training/datasets/science_targets_mixed_v1；独立复制图片和标签，不覆盖原始数据。

| split | 合成 | 手机真实 | 合计 |
|---|---:|---:|---:|
| train | 480 | 117（视频A、C） | 597 |
| val | 120 | 57（完整视频B） | 177 |

用户确认三个视频来自不同场景。按完整采集组留出B，而非相邻帧随机拆分；保留原合成场景480/120划分。
全体774张跨split图片哈希和capture_group无交叉。source_type、source_video、源帧、视频SHA256、
capture_group、原标签哈希保存在本地preparation.json及runs/science_targets_mixed_v1/dataset_version.json。
源统计见source_counts.json。真实compact的33框全部位于B验证，训练compact依赖合成图；此局限不通过泄漏验证图来消除。
真实验证仅一个场景，不能外推其他场景或D435；没有独立test，本轮使用现有--pilot入口明确train/val开发实验。
合成标签沿用原实例Mask自动标注，不伪称600张均经人工逐图审核；独立review receipt明确区分来源。

原600张合成数据2404文件在准备前后集合/哈希一致；原视频和抽帧图片保持原样。
仅按本轮授权更新视频池的审核元数据并导入标签，混合训练在复制的数据上运行。

## 训练与复现

配置vision_training/configs/science_targets_mixed_v1.yaml：640、最多100轮、patience30、batch8、nbs64、warmup3。
适度亮度/饱和度、±10度旋转、轻平移/缩放、水平翻转；不做色相偏移、CLAHE或四通道结构。
官方初始化权重SHA256=0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1，
与本轮官方资产下载权重相同，不使用旧blue_cube模型。

```bash
# 已实际执行，输出目录存在时拒绝覆盖；不要直接重复运行。
vision_training/.venv/bin/python -m vision_training.train \
  --config vision_training/configs/science_targets_mixed_v1.yaml --pilot
```

本地输出vision_training/runs/science_targets_mixed_v1/：training.log、dataset_version.json；
baseline/下含weights/best.pt、weights/last.pt、results.csv、args.yaml、数据/环境/初始化快照、
runtime_setup.json、batch_audit.jsonl、runtime_result.json。大数据、标签和权重不入Git。
新增runtime_audit只使用公开回调和optimizer post-step hook观测实际调度，不改优化器或模型。

训练后的评估命令（只读best权重，输出目录拒绝覆盖）：

```bash
vision_training/.venv/bin/python vision_training/tools/evaluate_science_mixed_v1.py \
 --dataset vision_training/datasets/science_targets_mixed_v1 \
 --weights vision_training/runs/science_targets_mixed_v1/baseline/weights/best.pt \
 --output vision_training/runs/science_targets_mixed_v1/evaluation
```

评估按synthetic_val120、phone_val57和phone_train_fit117分开输出；固定阈值诊断为conf0.25/同类IoU≥0.5，
与Ultralytics验证P/R及mAP指标的阈值协议不同。训练集结果只表示拟合，不代表真实泛化。

## 证据边界

真实场景B是本轮留出验证，不是最终test；模型选择使用含B的混合验证指标。
真实D435、独立最终test、连续视频最长漏检时间均NOT VERIFIED。
本轮没有同条件“仅600张合成图”对照实验，不能将任何差异因果归结为加入真实图片。

## 实际GPU训练与评估结果

完成100轮，无提前停止；best为第100轮，用时948.53秒（15.81分钟）。实际7500 batch、971次optimizer step；75 batch/轮，warmup边界225，之后累积8。batch8未OOM，日志显存约1.26GB。

| 来源 | 图片数 | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|---:|
| synthetic_val | 120 | 0.9916 | 0.9978 | 0.9949 | 0.9447 |
| phone_val | 57 | 0.3041 | 0.5359 | 0.3673 | 0.2355 |
| phone_train_fit | 117 | 0.9855 | 0.9898 | 0.9946 | 0.8517 |

### synthetic_val逐类别

| 类别 | Precision | Recall | AP50 | AP50-95 |
|---|---:|---:|---:|---:|
| color_block | 0.9873 | 1.0000 | 0.9949 | 0.9718 |
| rock_compact | 1.0000 | 0.9933 | 0.9950 | 0.9138 |
| rock_spiky | 0.9874 | 1.0000 | 0.9949 | 0.9486 |

固定conf0.25、IoU0.5计数：{'tp': 254, 'fp': 2, 'fn': 0}。

### phone_val逐类别

| 类别 | Precision | Recall | AP50 | AP50-95 |
|---|---:|---:|---:|---:|
| color_block | 0.6996 | 0.8214 | 0.8374 | 0.5755 |
| rock_compact | 0.0820 | 0.3030 | 0.0538 | 0.0208 |
| rock_spiky | 0.1306 | 0.4833 | 0.2108 | 0.1102 |

固定conf0.25、IoU0.5计数：{'tp': 134, 'fp': 50, 'fn': 183}。

### phone_train_fit逐类别

| 类别 | Precision | Recall | AP50 | AP50-95 |
|---|---:|---:|---:|---:|
| color_block | 0.9905 | 0.9796 | 0.9943 | 0.8551 |
| rock_spiky | 0.9805 | 1.0000 | 0.9950 | 0.8483 |

固定conf0.25、IoU0.5计数：{'tp': 138, 'fp': 1, 'fn': 3}。

phone_train_fit没有compact真值，不给该类制造训练拟合指标。混合177张验证最终P/R/AP50/AP50-95为0.9384/0.6596/0.8165/0.6990，不能代替真实57张结果。各来源混淆矩阵PNG与原始矩阵均已保存。

RTX4060 Laptop GPU，FP32 batch1，预热10次后90次：网络平均2.269ms；文件读取/预处理/网络/NMS整体平均4.763ms，P50 4.748ms，P95 5.132ms（约209.9次/秒）。不包含ROS、相机或端到端部署。

## 主要问题和下一批标注优先级

1. 合成验证和A/C训练图指标高，但B明显下降；这是域差异/训练覆盖不足的观测，尚无单因果证明。尤其compact真实训练框为0，不能据合成val认定已学会真实岩石。
2. 真实小目标困难明显：B中compact的33框有25框在640输入下短边<16px；spiky 60框有26框<16px。`video_B_f0002182.jpg`为暗光远目标，固定阈值TP0/FP7/FN7；`video_B_f0002128.jpg` TP0/FP4/FN9。框错位、错类均可同时产生FP和FN，不应将所有FN解释成完全无框。
3. `video_B_f0000700.jpg`多目标小物体TP2/FP3/FN9；可见方块被预测为岩石。`video_B_f0002642.jpg`等灰色compact漏检。spiky近处可检测，远处和多目标场景较差。三类GT/预测拼图同时包含困难与较易图片；compact图中低错误数的单目标例仍可能是漏检，不伪称成功。
4. A/C训练图仍有上缘/左缘目标漏检（A_f0000388、A_f0002206）；训练拟合高不等于边缘已解决。

优先补标：来自新的独立训练场景/会话的真实compact（多角度、与spiky/普通岩石对照、近中远、灰色低对比），其次真实多目标暗光小目标、蓝/绿/红方块与岩石混淆、可辨认模糊及部分出画；再补人工确认的普通岩石/叶片/裂缝负样本。先保证compact有足量真实训练框与类间对照，再增数量。

继续batch_002时，A/C可加入训练候选；同视频B的全部后续图片仍归同一验证组，不能因为换batch就放入train。若决定将B用于训练，必须先另备独立真实验证场景并建立新版本，不能继续引用本轮B结果为独立泛化。最终真实test仍需另采独立会话。

本轮37项测试通过、训练dry-run、100轮GPU训练、三来源实际评估及三类拼图目视抽查完成。首次评估因文件列表相对路径解析失败，已修复为绝对路径后完成；失败日志保留evaluation.log和evaluation_failed_relative_path/，成功日志evaluation_fixed.log。没有改动训练环境或ROS模块。

手机B固定conf0.25、IoU0.5逐类计数：

| 类别 | TP | FP | FN |
|---|---:|---:|---:|
| color_block | 122 | 15 | 102 |
| rock_compact | 0 | 16 | 33 |
| rock_spiky | 12 | 19 | 48 |

因此compact在该工作阈值下没有正确匹配，不应作为可部署三类模型；AP50=0.054来自包含更低置信度预测的完整验证曲线。混合验证高分主要掩盖了真实岩石问题。
