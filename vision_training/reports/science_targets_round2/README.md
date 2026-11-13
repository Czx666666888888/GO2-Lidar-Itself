# 第二轮优先送标与960输入检查

日期：2026-11-14。远端最新Push与本地起点均为0691a5d734a95d1f97655a715925ceb5838b915e。
证据：本地869张逐图读回/尺寸/哈希/质量指标、A/C总览和180张五页拼图目视检查、GPU只读推理。
本轮没有训练、模型导出、正式标签生成或ROS2/机器人操作。

## 剩余图片与实际选择

| 源视频 | 未审核 | JPG读回Laplacian中位数 | 指标<25 | 本轮送标 |
|---|---:|---:|---:|---:|
| A | 370 | 10.35 | 284 | 40 |
| B | 282 | 40.10 | 104 | 0 |
| C | 217 | 16.97 | 146 | 140 |
| 合计 | 869 | — | 534 | 180 |

所有869张保持原尺寸、可读取，字节SHA256匹配原manifest；来源仍为用户确认的phone_real，capture_group与原视频关联保留。
上述Laplacian为重新读回JPG计算，原manifest源解码帧指标也保留，不把JPEG编码造成的差值当数据损坏。
全量图片来源/质量/处置清单位于本地remaining_inventory.csv，全部15页原图预览位于inventory_sheets/。
自动指标无法可靠给出目标类别或数量，故三类分布仍Unknown / needs manual annotation；没有生成伪标签。

C预览包含灰色立体岩石，表观大小与朝向变化明显：0–23秒选50，31–52秒选55，57秒至结束选35。
不能把朝向变化后的同一个岩石自动解释为另一个类别；compact/spiky均需人工按实物外形判断。
A中6–16秒选16、45–49秒选8，保留与方块同框/靠近边缘的灰色斑块待确认；60–73秒选8、105秒至结束选8，补多目标、方块颜色/形状混淆、较暗背景与远小目标。
A灰色斑块可能是地面/背景，不声称已找到compact。首批A已标框只有类0、C只有类2，是既有标签事实，不证明剩余帧一定没有类1。

选图方法：目视定位上述时间窗后，在每窗可用未审核帧中均匀分层选择；不依赖首轮模型预测，不按清晰度一刀切。
180张中119张JPG读回Laplacian<25，仍保留可辨认模糊及低对比困难样本；严重不可辨认时标uncertain。
全图亮度范围约49.70–188.60，没有把“全图均值较低”直接等同真实弱光或目标可见性；预览中有局部暗目标。
覆盖不同表观距离/大小、多目标、边缘、叶片/裂缝背景，不声明量测过真实米制距离或逐图框数。
候选数不是确认岩石数；若人工最终仍找不到compact，需要补采独立真实compact训练场景，本轮不能保证补齐这一缺口。

## CVAT本地交付

仓库绝对根目录：`/home/czx666/today_full_20260901/today_full`。

- ZIP：`vision_training/annotations/science_targets_round2_priority/round2_priority_180.zip`
- manifest：同目录`manifest.json`；Git副本为本报告目录`selection_manifest.json`。
- 审核表：同目录`round2_review.csv`，180张全部pending。
- 来源/清晰度清单：同目录`remaining_inventory.csv`（869张）。
- 全部送标图拼图：同目录`contact_sheets/page_01.jpg`至`page_05.jpg`；本报告目录也提交了五页和代表拼图。
- 全量未审核图拼图：同目录`inventory_sheets/`，B仅作只读库存预览，没有进入ZIP。

ZIP直接包含180张原尺寸JPG，原文件名不变、无目录/标签；CRC、逐图SHA256、图像解码/尺寸、名称与内容唯一检查通过。
ZIP SHA256：`b8908f42033b5ad88dd5bef6cc3dfb7791f985b2cc74639cffb5b742f8f4382c`。
没有与已审核174张重叠；C140/A40/B0。保留source_type/source_video/原视频SHA256/frame_index/source_seconds/capture_group/image_sha256及选取原因。
这是定制第二轮包，不等同原六包中的batch_002；其图片来自原未标注包的子集，不要同时重复标注原包中同名图片。
180张仅为送标候选，未导入训练池、未改reviewed或训练split；原池依然174已审核、869未审核，其中180已整理送标。

CVAT按0 color_block / 1 rock_compact / 2 rock_spiky创建Detection任务，再上传此ZIP。按账户实际限制安排任务，不能假设账户上限。
逐物体画全部可辨认正式目标，边缘画可见部分，普通岩石/灰色地面不标科学目标；六色方块仍同一类。
review.csv的positive只用于已标全且类别确认的图，negative只用于人工确认无目标，难辨认为uncertain，其余pending。
可只导出Ultralytics YOLO Detection annotations；保持文件名，随后复用现有import_cvat.py与本审核表匹配原图。
缺少标签文件不等于空标签或无目标，必须先人工确认。此次不自动导入和训练。

完整B（包括原已审核57张和其余282张）继续保留为当前独立验证场景；所有后续B图片均不得混入train。
A/C新标注仅增加原训练场景覆盖，不能据此扩大独立场景数或宣称解决泛化。

复现准备（已有输出时拒绝覆盖）：

```bash
vision_training/.venv/bin/python vision_training/tools/prepare_priority_round2.py \
 --dataset vision_training/datasets/science_targets_video_v1 \
 --config vision_training/configs/science_targets_round2.yaml \
 --output vision_training/annotations/science_targets_round2_priority
```

## imgsz=960兼容性与显存

源码确认：yolo_detector.py的imgsz要求正数且为32倍数，960满足；predict_args透传到Ultralytics。没有修改ROS2文件、配置或启动节点。
现有YOLO11n .pt通过640和960实测推理，仍使用首轮best.pt。Ultralytics 8.3.40的pre_transform对同尺寸PT图片自动矩形padding，不能简单把imgsz等同方形张量。
export.py支持传入960，但当前ONNX导出dynamic=False；已有固定640 ONNX不可直接当960模型使用，需要将来单独导出960并匹配metadata.imgsz（本轮未导出）。
节点会检查元数据尺寸一致性；即使是PT，携带640元数据时也不能只改参数绕过校验。

RTX4060 Laptop 8GB、FP32、batch1，每项独立进程、预热5次+20次重复，限定CPU16–19与OMP/OPENBLAS线程为1：

| 场景 | 实际张量 | PyTorch峰值allocated | 峰值reserved |
|---|---|---:|---:|
| 640真实C图 | 1×3×384×640 | 55.99 MiB | 70 MiB |
| 960同一C图 | 1×3×544×960 | 71.88 MiB | 90 MiB |
| 960方形warmup | 1×3×960×960 | 94.82 MiB | 120 MiB |

实测均无OOM。以上仅PyTorch allocator统计，**不是整进程/整节点显存需求**，不包含CUDA驱动/上下文、ROS图像缓存及其他进程。
方形输入像素是640方形的2.25倍，但显存包含权重、缓存和算子工作区，不能机械按2.25倍保证训练显存。
本轮仅推理，960训练batch8显存、ROS2实际流、NX部署以及检测精度变化均NOT VERIFIED；没有为了测显存做反向传播或训练。
本轮CPU线程限制与上一轮计时条件不同，JSON中的推理时间只作运行证据，不能据此作640/960精度/速度或跨轮性能结论。
首次640探测进程139异常退出，底层原因未确认；限制线程后通过。首次960脚本错误假定方形张量而断言失败，已修正为记录实际padding形状并重新通过，未修改库或安装环境。

## 保留与验证

首轮best/last权重SHA256与原training_result一致，训练results.csv与首轮Git报告匹配，原评估metrics.json与Git报告匹配；见preservation.json。
未覆盖首轮数据/权重/评估，没有新标签、审核回执或训练。原已存在的机器人/导航/decisions未提交改动不纳入本轮。
2项针对性测试通过（B排除、已审核排除、原池不改、ZIP对应、待审核状态及时间分层）；实际869图核验、180图ZIP完整性与三项GPU推理检查通过。
