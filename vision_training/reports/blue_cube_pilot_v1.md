# blue_cube_pilot_v1：首次20图试训练

2026-11-10，来源：本机真实CUDA训练/评估输出及逐图视觉核查。训练和证据生成基于
`feature/reusable-yolo-training`远程基线`ac76fe9795ed7a4ddd8124f157538b3a94fb1d57`。

结论：训练链已执行，但本轮模型检测效果差，不具备使用条件。没有可报告的成功检测案例。
在`conf=0.25`下6张验证图、12张未标注观察图均输出0框；后者无真值，不能据此算准确率。

| 项目 | 实际值 |
|---|---|
| 人工标注图/框 | 20 / 20（blue_cube，ID 0） |
| train / val / test | 14 / 6 / 无 |
| 模型 | 官方本地yolo11n.pt微调，RTX4060 Laptop GPU |
| imgsz / batch / device | 640 / 4 / 0 |
| epochs / patience | 上限50 / 15；实际20轮早停，best来自第5轮 |
| 耗时 | results.csv累计16.2077秒；训练器日志显示0.005小时（舍入） |
| 增强 | hsv_v=0.25、scale=0.15、translate=0.05；无mosaic/mixup/旋转 |
| 验证Precision / Recall | 0.003333（0.3333%） / 1.0（100%） |
| 验证mAP50 / mAP50-95 | 0.00996661（0.9967%） / 0.00712589（0.7126%） |

耗时口径为训练器CSV累计时间，未单独测量环境导入、AMP预检和最终独立评估的总墙钟耗时。
P/R/mAP取独立evaluate.py的batch=1评估；训练结束自动验证的mAP50=0.00995略有差异，
原始两组数值同时保留在metrics YAML，报告不挑较高的一组。AP评估使用库默认低置信度
候选（0.001），P/R来自评估曲线工作点，不是可视化conf=0.25的Precision/Recall。
低Precision和近零AP说明高Recall不能解读为可用识别。只有6张、单个验证视频，结果高度不稳定。

数据仅按4个原视频划为14/6，同视频相邻帧不跨split。它们来自同一实验室场景，尚无独立
会话/背景验证。本轮不建正式测试集；未标注161张没有用于训练，原181张哈希前后相同。
版本与标签原文见[version.yaml](../datasets/blue_cube_pilot_v1/version.yaml)，标签未自动生成或修改。

实际查看了6张验证对比、12张定性对比及20张标注叠框：

- 验证`222842...00000000/00000045`：上缘部分出画小目标，conf=0.25漏检。
- 验证`222842...00000090`：目标完整可见且不在最上边缘，仍漏检。
- 未标注`204520...00000195/00000315`：偏暗画面中可辨认的立方体，未输出框。
- 未标注`204520...00000405`：下边缘近处明显截断目标，未输出框。
- 未标注`223231...00000210`：可见较完整目标，仍未输出框。
- 部分历史帧目标可能已出画或很难辨认，零框不自动记作成功负样本，也不计算定量漏检率。

12张定性图片先按既有dim_appearance、edge_check、motion_blur_keep标签选取，再补来源视频，
选择发生在推理前；全部来自训练与验证均未使用的视频，仍属同一lab场景。选择列表、
每图输出与哈希保存在本地predictions_v1。没有拿训练图片证明泛化，也没有调低展示阈值
制造成功案例。小数据量、很少的实际更新及域差异可能相关，当前未做对照，失败根因未确认。

GPU计时使用.pt、batch1、640、FP32、CPU亲和16–19及OMP/OPENBLAS单线程：5次预热后，
18张图各重复5次，共90次。GPU推理均值/P50/P95为6.385/6.368/6.473 ms；同步predict墙钟
均值/P50/P95为8.969/8.931/9.441 ms，后者含读图、预处理与NMS，非ROS端到端延迟。
不从该短时计时推断持续帧率、视频连续漏检或GO2部署性能。

本地成果根目录：`vision_training/runs/blue_cube_pilot_v1/`。

- 模型：`train_v1/weights/best.pt`、`train_v1/weights/last.pt`。
- 曲线与配置：`train_v1/results.png`、`results.csv`、`args.yaml`、`config.yaml`、`data.yaml`。
- 复现：`dataset_manifest.json`、`environment.json`、`requirements.freeze.txt`、`pretrained.json`。
- 独立验证：`eval_v1/metrics.json`、`image_timings.json`、`samples/`及`validation/`。
- 对比总图：`predictions_v1/overview.jpg`；完整`val_comparison.jpg`、`unlabelled_comparison.jpg`。
- 单图与原始结果：`predictions_v1/{val,unlabelled}/`、`cases.json`、`timings.json`、`summary.json`。
- 每阶段原始日志：对应目录`console.log`。本地数据版本：`datasets/blue_cube_pilot_v1/`。

[机器可读指标与哈希](blue_cube_pilot_v1_metrics.yaml)纳入Git；权重、原图、预测媒体与大运行输出留本机。
18项针对性测试通过；未重装框架、未全仓构建、未启动ROS或连接GO2。
真实D435连续推理、跨场景泛化、空间定位和机器人部署均`NOT VERIFIED`。
HSV/MAP/导航/运动控制无本轮修改；原有未提交bridge/10mm分析文件未纳入提交。

下一轮优先新增独立拍摄会话的人工标注：完整可见的中等尺寸目标、弱光小目标、
四边及部分出画目标、不同距离/角度/背景。加入真实蓝色干扰物及经人工确认的无目标图，
避免模型只记地面/颜色。先补数据和审核，保留整会话验证划分，再进行下一次微调；
不要反复用当前6张调参。可以新建配置把model指向本次best.pt做新增数据微调，须使用
新数据版本和新输出目录；这不是恢复已结束训练的优化器状态。若将旧val纳入训练，
必须重新建立验证划分，并撤销它作为留出评估的用途。正式test仍需独立会话。

复现（使用既有环境；此处省略已存在的审核回执准备步骤）：

```bash
# 原输出已存在，重跑先复制配置并改name，避免覆盖。
env -u PYTHONPATH PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  taskset -c 16-19 vision_training/.venv/bin/python vision_training/train.py \
  --config vision_training/configs/blue_cube_pilot_v1.yaml --pilot

env -u PYTHONPATH PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  taskset -c 16-19 vision_training/.venv/bin/python vision_training/evaluate.py \
  --weights vision_training/runs/blue_cube_pilot_v1/train_v1/weights/best.pt \
  --data vision_training/datasets/blue_cube_pilot_v1/data.yaml --pilot --split val \
  --imgsz 640 --device 0 --output vision_training/runs/blue_cube_pilot_v1/eval_v2

env -u PYTHONPATH PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  taskset -c 16-19 vision_training/.venv/bin/python vision_training/tools/pilot_predictions.py \
  --weights vision_training/runs/blue_cube_pilot_v1/train_v1/weights/best.pt \
  --dataset vision_training/datasets/blue_cube_pilot_v1 \
  --source vision_training/datasets/blue_cube/annotation_v1 \
  --output vision_training/runs/blue_cube_pilot_v1/predictions_v2
```

实际执行还设置了临时PYTHONPYCACHEPREFIX/MPLCONFIGDIR及本地YOLO_CONFIG_DIR；
延续现有CPU亲和条件，不声称此前间歇Python异常已解决。本轮训练/评估均正常退出。
