# blue_cube_pilot_v2：小样本更新预算修正实验

2026-11-10；基线与开始前最新Push：`3ea9fbc408c44dc031a5e8432a970a321922cebb`。
证据：RTX4060实际80轮训练、每5轮EMA检测、优化器post-step hook、已有V1和新V2的
同图独立推理、原始CSV/参数/图像/标签哈希及Ultralytics 8.3.40源码。

**结论：修正小样本配置后，模型已真正开始学会这14张训练图片。** V2 last.pt在conf0.25
下训练TP=14、FP=0、FN=0；按验证fitness选择的best.pt为13/0/1。这不代表真实场景泛化。
没有在失败后继续盲目加轮数或追加超参数搜索，本轮只训练一份80轮配置。

## 同图、同模型、同标准对比

V1/V2均使用同一份`datasets/blue_cube_pilot_v1/data.yaml`及审核指纹
`f2cb59d2fac89f27eb510628272b3815fcb2601d0bc8db30ed34ea769d1b4679`：14 train、6 val。
没有建立假test或复制/修改标签。20图来自同一实验室场景，仍按原视频划分。
两版均由相同hash的官方yolo11n.pt初始化、相同YOLO11n架构；V2没有使用旧best/last。
新版本是实验输出版本，不是伪造一份不同训练数据集。

以下均为batch1、imgsz640、FP32、同一推理/匹配代码、NMS IoU0.7、max_det300、无增强。
正确匹配要求类别正确且IoU≥0.5，每个人工GT最多1 TP，重复框计FP。TP/FP/FN不是框总数。
V1数值读取已保存的真实GPU分析结果，V2用相同analyze_pilot.py重新推理，不重跑V1训练。

|权重|conf|train TP/FP/FN|val TP/FP/FN|
|---|---:|---:|---:|
|V1 best（epoch5）|0.05|0/0/14|0/0/6|
|V1 best（epoch5）|0.25|0/0/14|0/0/6|
|V2 best（epoch70）|0.05|14/1/0|5/0/1|
|V2 best（epoch70）|0.25|13/0/1|4/0/2|
|V2 last（epoch80）|0.05|14/1/0|6/0/0|
|V2 last（epoch80）|0.25|14/0/0|4/0/2|

也保存了V1 last同标准指标，见机器可读YAML：V1 last在conf0.25下训练0/11/14、验证0/3/6，
因此不是通过只比较失败V1 best、隐藏已成功的V1 last来制造改善。

V2 last在conf0.25下14个训练匹配的IoU范围0.878–0.974；best的13个为0.908–0.978。
验证4个匹配的IoU范围best为0.804–0.872、last为0.787–0.902。已查看预测叠框图，框对应
实际立方体，不是地面反光。训练图片用于确认拟合能力，不能作为泛化验收。
conf0.05的1个训练FP来自`222516...00000120`的同目标重复框（两个框都与GT有较高IoU），
不是背景误检；仍按一对一规则计FP，未为改善指标修改NMS阈值。

## 参数生效及实际更新

|参数|V1|V2|
|---|---:|---:|
|model / imgsz / batch / device|yolo11n / 640 / 4 / 0|相同|
|nbs|64|4|
|epochs上限 / 实际|50 / 20|80 / 80|
|patience|15|80|
|warmup_epochs|3（默认）|3（显式沿用）|
|workers / seed / amp|0 / 42 / true|相同|
|optimizer实际选择 / 初始目标LR|AdamW / 0.002|相同|
|hsv_v / scale / translate|0.25 / 0.15 / 0.05|相同|
|mosaic / mixup|0 / 0|相同|
|最优轮次|5|70|

解析后的args.yaml逐项比较，除nbs、epochs、patience和新输出/快照路径外，其余参数相同。
V2有效参数已由runtime_setup.json核对，不仅是在YAML写入未透传的字段。

安装版`engine/trainer.py`实际实现：

```python
accumulate = max(round(nbs / batch_size), 1)
nw = max(round(warmup_epochs * nb), 100) if warmup_epochs > 0 else -1
# 预热中的梯度累积从1插值到nbs/batch_size
```

因此不能忽略条件分支，也不能把warmup_epochs=3解释成只预热12个batch。本次明确保留
100batch预热边界（约25轮），320batch总训练量能够越过它，没有声称已关闭预热。
当前源码中0会进入nw=-1分支，但本轮未使用或实际验证零预热；换版本不能直接套用。
以后若需单独检查零预热，应核对版本/条件分支、运行配置和实际批次LR，而非只改YAML。
本轮不修改Ultralytics源码，也不需要为已拟合的结果追加该对照。

实测全部320个batch的`accumulate`均为1，nbs始终为4；PyTorch优化器post-step hook
共记录313次step调用。7个AMP跳步发生于全局batch index 0、1、2、3、4、5、8，伴随
GradScaler缩放调整；不是梯度累积吞掉更新。hook计的是实际optimizer.step调用，
不保证每个参数每次都有非零变化。每个batch的LR、AMP scale、累积值、step累计均已保存。
初始LR从0预热，batch99为0.00139194，最终LR为0.00004475；未把auto配置中的0.01误称实际LR。

V1先前“约17次”仅为源码控制流重建，缺少实际AMP step记录；不可写成V1精确更新次数，
也不可据此计算精确有效更新倍数。V2的313次为本轮hook观测。这次同时改变nbs与训练/早停
预算，证明的是这组最小配置修正能使训练拟合，不能单独归因于nbs或轮数中的某一项。

## 训练期间每5轮检测

以下每项格式均为TP/FP/FN。评估使用独立EMA副本的FP32无增强推理，保存逐图匹配，
不对训练模型做fuse或更新；隔离副本并恢复Python/NumPy/Torch随机状态。监测耗时单独记录。
|Epoch|实际step累计|train 0.05|train 0.25|val 0.05|val 0.25|
|---:|---:|---|---|---|---|
|1|0|0/0/14|0/0/14|0/0/6|0/0/6|
|5|13|0/0/14|0/0/14|0/0/6|0/0/6|
|10|33|0/0/14|0/0/14|0/0/6|0/0/6|
|15|53|0/26/14|0/13/14|0/12/6|0/6/6|
|20|73|0/0/14|0/0/14|0/0/6|0/0/6|
|25|93|0/4200/14|0/4200/14|0/1800/6|0/1800/6|
|30|113|3/0/11|2/0/12|0/0/6|0/0/6|
|35|133|13/3/1|11/0/3|2/1/4|0/0/6|
|40|153|13/4/1|9/0/5|3/0/3|0/0/6|
|45|173|13/3/1|13/0/1|4/0/2|4/0/2|
|50|193|14/2/0|13/0/1|6/0/0|4/0/2|
|55|213|14/1/0|13/0/1|4/0/2|4/0/2|
|60|233|14/2/0|13/0/1|4/0/2|3/0/3|
|65|253|14/1/0|13/0/1|4/0/2|3/0/3|
|70|273|14/1/0|13/0/1|5/0/1|4/0/2|
|75|293|14/1/0|14/0/0|6/0/0|4/0/2|
|80|313|14/1/0|14/0/0|6/0/0|4/0/2|

第25轮FP计数触及每图max_det300上限（train4200、val1800），不能当作未截断原始候选数量。
前期异常不是被隐藏的“平稳收敛”；第30–45轮后才逐渐形成可靠检测。第75/80轮在conf0.25
下训练14/0/0，最终重载last.pt独立推理也得到相同结果，不只靠训练过程中的反馈证明拟合。

## 损失、最优轮次与耗时

|指标|V1 epoch1|V1 epoch20|V2 epoch1|V2 epoch80|
|---|---:|---:|---:|---:|
|train/box_loss|1.72520|0.90982|1.72520|0.59055|
|train/cls_loss|4.92858|1.99748|4.92858|0.43132|
|train/dfl_loss|1.37669|0.95236|1.37669|0.85968|
|val/box_loss|1.52817|1.32728|1.52817|1.37156|
|val/cls_loss|4.19462|4.05627|4.19462|0.89924|
|val/dfl_loss|1.02397|1.06230|1.02397|1.05146|

best.pt来自第70轮，由checkpoint中保存的验证指标与CSV唯一匹配核对；不是训练TP最高
的轮次。best按0.1×mAP50+0.9×mAP50-95的验证fitness选择，所以它的训练拟合不必优于last。
第70轮CSV mAP50=0.995、mAP50-95=0.74461；最终重载best的自动验证为mAP50=0.995、
mAP50-95=0.71868、P=1.0、R≈0.93056。两阶段数值均保留，未把训练中最高值冒充重载结果。
训练时验证和重载评估的精度/模型状态等路径存在差异，本轮未做差异来源消融。
AP曲线工作点P/R与上面的固定conf检测统计口径不同，不互相替代。

V2的model.train调用实测墙钟**87.85秒**，包括初始化/预检、监测、最终验证与保存；
results.csv末行累计74.6945秒（训练器计时，含此前监测）；监测副本推理共5.42秒。
V1 CSV累计16.2077秒。两种时间口径均标明，不能直接称为GPU纯训练时间。

## 数值异常与验证边界

V2第24/25/27轮val的三项loss为NaN，第26轮val/cls_loss为NaN，共10个非有限CSV单元。
原始CSV完整保留；分析JSON用null及nonfinite_csv_events显式记录，图中留空，不置零、不插值。
首次导出因严格JSON拒绝NaN而失败，修正分析导出后重新评估已有权重，没有重训或改CSV。

源码确认AMP启用时训练内验证强制FP16。这使精度/早期EMA统计不稳定成为可能原因，但
**未证实NaN唯一来自FP16或预热**。第25轮独立FP32 EMA监测亦出现大量背景框，不能用
“仅是FP16日志问题”解释掉异常。后期loss恢复有限，best/last独立FP32输出均有限并可匹配。
因此可确认拟合改善，仍不能说本轮全程数值稳定或环境问题已经解决。

预热在本次仍延迟前期学习，但没有阻止80轮最终拟合。既然目标已达，不再盲目延长训练。
若下一轮复现NaN，最小的独立对照可使用现有amp:false配置检查混合精度影响；若要隔离
预热，则在版本/运行日志确认后单独比较warmup_epochs=0。二者不要同时改，本轮均未执行。

6张val仍是同实验室场景且已反复用于开发分析；conf0.25仍漏2张上缘小目标。best对训练
`222516...00000045`的匹配分数仅0.0924，所以0.05能检出、0.25漏检；last已能检出它。
val `222842...00000000/00000045`在last的分数约0.114/0.054，IoU约0.903/0.829，说明
主要是这些边缘小目标的置信度不足，不能靠降低阈值宣称真实泛化已解决。
下一步有价值的是新增独立拍摄会话、弱光/边缘/尺度变化及人工确认负样本，并保留整组
验证，而不是继续对这6张调参。真实D435、跨场景泛化与机器人部署均`NOT VERIFIED`。

## 保存位置与复现

本机实验根目录：`vision_training/runs/blue_cube_pilot_v2/`。

- `train_v2/weights/best.pt`、`last.pt`；`results.csv/png`、`args.yaml`、`config.yaml`、`data.yaml`。
- `train_v2/dataset_manifest.json`、环境/依赖/官方初始化hash快照、`training_result.json`。
- `train_v2/runtime_setup.json`、`batch_audit.jsonl`、`fit_progress.jsonl`、`monitor_result.json`。
- `analysis_v2/analysis.json/yaml`：20图每阈值全部预测、IoU和非有限CSV事件。
- `comparison_v1/{v1_best,v2_best,v2_last}/conf_{0.05,0.25}/{train,val}/`：全部原尺寸预测图。
- `comparison_v1/conf_0.25_overview.jpg`及`conf_0.05_overview.jpg`：左V1 best、中V2 best、右V2 last。
  绿色GT、青色IoU≥0.5候选、红色不匹配框；重复青框仍只计1 TP。每页标明TP/FP/FN。
- `comparison_v1/conf_{0.05,0.25}_page_1..4.jpg`覆盖全部20图，未仅挑成功图片。
- 对应阶段`console.log`保留训练与分析原始日志。

[学习曲线对比](blue_cube_pilot_v2_learning.png)及[机器可读指标](blue_cube_pilot_v2_metrics.yaml)入Git。
权重/原图/预测媒体留本机；V1模型、CSV、报告、20图/标签及181张送标原图共236个保护文件
前后hash全部一致，Ultralytics trainer源码hash前后相同。

```bash
# 已有输出拒绝覆盖；重跑先另建配置并改name，不覆盖本实验。
env -u PYTHONPATH PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  PYTHONPYCACHEPREFIX=/tmp/blue_cube_v2_cache \
  YOLO_CONFIG_DIR="$PWD/vision_training/.ultralytics" MPLCONFIGDIR=/tmp/blue_cube_v2_mpl \
  taskset -c 16-19 vision_training/.venv/bin/python vision_training/train.py \
  --config vision_training/configs/blue_cube_pilot_v2.yaml --pilot --monitor-every 5
```

在现有通用train.py仅增加经校验的nbs/warmup_epochs透传、patience校验与训练调用计时；
监测为显式开启的pilot回调，默认正式工作流和三组审核要求保持不变。
30项针对性测试、flake8、git diff --check及本轮实际CUDA训练/推理检查通过。
未重装框架、未改Ultralytics源码、未做全仓构建，未改HSV/ROS2推理/MAP/导航/运动控制，
未启动或连接GO2；已有无关bridge/10mm未提交改动保留且不纳入提交。
