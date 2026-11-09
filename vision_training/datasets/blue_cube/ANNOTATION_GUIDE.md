# blue_cube第一批人工标注：181张

## 当前停在人工标注环节

远端基线已核对为19c840d50442c7db764e6050e13e39ffb8541cf3。
PC本地213张原图/原manifest完整，213张均能解码，当前pool标签数0、审核回执0。
本轮没有安装依赖、没有训练模型、没有生成自动框或伪标签，没有启动机器人。

质量筛查查看了全部213张缩略图，逐对比较31对冗余候选，并放大极端模糊/边缘等帧。
得到181张可送人工标注图，31张同视频近重复帧不送第一批，1张严重双重拖影帧留待
复核。没有字节完全相同的原图，也没有损坏图；近重复是视觉筛选，不冒充精确重复。
原始213张不删除、不改写，不把低清晰度地砖背景自动判成无效。

本机目录（大图片/ZIP不提交Git）：

```text
vision_training/datasets/blue_cube/
  annotation_plan.yaml               # 可追溯筛选：文件名、代表帧、注意项、输入哈希
  annotation_report.yaml             # 本次计数与包哈希
  pool/images/lab_floor_20261108_09/    # 原213张，全部保留
  annotation_v1/
    images_for_cvat.zip               # 直接上传CVAT，仅181张原尺寸RGB，无标签
    images/                          # 与ZIP一致的原图副本
    classes.txt                      # 只有blue_cube
    manifest.json                    # 每帧来源/源时间/哈希/筛选状态
    human_review.csv                 # 181张逐图审核表，全部PENDING、数量为空
    hold/                            # 严重拖影帧，不在上传ZIP中
    contact_sheets/                   # 213张全量筛选索引图
    summary.json
```

`attention`仅为人工复核提示，不是目标框、负样本真值或正式困难场景标签。
保留提示计数：motion_blur_keep 15、edge_check 28、dim_appearance 18、
small_target_check 26；同一图片可命中多个提示，不能相加，也不能据此宣称场景均衡。
第一批只有一个capture_group，不随机拆成train/val/test。

## 最简CVAT操作

1. 打开你已有的CVAT实例（或CVAT在线服务），新建项目，**只建一个类别blue_cube**，
   图形用Rectangle。不要创建wall/cable/reflection正类，不开自动标注。
2. 新建图片任务，上传本机`annotation_v1/images_for_cvat.zip`。
   确认任务图片数为181，原图640×480，文件名没有改写；不必安装新GUI或新训练环境。
3. 逐张画框并检查漏标，按下面规则；可同时填`human_review.csv`的
   human_status（POSITIVE/NEGATIVE/UNSURE）、object_count、reviewer和notes。
   每张都需检查，不能把“没有画框”自动解释为已检查的背景。
4. 人工复核全部图片及类别后，导出 **Ultralytics YOLO Detection 1.0**（普通矩形
   检测格式，不选Track/OBB/Segmentation）。版本菜单名称略有差异，以普通Detection
   五列标签为准；包含图片导出便于完整交回审核。保存ZIP与审核表，保留原文件名。
5. 将导出ZIP和审核表放回PC。下一步先核对`data.yaml names: {0: blue_cube}`、
   文件配对、框界限、漏标/背景和独立拍摄组，再由现有工具生成审核回执与训练split。
   **本轮在此之前停止，不提前运行review-group，不启动train.py。**

官方格式说明：[CVAT Ultralytics YOLO导出](https://docs.cvat.ai/docs/dataset_management/formats/format-yolo-ultralytics/)。
该导出含data.yaml与labels下对应图片的txt，每行`class_id cx cy w h`（坐标归一化）。
CVAT导出的默认train目录只是导出组织方式，不能当成本项目已完成防泄漏划分。
若导出省略空标签文件，必须先核对人工NEGATIVE记录，再补对应空txt；不能为全部
缺失标签批量生成空txt，否则会把漏标正样本错误地当成背景。

## 统一画框规则

- 每张里所有能辨认的蓝色立方体都框；一个实体一个矩形，紧贴整个可见实体，
  包括可见的顶面和侧面，不只圈最亮的蓝色像素。
- 部分出画也框其**可见部分**，框限在0..640、0..480之内，不补猜画外尺寸。
  特别检查顶部、左侧和底部：例如源索引085、107、179等，不能因只有一小块就跳过。
- 清晰度下降但仍能辨认实体的运动模糊图保留并画可见范围；不要框地面拖影/倒影，
  不将矩形向阴影/反光扩大。索引148、173、174、209等需仔细区分实体与地面反光。
- 蓝墙、线缆、亮斑、地面蓝色倒影不标为blue_cube。完全没有可辨认目标的背景图
  保留，逐张人工确认NEGATIVE、object_count=0；导出应对应空标签。
- 不能判断实体边界时填UNSURE，不猜框、不当背景；与上下文原视频比对后再定。
  `hold/approach_20261108_222156_798690_raw_00000150.jpg`即原索引017，严重双重重影，
  当前未送标注；它不是已确认负样本，不需要为了凑数量强行使用。
- 不使用HSV检测结果作为标签真值。索引只用于查manifest/contact sheets，CVAT用原文件名。

## 场景覆盖与最少补采

以下依据本轮视觉质量筛查，不是人工目标标注、实测距离/照度分组或模型评估结果。

| 场景 | 当前证据/限制 | 最少补充 |
|---|---|---|
| 正常光照 | 有较清晰单目标，集中同一地砖/反光背景 | 换一个背景/摆位，避免只记住地砖 |
| 弱光 | 有偏暗、噪声/曝光变化画面；未测照度，没有独立弱光组 | 独立低照场景，确认目标仍可辨认；含背景负样本 |
| 视野边缘 | 有右侧近边、顶部/左侧/底部截断；保留重点复核帧 | 补左右边缘、角落不同截断比例，仍按可见范围标框 |
| 运动模糊 | 保留15张重点提示样例；另有1张严重重影暂不使用 | 手持相机轻移/小幅移动目标，少量可辨认模糊，不需自动运动 |
| 远距离 | 有图像中很小的目标；不能仅据像素大小填写实际米数 | 用尺量0.5/1/2/3 m等少量距离，保存距离记录 |
| 蓝色干扰物 | 有线缆/反光；线缆看起来深色，不能当作已覆盖蓝色线缆；未见蓝墙覆盖 | 蓝色纸板/墙面、蓝色线缆与反光物，分别拍纯负样本与目标同框 |
| 多目标 | 213张质量筛查未见足够明确的多目标场景 | 放2–3个物块，同框分开/近邻/轻遮挡，各实体标框 |

优先补**两个独立拍摄组**，每组约40–60张人工挑选图片作为初始建议，不是已采集结果：

- 新组B：更换背景/布局，正常光照；目标单/多实例与蓝色干扰物，含无目标背景。
- 新组C：另一次采集、不同布局/光照，加入弱光、边缘和少量可辨认模糊，也有普通画面。

可用固定相机/手持相机、人工摆物，不安排大量GO2自动运动。现有组可先作训练候选，
新组按独立会话分配val/test；不要将同会话相邻帧或同布局重跑按视频文件名随机拆开。
测试集固定后不用于反复调参。B/C均需类别/框/背景审核，不因为新拍摄就自动视为真值。

## 人工审核后接回已有训练工作流

先将181张已标图片与新独立组的标签按manifest合并为干净的待训练版本，排除31个冗余
和1个待定帧，保留原pool作证据。**不要直接对原213张pool执行review-group**，因为
其中32张不在本次CVAT任务内；也不要把CVAT导出全放train后声称完成了独立测试集。
导入和按拍摄组划分使用已有tools/dataset.py；合并后让
`vision_training/datasets/blue_cube/data.yaml`指向审核后的images/labels与names即可，
默认`configs/blue_cube.yaml`及train.py不需要改核心代码。

确实完成合并、独立划分及审核回执后，才执行：

```bash
cd /home/czx666/today_full_20260901/today_full
source vision_training/.venv/bin/activate
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PYTHONPYCACHEPREFIX="$PWD/vision_training/.pycache"
export YOLO_CONFIG_DIR="$PWD/vision_training/.ultralytics"

python vision_training/tools/dataset.py check \
  --data vision_training/datasets/blue_cube/data.yaml --require-review
python vision_training/train.py --config vision_training/configs/blue_cube.yaml
```

仍使用既有RTX4060/YOLO11n环境；不重装依赖。输出按原框架保存best.pt、last.pt、参数、
曲线与数据/依赖快照；随后用evaluate.py在独立test上出报告。当前这些前置条件未满足，
没有生成真实蓝色立方体权重或mAP。

## 标注包复现

已有annotation_v1请直接使用，重复执行不覆盖：

```bash
python vision_training/tools/prepare_annotation.py \
  --dataset vision_training/datasets/blue_cube \
  --plan vision_training/datasets/blue_cube/annotation_plan.yaml \
  --output vision_training/datasets/blue_cube/annotation_v2
```

该工具使用现有OpenCV/YAML，校验原manifest/图片指纹及ZIP每张内容，不安装依赖，
不创建labels或review回执。输入图变化需要重新视觉审核计划，不能复用旧筛选结果。
换正式比赛目标时新建数据集/类别/标注，继续使用相同train.py、evaluate.py、export.py，
最后仅修改ROS2权重路径及需要的类名筛选；competition.example.yaml保持原样。
