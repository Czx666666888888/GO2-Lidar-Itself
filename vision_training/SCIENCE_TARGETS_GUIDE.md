# 科学目标：采集、CVAT 标注与训练准备

2026-11-11；仅视觉感知。路线为 RGB YOLO11n + D435 Aligned Depth 相机系粗定位。
不改变 HSV、深度检测阈值、MAP 转换、导航或接近控制。

## 首批实际采集

使用静态 D435、人工摆放目标和 GO2 实际补光条件即可，不需要 GO2 自主运动。
先确认预览、RGB/Aligned Depth/CameraInfo 持续到达，且尺寸相同、相机帧一致。
本轮本机没有枚举到 D435，真实采集 `NOT VERIFIED`。

| 内容 | 需要覆盖 |
|---|---|
| color_block | 六种真实颜色的 5 cm 方块，近/中/远、正面/斜视、单个和多个 |
| rock_compact | 灰色紧凑低多边形人造岩石，多视角/背景/距离；记录实测尺寸与材质 |
| rock_spiky | 明显尖峰的人造岩石，包括尖峰遮挡、轮廓重叠；记录实测尺寸与材质 |
| 负样本 | 普通岩石、灰色障碍物、无目标地面；必须人工确认确实没有三类目标 |
| 困难样本 | 真实弱光与补光、边缘、运动模糊、普通岩石干扰、小障碍物及≤6°坡面背景 |

每次记录日期、会话 ID、场景、补光/曝光、距离、目标摆放和设备安装状态。
建议至少三组独立会话/场景，分别用于 train/val/test；同一次摆放连续录制的多个视频
属于同一组。不要随机拆相邻帧，也不要只改会话名就声称场景独立。
软件亮度增强不能算真实弱光覆盖；六色按同一类别标注。岩石未到位时先标注已有实物，
不伪造样本或假定尺寸。RGB 标注图无需全部配 Depth；几何测试另保留 RGB-D。

## PC 采集 RGB-D

仓库根目录执行。若相机尚未启动，仅启动一次现有相机/预览 launch：

```bash
source /opt/ros/humble/setup.bash
source camera_ws/install/setup.bash
ros2 launch go2_science_perception depth_viewer.launch.py
```

它包含相机驱动；已有唯一相机源时不重复启动，只运行现有 viewer 节点：
`ros2 run go2_science_perception depth_viewer`。
确认画面和流状态后，新终端使用源码入口，无需全仓构建：

```bash
cd /home/czx666/today_full_20260901/today_full
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/camera_ws/src/go2_science_perception:$PYTHONPATH"
/usr/bin/python3 -m go2_science_perception.rgbd_capture --ros-args \
  -p session_id:=session_A_20261111 -p scene:=gray_cave_lamp_on \
  -p duration_sec:=30.0 -p sample_interval_sec:=1.0 \
  -p max_rgb_depth_delta_sec:=0.10
```

采集器只订阅三个现有相机话题，不启动驱动，不发布任何 ROS 消息。
完成时间为 duration + 约 0.2 s 配对等待；Ctrl-C 会保存待处理图并退出。
相机话题可通过 `color_topic`、`depth_topic`、`camera_info_topic` 参数指定，
Depth 必须是 wrapper 的 `aligned_depth_to_color` 流。

默认保存到 `camera_ws/records/rgbd/<session_id>/`，已有会话目录拒绝覆盖：

- `session.json`：会话、场景、话题与参数；`summary.json`：接收/保存/可配对样本数。
- `<序号>_<RGB纳秒>/rgb.png`：无叠字 PNG，OpenCV 读回为 BGR8；是相机原图内容。
- 同目录 `depth.npy`：原始数值和 dtype，无缩放/彩色化；`16UC1/mono16` 为 uint16 毫米，
  乘 0.001 转米；`32FC1` 为 float32 米，NaN 与深度空洞原样保存。
- `sample.json` 和会话 `samples.jsonl`：RGB/Depth/CameraInfo 各自源时间戳、实际
  `RGB−Depth` 秒差、CameraInfo 时间差、frame、尺寸、K/D/R/P、编码、单位及会话 ID。
  `frame_index` 是采集器收到 RGB 的序号，不冒充相机硬件帧计数；静态流无源视频时
  `source_video=null`，以源话题、会话和精确时间戳追溯。

RGB 样本到达后等待 0.2 s，从最近 90 条 Depth/CameraInfo 按源时间选择最近者；
这只是近时间匹配，**不是硬件同步证明**。超时差、尺寸/frame 不同、无深度/内参等
会写 `flags`、`geometry_usable=false`；可用 RGB 仍保留，缺 Depth 时不写假 depth.npy。
只有 `flags=[]` 的完整样本可进入后续几何检查，仍不能据此认定对齐精度已标定。
首批需实际查看图像/深度数值、时间差分布和连续速率；本轮尚未实测这些。

读回一份样本（替换实际目录）：

```python
from pathlib import Path
import cv2, json, numpy as np
sample = Path('camera_ws/records/rgbd/session_A_20261111/实际样本目录')
m = json.loads((sample / 'sample.json').read_text())
rgb = cv2.imread(str(sample / m['rgb_file']))
depth = np.load(sample / m['depth_file'], allow_pickle=False) if m['depth_file'] else None
print(rgb.shape, None if depth is None else (depth.shape, depth.dtype),
      m['rgb_depth_delta_sec'], m['flags'], m['camera_info'])
```

## 导入标注池并上传 CVAT

沿用已有 venv，不重装依赖；命令中的会话名、文件名应换为实际值。

```bash
vision_training/.venv/bin/python vision_training/tools/dataset.py import-rgbd \
  --dataset vision_training/datasets/science_targets_v1 \
  --session camera_ws/records/rgbd/session_A_20261111 \
  --group session_A --tags lamp_on gray_cave
```

该入口复制 RGB、保留会话/场景/原时间戳/帧序号/完整 RGB-D sidecar 路径，**不创建标签**。
普通源视频可继续使用已有 `dataset.py extract --dataset ... --video ... --group ...`；
manifest 保留视频路径和帧号，用 `--tags` 记录场景/光照；普通图片用已有 `import`。
同场景会话必须统一 group。几何样本不合格不表示 RGB 是负样本。

把 `pool/images/<group>/` 的原图上传 CVAT，同一个任务可混合多类、多目标。
固定标签顺序：

```text
0 color_block
1 rock_compact
2 rock_spiky
```

六色方块统一 color_block；两种人工岩石按结构标注，普通自然岩石作为干扰背景。
逐图标全部可辨认目标，包括边缘/部分遮挡；无法可靠判断的图标为 uncertain，暂不训练。
只需 Bounding Box，不要 mask/6D 位姿。导出 **Ultralytics YOLO Detection**（data.yaml +
labels/...）或经典 **YOLO 1.1**（obj.names + obj_train_data/...）ZIP。
建议 CVAT 开启图像导出以单独备份；导入器使用本地池图片，不采用 ZIP 中的 split。

人工制作 review.csv（image 为原文件名，不能有重复）：

```csv
image,status
实际含目标图.png,positive
实际无目标图.png,negative
尚未完成图.png,pending
无法确认图.png,uncertain
```

positive 必须有有效框；negative 必须明确人工确认且有空 txt。
若 CVAT 导出省略空 txt，请人工确认对应 negative 后在**新导出副本**补齐空文件并重新打包；
禁止按 train.txt 批量给缺失标签图片补空文件。pending/uncertain 不导入为训练标签。

```bash
vision_training/.venv/bin/python vision_training/tools/import_cvat.py \
  --dataset vision_training/datasets/science_targets_v1 \
  --archive /实际路径/cvat_export.zip --review-csv /实际路径/review.csv \
  --reviewer '实际审核人' --confirm-manual-review
```

所有选中记录先检查再写入；保留原标签 bytes、ZIP 哈希、名称映射和审核信息。
旧 blue_cube 导出可加 `--class-map /路径/alias.yaml`，文件内容 `blue_cube: color_block`。
历史已迁移框可人工复查三类完整性后，使用既有 `review-group --confirm-manual-review`；
不要让软件替人签署审核。标签需修改时建立新数据版本，工具不覆盖不同的既有标签。

## 划分、预算检查与以后训练

待三类实物及负样本标注完成，复制数据集内 splits.example.yaml，填写真实 group。
按组完整划分，验证/测试必须来自独立采集会话或场景；20 张历史图不拆到多个 split。

```bash
vision_training/.venv/bin/python vision_training/tools/dataset.py split \
  --dataset vision_training/datasets/science_targets_v1 --assignments /路径/splits.yaml
vision_training/.venv/bin/python vision_training/tools/dataset.py check \
  --data vision_training/datasets/science_targets_v1/data.yaml --require-review
vision_training/.venv/bin/python vision_training/tools/training_budget.py \
  --config vision_training/configs/science_targets_v1.yaml
vision_training/.venv/bin/python vision_training/train.py \
  --config vision_training/configs/science_targets_v1.yaml --dry-run
```

当前这些正式检查会因还没有完整 split/审核而拒绝，属于预期；**本轮不执行正式训练**。
正式基线 YOLO11n/640/device0/batch4/epochs100/patience30；增强为 hsv_v0.3、degrees10、
fliplr0.5、flipud0、scale0.15、translate0.05，不启用强制 CLAHE 或其他网络结构。
预算工具报告每个 split 的框数和缺失类别；补齐三类独立验证/测试后再启动训练。

本机固定 Ultralytics **8.3.40** 源码：单 GPU `nb=len(train_loader)`，通常为
`ceil(N_train/min(batch,N_train))`；梯度累积初始 `max(round(nbs/batch),1)`，预热中从
1 向 nbs/batch 插值；预热边界 `max(round(warmup_epochs*nb),100)`（warmup>0）。
当前 nbs4 只是 batch4 的无累积起点；数据量/显存变化后须重新核对，不能直接推广 V2。
正式 train 尚未分配，实际 nb、显存占用、实际更新次数均 `NOT VERIFIED`。
100 epoch 是上限，patience30 可提前结束；batch 数不是有效参数更新数，AMP 可能跳步。
先运行预算/配置 dry-run，实际训练时再核对 args.yaml、DataLoader 长度和累积日志，
不要使用仅支持单类单框的旧 `--pilot --monitor-every` 审计入口运行正式三类数据。

## ROS2 输出与当前验证

源码确认 `yolo_detector.py` 类别取自权重，支持三类/多框；JSON 每个目标含 class_id、
class_name、confidence、bbox_xyxy，有可靠深度时同一目标增加 xyz_m，整体有原 RGB
时间戳与 camera frame。独立 PointStamped 沿用 RGB header；PointStamped 本身没有类别，
需要类别与三维点关联时使用 JSON 的同一 detection，不靠多条点的到达顺序猜测。
原 HSV 接口保留，YOLO 不接 MAP、manager 或运动；本轮未改 detector/geometry 阈值。

本轮：迁移真实 20 图并校验历史哈希；CVAT 多类别/多框/人工负样本/缺失标签测试；
uint16/float32 深度无损读回、时间戳与内参关联检查；既有训练框架兼容检查；
仅本机独立 ROS 域两秒无相机启动/退出（0 帧）。详细结果见
[本轮验证记录](reports/science_targets_v1_setup.md)。
仍待：真实 D435 静态采集、实际对齐/时间差/吞吐、正式岩石与六色数据、人工审核、
独立 split、三类别权重训练和真实识别/粗定位精度；全部 `NOT VERIFIED`。
