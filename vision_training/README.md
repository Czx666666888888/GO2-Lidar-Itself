# 可复用 YOLO 训练与 PC ROS2 推理

默认 YOLO11n 官方预训练权重，模型、类别和数据集均由配置决定。
正式目标公布后新建数据目录、标注并换 YAML 即可，不改训练核心或检测逻辑。
旧 HSV 与真实接近程序保持原样；本节点不接 manager/地图/导航/运动。

## 目录

```text
vision_training/
  configs/blue_cube.yaml           # 首个项目的训练参数
  configs/splits.example.yaml      # 拍摄组分配示例
  datasets/blue_cube/data.yaml     # 仅这里声明 blue_cube 类别
  tools/dataset.py                 # 抽帧、导入、审核、分组、检查
  tools/inventory_videos.py        # 原始视频盘点
  tools/check_environment.py       # 版本和真实 CUDA 运算检查
  tools/ros_smoke.py               # 隔离 DDS 的实际模型/合成深度检查
  train.py / evaluate.py / export.py
  common.py / dataset.py / tests/
  requirements.txt
  .venv/ pools/images/labels/runs/weights/artifacts/  # 本机生成，不提交大数据
```

## 1. 环境（Ubuntu 22.04 PC）

本机检测到 RTX 4060 Laptop 8 GB，驱动 595.99.02，Python 3.10.12。
`nvidia-smi` 的 CUDA 13.2 是驱动支持信息，不代表本工作流使用 CUDA 13.2。
选择官方 torch 2.5.1/torchvision 0.20.1 CUDA 12.4 wheel，不安装或升级系统 CUDA。
版本与实际检查结果见 [VALIDATION.md](VALIDATION.md)。其他 GPU 应重新检查兼容性。

在新的 PC 终端执行；训练不需要 source ROS、不需要机器人连接：

```bash
cd /home/czx666/today_full_20260901/today_full
/usr/bin/python3 -m venv vision_training/.venv
source vision_training/.venv/bin/activate
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PYTHONPYCACHEPREFIX="$PWD/vision_training/.pycache"
export YOLO_CONFIG_DIR="$PWD/vision_training/.ultralytics"
python -m pip install --upgrade pip==24.3.1
python -m pip install torch==2.5.1 torchvision==0.20.1 \
  --index-url https://download.pytorch.org/whl/cu124
python -m pip install --no-compile --no-warn-conflicts -r vision_training/requirements.txt
python -m pip check
python vision_training/tools/check_environment.py --output /tmp/yolo_environment.json
```

如未安装 venv 组件，先安装 `python3.10-venv`。不要用 sudo pip 或把这些库装入系统。
训练依赖和 ROS2 都通过 `/usr/bin/python3` 的 Python 3.10 ABI；venv 没有
`--system-site-packages`。numpy固定1.26.4以兼容Humble的cv_bridge，OpenCV固定4.10。
ROS使用时显式source ROS并指定venv解释器；系统HSV继续使用原解释器。
本机安装中出现过pip的ZIP解析/段错误，使用保留wheel重试并关闭安装期编译/重复冲突
扫描后安装完成，再单独`pip check`通过。检查过程也有间歇Python导入异常，根因尚未
确定；隔离缓存不代表已修复。部分后续检查使用`taskset -c 16-19`及单线程通过，
此为本机运行条件，不是其他PC通用要求，也不视为长期稳定性验收。详见VALIDATION。

精确复现本次已解析依赖（仍先安装对应CUDA wheel）：

```bash
python -m pip install --no-compile --no-warn-conflicts \
  --extra-index-url https://download.pytorch.org/whl/cu124 \
  -r vision_training/requirements.lock.txt
python -m pip check
```

## 2. 数据格式、标注与防泄漏

标准检测标签：每行 `class_id x_center y_center width height`，后4项归一化至0..1。
`data.yaml`支持names列表或连续0..N-1映射，train/val/test支持图片目录、图片清单txt
或其列表。相对path以data.yaml目录为基准；清单内相对图片路径以清单所在目录为基准。
目录需遵循`images/...`对应`labels/...`，背景图也必须有人工确认的空txt，缺失标签报错。
图片清单建议用绝对路径，工具生成的清单均为绝对路径。

```text
datasets/my_target_v1/
  data.yaml
  pool/images/capture_A/*.jpg      # 待标注，工具生成
  pool/labels/capture_A/*.txt       # 人工标注后放入，文件名必须对应
  pool/manifest.json               # 来源、frame index/视频秒数、group/tags、审核哈希
  images/{train,val,test}/...      # 分组完成后生成
  labels/{train,val,test}/...
  subsets/test_low_light.txt       # 仅对人工明确标记的tag生成
  data.review.json                # 人工审核与当前文件/类别指纹绑定
  split_manifest.json             # group→split与源图像关系
```

推荐用现有 [CVAT](https://docs.cvat.ai/docs/manual/advanced/formats/format-yolo/)
标注并导出YOLO检测格式。将导出的txt按原图片路径放入pool/labels；核对导出的类别ID
次序与data.yaml一致。无需开发或安装自定义GUI。HSV候选只可辅助人工检查，不能直接
视作真值；本轮未生成伪标签。未标注图片不能用空txt批量冒充负样本。

抽帧（原视频保持不变；相同拍摄会话/布景使用同一group）：

```bash
python vision_training/tools/dataset.py extract \
  --dataset vision_training/datasets/blue_cube \
  --video camera_ws/records/approach_20261109_203609_327607_raw.mp4 \
  --group capture_session_A --interval 1.0
```

已有图片及标签导入（仍然是未审核状态）：

```bash
python vision_training/tools/dataset.py import \
  --dataset vision_training/datasets/blue_cube \
  --images /absolute/path/images --labels /absolute/path/labels \
  --group capture_session_B --tags low_light motion_blur
```

tags是人工给定的拍摄组属性；只能把确实整个组符合条件的数据标成该属性。
混合视频应先按场景片段整理、明确帧级子集，不能给整段强加困难场景标签。

人工逐张检查类别、边界框、漏标和负样本后，记录审核人（**不能提前执行**）：

```bash
python vision_training/tools/dataset.py review-group \
  --dataset vision_training/datasets/blue_cube \
  --group capture_session_A --reviewer YOUR_NAME --confirm-manual-review
```

复制`configs/splits.example.yaml`，把所有拍摄组各分配一次。相邻帧、同一原视频及同一
连续会话不得跨train/val/test；同场景连续重跑亦建议同组。至少准备3个独立组。
分组由人确定，工具检查group互斥及跨split完全相同图片哈希；它不能自动证明场景独立，
也不能识别所有近重复图像。不能用多视频文件名替代真实拍摄组。

```bash
python vision_training/tools/dataset.py split \
  --dataset vision_training/datasets/blue_cube \
  --assignments vision_training/configs/splits.local.yaml
python vision_training/tools/dataset.py check \
  --data vision_training/datasets/blue_cube/data.yaml --require-review
```

工具检查图片可读、标签配对/孤儿、每行格式、类别ID、有限坐标、框界限与跨split重复。
split不会覆盖已生成目录；数据更新采用新版本目录。审核后的图片/标签/类别变化会使
训练前指纹检查失败，必须重新审核。已有外部完整YOLO数据集经人工确认可执行：

```bash
python vision_training/tools/dataset.py review-dataset \
  --data /absolute/dataset/data.yaml --reviewer YOUR_NAME --confirm-manual-review
```

外部数据的拍摄组划分仍由你保证，单纯审核回执不代表已验证防泄漏。

本机首个项目已提取10份raw视频的213张待标注图片，归为同一lab_floor拍摄组，
**目前无人工审核标签，不可正式训练**。详情及缺场景见
[datasets/blue_cube/README.md](datasets/blue_cube/README.md)。不要重复执行上述抽帧示例
来产生相同视频的另一个组；示例用于新数据，或先使用新的数据集版本目录。

后续已完成该批图片的质量筛查并打包181张CVAT送标图片，原213张保留；
见[最简标注步骤与数据覆盖](datasets/blue_cube/ANNOTATION_GUIDE.md)。新增
tools/prepare_annotation.py仅根据已视觉复核的计划准备图片包，不生成标签/审核回执。

## 3. 配置训练

编辑`configs/blue_cube.yaml`：project、data、model、output、name、imgsz、epochs、batch、
device、patience、workers、seed、augmentation。所有配置路径相对配置文件；官方模型
名如`yolo11n.pt`下载到weights/；本地模型可写相对/绝对路径。仅支持detect任务。

```bash
python vision_training/train.py --config vision_training/configs/blue_cube.yaml --dry-run
python vision_training/train.py --config vision_training/configs/blue_cube.yaml
```

当前无审核数据时dry-run也会明确失败，这是预期的前置条件检查。
8GB显存初始batch=8，OOM后减为4，不应通过修改系统CUDA处理普通显存不足。
相同输出目录不会覆盖，重跑修改name。预训练是官方COCO权重，不等于已识别新目标。

输出`runs/blue_cube/baseline_v1/`包含Ultralytics的weights/best.pt、last.pt、args.yaml、
results.csv、results.png、验证曲线/混淆矩阵，以及config.yaml、绝对路径data.yaml、
dataset_manifest.json（图片/标签哈希）、environment.json、requirements.freeze.txt、
pretrained.json（源权重哈希）、training_result.json。保存dataset目录与原始标签才能
重现训练；快照不复制大图像，也不保证不同GPU逐位相同。勿在训练中改数据。

## 4. 评估与困难场景

```bash
python vision_training/evaluate.py \
  --weights vision_training/runs/blue_cube/baseline_v1/weights/best.pt \
  --data vision_training/datasets/blue_cube/data.yaml --split test \
  --imgsz 640 --device 0 --output vision_training/runs/blue_cube/test_v1
```

输出Precision、Recall、mAP50、mAP50-95、每图时间、预热后batch=1端到端wall均值/P50/P95，
同时保存库内preprocess/inference/postprocess速度。wall时间包含图片读取与NMS，
不代表ROS端到端延迟。样例默认保存前20张而非挑选高分图。
评估权重类别次序必须与data.yaml一致，否则拒绝。test只用于最终验收，不反复调参。

仅在真实存在并人工标记子集时：

```bash
python vision_training/evaluate.py \
  --weights vision_training/runs/blue_cube/baseline_v1/weights/best.pt \
  --data vision_training/datasets/blue_cube/data.yaml --split test --subset low_light \
  --device 0 --output vision_training/runs/blue_cube/test_low_light_v1
```

没有tag/子集会报错，不伪造分组指标。最长连续漏检与实际物块检出率需人工审核视频
时间线/目标身份，当前不计算，也不能用抽帧AP推断。

## 5. 导出

```bash
python vision_training/export.py \
  --weights vision_training/runs/blue_cube/baseline_v1/weights/best.pt \
  --data vision_training/datasets/blue_cube/data.yaml --format onnx \
  --imgsz 640 --version blue_cube_v1 \
  --output vision_training/artifacts/blue_cube_v1_onnx
```

将`--format onnx`改成`--format pt`，另用新output可生成PyTorch部署包。
导出目录包含model.pt、可选model.onnx及metadata.json：类别映射、分辨率、版本、
权重哈希和推理配置。ONNX经过checker与实际推理后重新核对类名。
静态batch=1、FP32、opset17；第一阶段ONNX使用CPU runtime，GPU推理优先.pt。
节点启动时有metadata.json则校验字节哈希、names、imgsz；不一致拒绝启动。
confidence/iou在ROS配置中显式设置，metadata记录导出时建议值，不静默覆盖用户配置。
TensorRT留待目标GPU构建/实测；PC engine不能默认复制给GO2 Orin/NX。

## 6. PC ROS2独立推理

仅build本包，使用新的安装目录，不替换现有实机HSV安装：

```bash
cd /home/czx666/today_full_20260901/today_full
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
colcon --log-base /tmp/yolo_ros_build_logs build --base-paths camera_ws/src \
  --packages-select go2_science_perception \
  --build-base /tmp/yolo_ros_build --install-base /tmp/yolo_ros_install
source /tmp/yolo_ros_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
export PYTHONNOUSERSITE=1
export PYTHONPYCACHEPREFIX="$PWD/vision_training/.pycache"
export YOLO_CONFIG_DIR="$PWD/vision_training/.ultralytics"

ros2 launch go2_science_perception yolo_detector.launch.py \
  python_executable:="$PWD/vision_training/.venv/bin/python" \
  model_path:="$PWD/vision_training/artifacts/blue_cube_v1_onnx/model.onnx" \
  params_file:="$PWD/camera_ws/src/go2_science_perception/config/yolo_detector.yaml"
```

ONNX时先复制参数文件为本机配置，把device改为`'cpu'`。`.pt`默认device=`'0'`。
上述命令需要已启动D435 RGB/Aligned Depth/CameraInfo，不会自动启动相机或机器人。
实际蓝色立方体模型尚未训练，示例artifact路径目前不存在，不要将COCO模型当作它。
可在独立终端复用现有相机启动方式（连接/对时需正常）：

```bash
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 \
  'bash /home/unitree/go2_d435_foxy_ws/launch_camera.sh'
```

配置支持model_path、metadata_path、class_names（逗号分隔实际模型类名，空为全部）、
confidence、iou、imgsz、device、速率和深度阈值。更换配置/模型后重启节点，无需改代码。
只接受本地已有.pt/.onnx，ROS启动不下载权重；预训练checkpoint亦必须来自可信来源。

| 输出 | 类型 | 关联与含义 |
|---|---|---|
| `/science/yolo/detections` | std_msgs/String，JSON schema_version=1 | 每源RGB帧一个，包含rgb_stamp_ns/frame_id/图像尺寸、实际names、model_sha256、N个class_id/name/confidence/bbox_xyxy及可选xyz_m/depth质量 |
| `/science/yolo_target_point` | geometry_msgs/PointStamped | 每个可靠3D一个，header等于原RGB；坐标单位m、color optical；多目标对应同帧JSON的有效depth/xyz条目顺序 |
| `/science/yolo/debug_image` | sensor_msgs/Image | 框图，header保留原RGB；无需窗口 |

JSON每帧记录inference_ms（含预处理/NMS的predict调用wall，GPU已同步）、库内speed、
processing_ms、RGB/Depth原始ns及有符号时间差。`log_path`可指定JSONL永久记录。
只处理最新RGB、同一stamp一次；过期/未来RGB发拒绝状态。Depth按缓存最近源时间匹配，
RGB可继续2D推理，不依赖Depth到达。尺寸/frame/时间差/深度有效率/样本数/10–90分位
散布不合格均抑制3D，2D仍保留。相机内参和支持的畸变系数用于反投影。
框中央ROI的深度是粗定位，不能证明该表面必属目标；背景占据ROI时仍需场景验证。
与旧HSV点不同，此处每个点使用RGB时刻并在JSON保存实际深度时刻，无同步则不发点。
无深度绝不伪造Z、外参、map点或导航目标。

YOLO节点没有`/science/target_coarse_point`、`/goal_point`、`/cmd_vel_raw`或Sport输出。
现有HSV可独立运行，两者输出不重复；本阶段不把YOLO重映射到HSV正式接口。

## 7. 正式目标公布后的最少步骤

1. 新建`datasets/competition_v1/data.yaml`，定义实际names和split目录。
2. 整理按会话分组的新视频/图片，用CVAT人工标注并逐组review，整组split。
3. 复制训练YAML，改project/data/name及需要的model/训练参数；执行check、train。
4. 固定test及已标记困难子集evaluate；保留原始视频作后续连续漏检评估。
5. export到新版本artifact；ROS参数换model_path、imgsz及目标class_names，重启。

不要求修改train.py、evaluate.py、export.py或ROS检测逻辑。

## 验证及官方依据

```bash
PYTHONPATH=.:camera_ws/src/go2_science_perception python -m pytest -q \
  vision_training/tests camera_ws/src/go2_science_perception/test/test_yolo_geometry.py
```

实际执行记录与未验证项见[VALIDATION.md](VALIDATION.md)。本工程源码遵循仓库原许可；
Ultralytics第三方代码/权重使用其自身AGPL-3.0或相应商业许可，未将第三方权重提交仓库。

- [PyTorch官方历史版本CUDA wheel](https://docs.pytorch.org/get-started/previous-versions/)
- [YOLO11](https://docs.ultralytics.com/models/yolo11/)
- [YOLO数据格式](https://docs.ultralytics.com/datasets/detect/)
- [训练](https://docs.ultralytics.com/modes/train/)、[评估](https://docs.ultralytics.com/modes/val/)、[导出](https://docs.ultralytics.com/modes/export/)

## 2026-11-10：20图pilot（不替代正式工作流）

用户CVAT人工标注已导入独立`blue_cube_pilot_v1`，按原视频14 train / 6 val，无test。
仅显式`train.py --pilot`与`evaluate.py --pilot --split val`接受两组；默认仍要求
train/val/test及有效人工审核指纹，pilot不得声明test。未削弱正式检查。
见[实验报告](reports/blue_cube_pilot_v1.md)、[数据版本](datasets/blue_cube_pilot_v1/README.md)。
实际20轮早停；conf=0.25未检出验证/定性图中的可见目标，不代表模型已可用。
上文“尚无人工标签/未训练”是11月9日基线状态，本段为11月10日新增结果。
