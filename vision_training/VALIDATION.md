# 2026-11-09 工作流验证记录

来源：当前checkout静态检查、PC实际命令输出、人工查看历史录像抽样；不涉及机器人运动。
基线从远端`feature/d435-perception-workspace`的
`e15c66ce2beb998ef9c73464b0bff72cbd4e2266`创建。新分支为`feature/reusable-yolo-training`。

## 环境与安装

| 项目 | 实测 |
|---|---|
| 系统 | Ubuntu 22.04，Linux 6.8.0-138-generic，x86_64 |
| GPU | NVIDIA GeForce RTX 4060 Laptop GPU，8188 MiB |
| 驱动 | 595.99.02；nvidia-smi显示CUDA兼容信息13.2 |
| Python | /usr/bin/python3 3.10.12，独立venv，无system-site-packages |
| torch/torchvision | 2.5.1+cu124 / 0.20.1+cu124，实际runtime12.4 |
| CUDA | is_available=True，GPU矩阵运算与CPU结果相同，synchronize通过 |
| Ultralytics | 8.3.40，官方yolo11n.pt可下载/加载 |
| NumPy/OpenCV | 1.26.4 / opencv-python4.10.0.84（cv2版本4.10.0） |
| ONNX/runtime | 1.17.0 / 1.20.1，第一阶段CPU ONNX推理 |
| pip check | No broken requirements found |

未修改驱动、系统CUDA或系统Python依赖。venv路径为vision_training/.venv。
完整已解析版本见requirements.lock.txt；精简根依赖见requirements.txt。
环境摘要、模型哈希与ROS用例结果见validation_summary.json。

**已观察到的环境限制**：安装/导入期间曾出现间歇Python段错误、ZIP时间戳解析异常、
Matplotlib导入TypeError/SystemError；有些错误发生于pip/标准库，未进入训练逻辑。
最初shell继承ROS PYTHONPATH，后已清除；pip22升级24.3.1，完整wheel从保留下载目录
恢复安装。使用`--no-compile --no-warn-conflicts`安装后单独pip check通过。
隔离PYTHONPYCACHEPREFIX后有成功导入，但后续仍出现间歇异常，不能认定缓存为唯一根因。
双类别训练、导出、ONNX ROS检查使用CPU亲和16–19及OMP/OPENBLAS单线程完成；
单类别GPU训练、GPU评估、PT ROS检查曾在未限制亲和时完成。
这些是实际测试条件，不是问题已修复的证明；宿主解释器/系统长期稳定性仍待排查。
未修改系统文件、未添加自动重试隐藏失败，也未声称需要特定硬件修复。

## 软件与实际流水线

- 新增数据/通用配置/深度几何检查：16 passed。涵盖单/多类别同一training_plan、
  非法标签/出界框、背景空标签与缺失标签、孤儿标签、跨split图片重复、审核哈希失效、
  拍摄组互斥、真实import→review→split过程、模型类别次序、深度空洞/混层/边缘/反投影。
- 旧HSV/搜索/接近针对性回归：53 passed，使用原系统Python并source Humble+deploy_ws。
  最初漏source deploy_ws的一次检查异常退出，正确环境重跑53项通过。
- 修改文件flake8、git diff --check通过；只build go2_science_perception成功，
  /tmp/yolo_ros_install，未做全仓库build。旧HSV源码、配置和launch未改。
- 正式ros2 launch入口以venv解释器加载PT导出包通过；12秒限时SIGINT退出，进程cleanly
  finished。首次检查暴露重复SIGINT打断清理，已在新节点finally中屏蔽后续SIGINT，
  包级重建与入口复测通过；timeout工具自身返回124是到时退出标记，不是模型加载失败。
- 当前blue_cube训练被缺少已审核标签/有效split的前置检查阻止；这是预期状态，
  不是蓝色模型训练成功。没有制造空标签/HSV伪标签绕过审核。

**通用训练实际检查**：仅为工程测试，以程序生成的确定性矩形及其精确框创建两个
独立合成fixture（1类fixture_A、2类fixture_A/fixture_B），各8 train/4 val/4 test图。
合成测试回执明确标记`SYNTHETIC_UNIT_TEST_GEOMETRY_ONLY`，不对应任何真实图片的人工
审核；不写入blue_cube数据集。两份YAML调用完全相同train.py，各1 epoch、imgsz64、
batch2、device0、workers0、amp=false。均完成训练并有weights/best.pt、last.pt、
args.yaml、results.csv/png、config/data/environment快照与训练结果JSON。
测试指标为0，不作为实际识别性能；不交付或命名成蓝色立方体模型。

evaluate.py实际检查：单类别fixture的4张test图，输出P/R/mAP50/mAP50-95、样例、
逐图速度与汇总；另用明确名为fixture_subset的2张子集检查subset入口。
GPU全test的batch1 wall均值约3.12ms；CPU子集约26.18ms中位。
这里只证明计时/指标输出链执行，不推断实际D435速度或场景精度。

export.py实际检查：

- 官方COCO80类YOLO11n：PT复制导出、ONNX FP32静态batch1/imgsz640/opset17成功；
  ONNX checker、实际ONNX推理、权重内类别与metadata类别一致。
- 双类别fixture权重：ONNX imgsz64导出后保持fixture_A/fixture_B，未硬编码COCO或蓝色类。
- 模型/输入大小/hash/version/inference参数记录于每个metadata.json。

## ROS2实际消息检查

`tools/ros_smoke.py`强制loopback、ROS_DOMAIN_ID=78，不连接GO2。
官方bus.jpg作为已知可检测输入，CameraInfo和深度为合成数据，非真实D435验收。
以下两次运行实际加载真实模型（未mock检测框），每次5类检查：

| 输入/检查 | 官方PT，选择全部 | 导出的ONNX，仅选择bus |
|---|---:|---:|
| 合格同步深度 | 5个2D、5个3D | 1个2D、1个3D |
| 全0无效深度 | 5个2D、0个3D | 1个2D、0个3D |
| RGB/Depth时差不合格 | 5个2D、0个3D | 1个2D、0个3D |
| optical frame不一致 | 5个2D、0个3D | 1个2D、0个3D |
| 缺少新的深度 | 5个2D、0个3D | 1个2D、0个3D |

每个结果含原RGB时间戳；debug Image与点header关联同一RGB。检查原HSV正式点话题
和cmd_vel_raw无发布者。PT五份样本predict wall约21.6–33.4ms，受线程/负载条件影响，
不等于持续FPS。ONNX时类别筛选来自配置，metadata哈希/实际names核对成功。
真实D435订阅、真实深度/多物体定位精度、连续长期速率仍NOT VERIFIED。

如要复现上述消息检查，先source Humble和/tmp/yolo_ros_install，再设置隔离缓存：

```bash
export PYTHONNOUSERSITE=1
export PYTHONPYCACHEPREFIX=/tmp/yolo_fresh_pycache
export YOLO_CONFIG_DIR=/tmp/yolo_settings
export MPLCONFIGDIR=/tmp/yolo_mpl_cache
vision_training/.venv/bin/python vision_training/tools/ros_smoke.py \
  --weights vision_training/weights/yolo11n.pt \
  --image vision_training/validation_local/bus.jpg \
  --output /tmp/yolo_ros_check.json --device cpu
```

测试图片、权重、训练结果与完整报告仅保留本机validation_local/、weights/及/tmp，
不将合成标签导入正式项目，不提交大模型/媒体。

## 首个真实项目与待办

10份原始RGB视频共207.8秒，提取213张，全部UNREVIEWED。
本轮未训练blue_cube模型，无真实P/R/mAP或困难分组结果。
需要人工标注、独立拍摄组划分和必要场景补充，详见datasets/blue_cube/README.md。
最长连续漏检时间/实际目标检出率、TensorRT engine和GO2 Orin/NX部署均NOT VERIFIED。
