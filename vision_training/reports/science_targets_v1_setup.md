# science_targets_v1 工程准备与验证

日期：2026-11-11。开始前联网 `git ls-remote` 并 fetch 确认
`feature/reusable-yolo-training` 最新 Push 为 `bfa7425cd47cbf2b8f690dbd23ce19a4710e28c8`；
新分支 `feature/science-targets-perception` 从该提交创建。未提交 bridge/10mm 工作保留，
不纳入本轮。最终 Push SHA 以 Git 历史及交付回复为准。

## 已实现

- 正式 0 color_block / 1 rock_compact / 2 rock_spiky 类别和版本元数据。
- 复用 pool、标签校验、人工审核、按组 split 和原训练框架；新增 CVAT Detection ZIP
  导入，支持多框、多类、名称映射及逐图 positive/negative/pending/uncertain。
  不按 train.txt 或缺失标签生成负样本，原始 CVAT 标签单独保留。
- 真实历史 20 图从原 pilot 哈希核对后复制；ID0 不变，blue_cube 映射到 color_block；
  逐图来源和哈希见 `datasets/science_targets_v1/legacy_mapping.yaml`。原标签不改写。
  整体归同一实验室组，新池 reviewed=false，三类完整性仍需人工检查。
- RGB-D 采集器复用 CvBridge、ROS sensor QoS 与现有三个相机流，新增简单有界缓存。
  RGB PNG / 原数值 Depth NPY / 原 CameraInfo / 独立时间戳与差值 / 会话和场景一起保存。
  不匹配的候选保留并明确标记，不冒充同步真值；RGB 可单独导入标注池，保留 sidecar。
- 正式训练配置：YOLO11n / 640 / GPU0 / batch4 / epochs100 / patience30；初始 nbs4
  仅是无累积起点；预算工具按已审核 split 的实际图数检查批次数、类别覆盖和预热。

## 本轮实测与证据等级

| 检查 | 结果 | 能证明什么 |
|---|---|---|
| 三类别与历史数据迁移 | 20 图/20 标签逐文件匹配原 pilot 哈希，类0映射，原数据保留 | 本地真实数据复制和版本可追溯；不证明新三类标签已审完 |
| 针对性 pytest | 34 passed | 含原 workflow 及 yolo_geometry 回归；新增多框/多类/负样本/缺失标签、classic CVAT alias、采集导入、深度无损保存与时间戳关联检查 |
| flake8 | 修改/新增 Python 文件，max-line-length100，通过 | 静态代码风格，不代表真实传感器运行 |
| 正式配置/预算入口 | 预期拒绝：images/train 尚不存在 | 尚未准备正式 split，不能提前训练 |
| 原 V2 配置 | training_plan(..., pilot=True) 解析20图与审核回执通过，nbs4 | 原训练框架兼容；没有重跑训练 |
| 源码检查 | 原 detector 从权重取类名、循环多框，JSON 按目标含 class_id/name/框/xyz；3D沿 RGB header | 三类权重加载路径是通用实现；没有正式三类权重的实测推理 |
| 采集 ROS 节点 | 独立本机域211，两秒后自动结束，samples=0 | Node 启动/订阅创建/退出/summary 通过；不证明收到相机数据 |
| USB 枚举 | /sys/bus/usb/devices 未见 Intel D435 8086:0b07 | 本机相机不在线；未连接机器人获取相机 |

pytest 实际命令（仓库根目录）：

```bash
PYTHONPATH="$PWD:$PWD/camera_ws/src/go2_science_perception" \
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 taskset -c 16-19 \
vision_training/.venv/bin/python -m pytest -q \
  vision_training/tests/test_workflow.py vision_training/tests/test_science_targets.py \
  camera_ws/src/go2_science_perception/test/test_rgbd_capture.py \
  camera_ws/src/go2_science_perception/test/test_yolo_geometry.py
```

CPU affinity 是已有本机运行条件，不是其他机器通用依赖。首轮新增 classic CVAT 测试的
JSON 键类型断言失败，修正为字符串键后上述34项全部通过；导入映射逻辑未据此改写。
venv 没有 flake8，使用系统 `/usr/bin/python3 -m flake8` 通过，未重装环境。
最初沙箱 DDS socket 受限，隔离本机域重跑后无该错误；最终启动输出路径为
`/tmp/science_targets_capture_smoke/rgbd_20261111T125112_247872Z`，仅0帧启动证据。

## Warmup / batch 结论

核对本机固定版本 Ultralytics8.3.40 的 `engine/trainer.py` 与 `data/build.py`：
单GPU真实 batch=min(configured_batch,N)，nb=len(train_loader)；预热至少100batch；
初始累积max(round(nbs/configured_batch),1)，预热中插值。源码未修改。
V2 已有14训练图的 nb4/100预热边界和313次optimizer.step是历史报告，本轮不重跑。
正式数据当前仅20历史池图、没有分配 train，**正式实际 nb 和 optimizer.step 均未知**。
新工具只报告规划值，AMP跳步和早停须以后实际记录，不能把上限当作实测更新次数。

## 仍未验证与下一步

真实 RGB/Depth/CameraInfo 连续采集、匹配时差分布、物理对齐精度和长期吞吐：
`NOT VERIFIED`。本轮读回测试使用小数组，明确只是格式/时间配对单元检查，没有假装硬件数据。
三类训练、三类权重加载实测、真实 D435 检测、多场景识别与粗定位准确性：`NOT VERIFIED`。
未运行全仓构建或正式训练，未启动任何导航/运动节点；未修改 HSV、yolo_detector、
yolo_geometry、MAP 或接近控制。岩石尺寸/材质仍未知。

用户可以立即按指南建立三类 CVAT 标注任务；先采六色、两种岩石、普通岩石负样本和
真实补光弱光，独立场景组成 train/val/test，完成漏标与负样本审核后再核对训练预算。
