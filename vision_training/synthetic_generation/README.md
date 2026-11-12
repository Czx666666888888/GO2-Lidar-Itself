# 科学目标 BlenderProc 合成数据

仅准备模型和 RGB YOLO Detection 数据，不训练、不连接 GO2、不修改 ROS2。
下列命令均从仓库根目录执行。
类别固定 `0 color_block / 1 rock_compact / 2 rock_spiky`。六色均为类0。

## 原始资产与尺寸

本机源目录 `/home/czx666/科学目标发布/`，配套 `科学目标截图.docx` 图1为紧凑岩石，
图2为尖峰岩石；发布 ZIP 包中也包含这两份 STL。原件只读，复制到 `assets/`，不进入 Git。
新机器请手动复制 `科学目标1.stl` 和 `科学目标2.stl` 到此目录；缺失时拒绝生成，不替代模型。
`assets/audit.json` 记录原始文件 SHA256、顶点/三角面、包围盒、拓扑与异常面检查。

| 文件 | 唯一顶点 / 三角面 | 原始坐标包围盒尺寸（单位未知） |
|---|---|---|
| 科学目标1.stl | 4161 / 8318 | 0.01715599 × 0.02146815 × 0.01907337 |
| 科学目标2.stl | 25903 / 51802 | 0.03961477 × 0.03329314 × 0.03273641 |

两模型按合并相同坐标顶点后的拓扑计算均封闭、面绕序一致；零面积、重复面、非有限坐标为0。
两模型STL内法向均无零值/非有限值、与几何法向无反向或超过1°的偏差；体积朝向为正。
目标2有8个面积≤1e-12原始单位平方的小面，不能称为零面积面；自相交 `NOT VERIFIED`。
原件不修复。导入仅平移原点到几何 XY 中心/底面以方便摆放，记录平移与导入前后包围盒。
`rock_compact_scale`、`rock_spiky_scale` 独立配置，默认1.0。
**STL未声明单位，默认坐标当 Blender 米使用只是渲染约定，不证明实物尺寸。**
先测量道具，再按“实测尺寸米 / 对应原始轴尺寸”调整，确认后更新
`physical_scale_confirmed`。`--allow-unverified-scale` 明确允许保留原比例的开发数据。

## 独立安装

不 source ROS，不复用或重装训练 venv。要求 Linux x64、足够磁盘与网络。

```bash
python3 -m venv vision_training/synthetic_generation/.venv
vision_training/synthetic_generation/.venv/bin/pip install -r vision_training/synthetic_generation/requirements.txt
mkdir -p vision_training/synthetic_generation/runtime
curl -L --fail -o vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64.tar.xz \
  https://download.blender.org/release/Blender4.2/blender-4.2.1-linux-x64.tar.xz
tar -xf vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64.tar.xz \
  -C vision_training/synthetic_generation/runtime
```

首次先安装 Blender 自带 Python 的独立依赖（固定NumPy1.26，避免旧二进制库ABI冲突）：

```bash
env PYTHONPATH= PYTHONNOUSERSITE=0 \
  PYTHONUSERBASE="$PWD/vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/custom-python-packages" \
  vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/4.2/python/bin/python3.11 \
  -m pip install --user --upgrade \
  --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
  -r vision_training/synthetic_generation/requirements_blender.txt
```

此步骤需要联网，只安装到项目内运行时。BlenderProc首次启动可能还会检查pip。
若网络失败后报缺包，用 `blenderproc run --force-pip-update` 重建实际包列表缓存，
不要把包缓存当作安装成功证据。运行包装器清除 ROS PYTHONPATH，默认使用 CPU。
底层渲染及分割使用 [BlenderProc 官方 API](https://dlr-rm.github.io/BlenderProc/blenderproc.api.renderer.html)。

## 立方体、材质与配置

`build_block.py` 用 Blender API 创建一个共享基础几何：边长0.05 m，应用缩放后倒角0.0005 m。
六个独立材质数据块：red、green、blue、yellow、natural_wood、color_6_pending。
粗糙度0.65、低强度微表面木纹；颜色为开发近似值，并非实物色度测量。
第六色只保留中性占位材质，`confirmed=false`，不渲染、不采样；实际确认后仅在 YAML 将pending键替换为实际颜色名称和RGBA。
原木微纹理不使用夸张贴图。材质图只展示已知五色。

```bash
vision_training/synthetic_generation/.venv/bin/python \
  vision_training/synthetic_generation/scripts/audit_assets.py \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/synthetic_generation/assets/audit.json
vision_training/synthetic_generation/.venv/bin/blenderproc run \
  --custom-blender-path vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64 \
  vision_training/synthetic_generation/scripts/inspect_import.py \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/synthetic_generation/assets/blender_mesh_audit.json
vision_training/synthetic_generation/.venv/bin/blenderproc run \
  --custom-blender-path vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64 \
  vision_training/synthetic_generation/scripts/build_block.py \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/synthetic_generation/assets
```

## 当前改进版与最小预览（本轮未执行渲染）

现有600张属于 `0014177` 版本，其实际配置保存在本地数据集manifest；
当前default.yaml是新采样候选，不代表已生成/已验收的新数据。
本轮只做离线测试及Blender几何/失败路径检查，没有重新渲染、导入池或训练。
审查与测试证据见 [GENERATOR_REVIEW.md](GENERATOR_REVIEW.md)。

未来由操作者执行的最小预览（6张，三个类别各2张，单场景预览不可用于泛化评估）：

```bash
vision_training/synthetic_generation/scripts/run.sh \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/datasets/science_targets_synthetic_v2_preview \
  --per-class 2 --allow-unverified-scale \
  --distance-band near --image-region center
env PYTHONPATH= PYTHONNOUSERSITE=0 \
  PYTHONUSERBASE="$PWD/vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/custom-python-packages" \
  vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/4.2/python/bin/python3.11 \
  vision_training/synthetic_generation/scripts/check_dataset.py \
  vision_training/datasets/science_targets_synthetic_v2_preview \
  --preview /tmp/science_targets_v2_preview.jpg
```

输出目录存在就拒绝覆盖。CLI的距离段/区域限定会写入有效配置，不是隐藏的采样覆盖。
这个near/center预览只检查最小链路，不能为mixed bands/regions的大批次提供验收。
未来必须先按最终有效配置完成小批量RGB/Mask目视验收，再在对应quality.json实际记录
visual_review=PASS；大批次仍要求预览配置、manifest哈希及所有工具代码哈希完全一致。
**不得使用旧600张/旧预览的质量记录批准新采样配置；本轮不启动新批次。**

距离段的range_m和weight、七种区域的weight均可改。weight=0关闭对应候选；
当前权重只是候选分布，不是每类成功数量保证。视角一旦选定距离段和区域，重试不切换；
按字面米制距离采样，不再按目标尺寸自动放大距离。若目标过小或摆放无法满足要求，
最多重试32次后失败，写manifest/failure.json和拒绝原因，不写COMPLETE；不降门槛补数量。
在实物尺寸未确认前，medium/far可能不可用，应依据尺寸/像素证据调整权重或范围。

## 不渲染的检查

复用已有独立Blender环境，本轮未安装新依赖。稳定支撑使用其已有SciPy；
host侧requirements也声明同版SciPy，未重装现有venv。检查新姿态数据和运行完整测试时，
可直接使用已具备依赖的Blender Python（不启动渲染器）：

```bash
env PYTHONPATH= PYTHONNOUSERSITE=0 \
  PYTHONUSERBASE="$PWD/vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/custom-python-packages" \
  vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64/4.2/python/bin/python3.11 \
  -m unittest discover -s vision_training/synthetic_generation/scripts -p 'test_*.py'
```

以下BlenderProc命令也均不调用渲染器：

```bash
vision_training/synthetic_generation/.venv/bin/blenderproc run \
  --custom-blender-path vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64 \
  vision_training/synthetic_generation/scripts/check_geometry.py \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --report /tmp/science_sampling_geometry_check.json
vision_training/synthetic_generation/.venv/bin/blenderproc run \
  --custom-blender-path vision_training/synthetic_generation/runtime/blender-4.2.1-linux-x64 \
  vision_training/synthetic_generation/scripts/check_generator_guards.py \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --report /tmp/science_sampling_guards.json
```

## 场景、相机与标签

灰地面，0–6°斜坡；程序生成变形低细分icosphere普通岩石/小障碍物，始终背景，
不使用特殊STL充当负样本。目标底部最低网格顶点接触地面，XY保守包围圆间隔防止交叠。
距离分near 0.4–0.8 / medium 0.8–1.5 / far 1.5–2.5 m；水平角全周，
相机实际光轴俯视仍限制25–65°（偏置后再检查），目标yaw、目标数、灰度、微表面、
面积光方向/能量与弱环境光随机变化；面积光形成局部阴影，避免随意改变目标形状/色调。
未模拟运动模糊、传感器噪声、机器人机体遮挡和真实洞壁扫描；域差异需用真实数据验证。
图像区域：center、left、right、top_edge、bottom_edge使用归一化目标中心位置范围；
near_edge_full按实际网格投影边缘迭代定位，距所选边4–20像素且完全在画内；
partial_outside沿随机边将投影框跨度的10–35%放在画外，并要求单目标Mask实际触边且有可见像素。
这里的出画比例是框跨度比例，不是物体面积比例。归一化u向右、v向下。
投影约束不合格就重试；检测框始终仅来自画面内可见Mask，不扩展到画外。

特殊岩石使用Trimesh稳定放置候选，rock_poses分别配置pose_ranks/min_probability/max_poses。
默认采用均匀密度重心假设，不代表实物重心已测量。姿态组合为坡面旋转×面内yaw×稳定姿态；
精确接地后检查支撑多边形、重力方向重心投影及边界余量，不接受仅单点接触的悬摆姿态。
坡面不稳定的候选重新选姿态，预算耗尽报错；不改变STL比例、不改变几何或伪造实物姿态。
遮挡通过视线投影自然产生；极端遮挡/过小目标丢弃整帧，不留下部分漏标的负样本。

相机默认 `synthetic_camera`，640×480、临时 K；不是D435标定结果。
可将 `camera.camera_info` 指向真实、已整流的 CameraInfo YAML（支持ROS小写k或标定文件
camera_matrix.data）。非零畸变会拒绝，不能静默丢弃畸变；来源原文写入manifest。
不要使用其他型号相机内参冒充D435。本轮未连接相机。

BlenderProc实例分割生成可见像素框（半开区间xyxy）；背景类别0，内部科学类别1–3，
输出YOLO时映射为0–2，不丢掉color_block。每个目标单独渲染作为同视角未遮挡轮廓，
visible/full-in-frame计算可见率（不代表画面外截断比例）；默认≥80可见像素、可见框短边≥12像素、可见率≥0.3。
若任意目标不满足阈值，整帧丢弃，每个固定视角请求最多32次尝试，不足就报错保留部分输出且不写COMPLETE。

输出包含 `images/{train,val}`、`labels/{train,val}`、`masks/{train,val}`（实例npy）、
`metadata/{train,val}`、`manifest.json`、`data.yaml`、`COMPLETE`。
manifest保留场景ID/类别/实例ID/模型hash/源路径/比例/位姿/相机K和位姿/光照/坡度/随机种子，
新增距离段/区域请求、未裁剪投影框、稳定姿态rank及支撑检查、拒绝记录、有效配置SHA256、
所有非测试Python工具代码哈希；CameraInfo导入保留原文与文件hash。
可选depth为米制渲染深度，不是实际D435测量。
自动检查逐图从mask重新计算BBox并核对YOLO、像素数、可见率及跨集合场景泄漏。
另核对保守XY间隔；原STL可访问时，从原始顶点和记录的变换独立重建世界坐标，
验证最低点到坡面的距离（不使用放置函数的自报结果代替此项检查）。

## 导入现有训练池（不训练）

```bash
vision_training/synthetic_generation/.venv/bin/python \
  vision_training/synthetic_generation/scripts/import_pool.py \
  --source vision_training/datasets/science_targets_synthetic_v1 \
  --dataset vision_training/datasets/science_targets_v1
```

导入要求COMPLETE、标签检查通过、类别一致、无覆盖，保留每场景group与synthetic_split。
记录 `source_type=synthetic`、完整sidecar和来源hash；`reviewed=false`，不冒充人工审核。
`phone_real`、`d435_real` 是其他独立来源；合成导入不改它们或历史标签。
本工具不改既有split、不自动执行review、不调用训练；未来统一划分须保留synthetic_split，
合成数据不得进入最终test。

旧600张结果见 `REPORT.md`；本轮改进的测试和未验证边界见 `GENERATOR_REVIEW.md`。
