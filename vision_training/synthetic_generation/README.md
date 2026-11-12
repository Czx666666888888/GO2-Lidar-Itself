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
第六色只保留中性占位材质，`confirmed=false`，不渲染、不采样；确认后在 YAML 填 RGBA 即可。
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

## 小批量与600张

```bash
vision_training/synthetic_generation/scripts/run.sh \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/datasets/science_targets_synthetic_preview_ground_checked \
  --per-class 4 --allow-unverified-scale
vision_training/synthetic_generation/.venv/bin/python \
  vision_training/synthetic_generation/scripts/check_dataset.py \
  vision_training/datasets/science_targets_synthetic_preview_ground_checked \
  --preview vision_training/synthetic_generation/previews/contact_sheet.jpg
```

查看 RGB 与叠框图，确认形状/俯视/颜色/落地/标签。只有实际查看通过后，才在预览目录的
`quality.json` 中记录 `visual_review: "PASS"` 及审核说明。不得仅凭自动检查填写视觉通过。
大批次要求预览配置完全一致、自动与视觉检查均 PASS。目标输出目录已存在则拒绝覆盖。

```bash
vision_training/synthetic_generation/scripts/run.sh \
  --config vision_training/synthetic_generation/configs/default.yaml \
  --output vision_training/datasets/science_targets_synthetic_v1 \
  --per-class 200 --allow-unverified-scale \
  --validated-preview vision_training/datasets/science_targets_synthetic_preview_ground_checked
vision_training/synthetic_generation/.venv/bin/python \
  vision_training/synthetic_generation/scripts/check_dataset.py \
  vision_training/datasets/science_targets_synthetic_v1
```

每类200表示以该类为主目标的200张，总计600；多目标图可能同时含其他类，因此每类实例数
会超过200。每摆放场景2视角，同场景全入同split；默认每5组1组val，其余train。
预览每类仅2场景，其train/val不用于评估。最后test必须使用独立真实D435数据。

## 场景、相机与标签

灰地面，0–6°斜坡；程序生成变形低细分icosphere普通岩石/小障碍物，始终背景，
不使用特殊STL充当负样本。目标底部最低网格顶点接触地面，XY保守包围圆间隔防止交叠。
距离0.3–0.8 m、水平角全周、俯视25–65°、偏置视线、目标yaw、目标数、灰度、微表面、
面积光方向/能量与弱环境光随机变化；面积光形成局部阴影，避免随意改变目标形状/色调。
未模拟运动模糊、传感器噪声、机器人机体遮挡和真实洞壁扫描；域差异需用真实数据验证。
遮挡通过视线投影自然产生；极端遮挡和过小目标丢弃整帧，不留下部分漏标的负样本。

相机默认 `synthetic_camera`，640×480、临时 K；不是D435标定结果。
可将 `camera.camera_info` 指向真实、已整流的 CameraInfo YAML（支持ROS小写k或标定文件
camera_matrix.data）。非零畸变会拒绝，不能静默丢弃畸变；来源原文写入manifest。
不要使用其他型号相机内参冒充D435。本轮未连接相机。

BlenderProc实例分割生成可见像素框（半开区间xyxy）；背景类别0，内部科学类别1–3，
输出YOLO时映射为0–2，不丢掉color_block。每个目标单独渲染作为同视角未遮挡轮廓，
visible/full-in-frame计算可见率（不代表画面外截断比例）；默认≥80像素、可见率≥0.3。
若任意目标不满足阈值，整帧丢弃，最多32次尝试，不足就报错保留部分输出且不写COMPLETE。

输出包含 `images/{train,val}`、`labels/{train,val}`、`masks/{train,val}`（实例npy）、
`metadata/{train,val}`、`manifest.json`、`data.yaml`、`COMPLETE`。
manifest保留场景ID/类别/实例ID/模型hash/源路径/比例/位姿/相机K和位姿/光照/坡度/随机种子。
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

验证结果与仍未验证项见 `REPORT.md`。
