# 合成数据实测记录（2026-11-12）

## 范围与起点

远端分支 `feature/science-targets-perception` 最新Push经 `git ls-remote` 实查为
`91e11fec99cc2203eac40040049bdf9fe9f91e74`，与本地HEAD一致。
既有bridge/测试/10mm分析及用户手册改动保留，不纳入提交。
本轮不训练、不启动ROS/相机/机器人，不修改现有YOLO核心、检测节点或导航模块。

## 模型与环境实测

- 两份STL在本地 `/home/czx666/科学目标发布/` 找到，与发布ZIP逐字节相同；
  配套文档图片1为compact、图片2为spiky。原件未改，副本哈希见 `assets/audit.json`。
- 紧凑模型：4161唯一顶点、8318三角面；原始尺寸
  `0.017155990 × 0.021468148 × 0.019073366`（原始坐标单位未知）。
- 尖峰模型：25903唯一顶点、51802三角面；原始尺寸
  `0.039614772 × 0.033293140 × 0.032736409`（原始坐标单位未知）。
- 两模型均封闭、绕序一致、体积朝向为正；STL内法向无零值/非有限值，
  无反向或超过1°的偏差。零面积面、重复面、非有限顶点均0；尖峰模型有8个
  面积≤1e-12原始单位平方的小面。自相交 `NOT VERIFIED`；不自动修复原件。
- 两个scale=1.0，Blender米制解释是渲染约定；不能据此声称实物仅约2/4cm，
  实物尺寸仍 `NOT VERIFIED`。每次导入记录原/平移归一化后包围盒和原点平移。
  Blender自身导入顶点/三角面实测与原始审计一致，见 `assets/blender_mesh_audit.json`；
  预览manifest中的缩放/归一化包围盒独立核对见 `assets/blender_import_audit.json`。
- 立方体实建：一个共享基础网格，六个材质数据块，变体边长
  `0.050000000745 m`，倒角 `0.000500000024 m`；见 `assets/block_audit.json`。
- 五个已知色可渲染；第六色仅中性占位，confirmed=false，未渲染/未随机采样。
  颜色/粗糙度是开发近似，不是实物色度或反射率测量。
- 项目内独立venv与Blender自带Python：Blender 4.2.1 LTS / BlenderProc 2.8.0；
  CPU实际完成RGB与实例分割渲染。两个隔离环境 `pip check` 均无依赖冲突。
- 重复调用实例分割时BlenderProc输出同key/path登记提示；已核对官方Utility源码，
  该路径替换既有输出登记。实际图片ID、标签和mask对应关系另由质量工具逐图检查。
- 宿主NVIDIA驱动查询失败，未依赖GPU；首次默认源慢/网络失败已通过镜像安装并
  固定Blender侧NumPy1.26.4解决。初次失败日志保留在本地runtime。

## 小批量与回归

实际小批量发现并修复：过亮光照、主目标过小、多目标间距计算及坡面接地符号错误。
早期预览目录均为调试材料，不是通过批次，也没有进入训练池。

最终验收预览：`vision_training/datasets/science_targets_synthetic_preview_ground_checked/`。
12张（每类主目标4张），6个场景，train/val各6张；它不是有效泛化评估集。
逐张叠框/多视角图、放大坡面compact和spiky原图及五色材质图已目视检查。
独立原STL世界坐标接地检查18次全部通过；同一检查正确拒绝修复前的悬空目录。
自动检查与目视检查PASS及预览manifest哈希见 `previews/quality.json`。

4项离线测试通过：可见像素框/类0/背景排除；遮挡和不可见目标拒绝；
掩码校验/分场景/导入来源及防覆盖；错误框和跨集合场景泄漏拒绝。
Python编译、shell语法、diff空白检查通过；没有声称做过全仓build或ROS验证。

## 600张批次

运行入口与参数：README的600张命令；`--per-class 200 --allow-unverified-scale`，
先通过最终预览的自动/目视质量门和配置/生成器哈希一致检查后启动。
实际完成 **600张RGB + 600份YOLO标签 + 600份实例mask + 600份逐图metadata**。
主目标图片数 color_block / rock_compact / rock_spiky = **200 / 200 / 200**；
实际框数（多目标图）= **392 / 368 / 444**。
300个独立摆放场景，每场景2视角；**train=480，val=120，无test**。
600张自动掩码框/YOLO/来源/ID/集合泄漏检查全部通过；
原STL世界坐标接地核对 **812次** 通过，保守目标间距检查通过。
第六色占位采样数0，五个已知颜色均有样本；相机全部标为synthetic_camera。
最终不同类别/视角叠框拼图已抽查，不能称为600张全部人工逐图验收。
完整统计、参数范围和manifest哈希：`previews/batch600_quality.json`。
本地运行原始日志：`vision_training/synthetic_generation/runtime/batch600.log`。
manifest的git_commit是生成时工作区基线；新增生成器以generator_sha256锁定，
生成期间同一分支已有独立本地提交db5068c（其他任务的一键测试工具）；保留其历史，
本轮不改其代码，最终合成源码另作一次独立Git提交。

## 输出与限制

- 输出位置：`vision_training/datasets/science_targets_synthetic_v1/`；大文件不进Git。
- RGB + YOLO可见区域框 + 实例掩码 + 完整来源/位姿/相机/光照/场景manifest。
- 普通岩石为独立变形icosphere，不使用特殊STL作负样本；所有可辨认目标必须满足
  可见像素≥80、可见率≥0.3，否则整张丢弃，避免部分漏标。
- 每摆放2视角且同split；不随机拆相似视角。合成只train/开发val，最终test必须真实D435。
- 训练池导入只提供命令且通过临时目录集成测试；未实际修改science_targets_v1池，
  不替用户作人工审核，不混淆synthetic/phone_real/d435_real。
- 临时相机K标记 `synthetic_camera`；实际D435 CameraInfo导入、可选Depth、GPU渲染、
  实物尺寸/第六色/材质匹配、真实洞壁/传感器噪声/泛化效果全部 `NOT VERIFIED`。
- 所有主目标朝上放置且仅随机yaw和≤6°坡度，不覆盖翻倒道具；背景为简化灰地面/
  灰岩而非实测洞壁模型。可见率分母是同相机画面内的单目标轮廓，不估计画面外面积。
