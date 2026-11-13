# 科学目标视频送标交付（2026-11-13）

来源：本地实际产物、完整重新解码核验、6项离线测试及三张视频总览抽查。
开始前fetch远端：feature/science-targets-perception 与本地均为971781db1e2fb808eec4019d1f1c913871739d24。
用户在本轮再次确认真实手机拍摄、不同场景；source_type=phone_real，分别记录capture_group，未建立正式split。

| 视频 | 完整解码 | 保留 | 非候选（采样间隔） | 重复排除 | 原尺寸 |
|---|---:|---:|---:|---:|---|
| A | 3743 | 444 | 3299 | 0 | 960×544 |
| B | 2808 | 339 | 2469 | 0 | 960×544 |
| C | 2145 | 260 | 1885 | 0 | 1280×720 |

共1043张，JPG质量97。自适应间隔0.25/0.30/0.40秒；全图差异与32像素局部块门槛、较长间隔pHash候选复核共同保守筛选。清晰度只记录，不单独拒绝图片。
重复筛选实际排除0张，不把采样间隔略过的7653帧称为重复；不声称相似度识别了目标姿态。
623张带低清晰度指标标记，部分明显模糊或无法辨认，交由人工标为uncertain。亮度全图指标未触发low_light标记，不代表不存在局部暗目标。
总览抽查可见远处小目标、灰色岩石、边缘截断、多目标同框和变化视角；没有逐图确认目标身份或量化覆盖率，也未见需要进一步大幅去重的证据。
岩石类别必须根据实物外形人工确认，不能靠此报告总览判定真值。

## 本地交付位置

以仓库根目录为基准：

- 原视频：`vision_training/video_sources/science_targets_20261113/`，保留原文件/链接与SHA256，不重编码。
- 数据根：`vision_training/datasets/science_targets_video_v1/`。
- `images/video_A/`、`video_B/`、`video_C/`：原尺寸图片，稳定文件名。
- `manifests/images.json`：逐图原视频哈希、源帧、时间、清晰度、前一保留帧相似度、原因、capture_group、reviewed=false。
- `manifests/video_*_timeline.jsonl`：每一帧的索引、时间、采样/排除原因，保留完整8696帧时间线。
- `cvat_packages/batch_001.zip`至`batch_006.zip`：174/174/174/174/174/173张，仅图片，不含标签。
- `cvat_packages/manifest.json`：每包数量、哈希与逐图对应；同目录`batch_*_review.csv`为人工审核表。
- `contact_sheets/`：各批代表拼图和所有图片分页拼图；本报告目录提交6张代表拼图与3张视频总览。

现有账户限制未知，200只是配置偏好。若CVAT拒绝当前数量，可不改名重新分小包：

```bash
vision_training/.venv/bin/python vision_training/tools/extract_training_frames.py package \
  --dataset vision_training/datasets/science_targets_video_v1 --batch-size 100 \
  --output vision_training/datasets/science_targets_video_v1/cvat_packages_100
```

## CVAT人工标注及导回

1. 创建Detection任务，按顺序添加 `color_block`、`rock_compact`、`rock_spiky`（ID 0/1/2），上传batch_001.zip。按账户实际限制调整包大小。
2. 所有可辨认正式目标分别画框，边缘目标画可见部分；六色方块统一类0，普通岩石不标成科学目标。
3. 编辑对应review.csv：完整标注为positive，人工确认无目标为negative，无法辨认/区分岩石为uncertain，其余pending。未标注不等于负样本。
4. 导出Ultralytics YOLO Detection或YOLO 1.1 annotations ZIP，可以不导出images。保持原文件名及三类顺序。负样本需要对应空标签文件。
5. 将ZIP保存在本地，用现有导入器按文件名匹配：

```bash
vision_training/.venv/bin/python vision_training/tools/import_cvat.py \
  --dataset vision_training/datasets/science_targets_video_v1 \
  --archive /实际路径/batch_001_annotations.zip \
  --review-csv vision_training/datasets/science_targets_video_v1/cvat_packages/batch_001_review.csv \
  --reviewer 用户姓名
```

pending/uncertain跳过，不进入正式训练。先完成第一批再讨论混合训练；本轮没有训练、标签生成或split。未来保留synthetic/phone_real等来源，按真实会话/场景划分，独立真实test不能用合成图代替。去重图片不能作为连续漏检时长评估序列，应读取原视频和完整时间线。

## 验证与限制

`verification.json`记录1043张全量源帧内容匹配、8696帧解码、ZIP CRC/哈希/集合完整、原尺寸、图片可读、无重名和跨包重复。6项测试包含局部小目标变化、完整时间线、重新分包、来源确认、共享采集组及仅标签CVAT导入。pytest首次受ROS自动插件缺少lark影响；设置PYTEST_DISABLE_PLUGIN_AUTOLOAD=1后6项通过，未安装依赖。

原600张合成数据没有重生成或覆盖，manifest哈希与Git已有batch600_quality一致（见synthetic_preservation.json）；本轮只读该目录，没有写入。没有训练、启动ROS/GO2、改模型架构、HSV、MAP、导航或抓取。真实D435泛化与正式训练效果：NOT VERIFIED。
