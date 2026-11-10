# blue_cube_pilot_v1（20张人工标注的小样本试训练）

来源：用户CVAT人工标注；本地`/home/czx666/下载/First 30.zip`，Ultralytics YOLO Detection。
ZIP及每张图片/标签哈希、原视频/帧号、人工标签原文和split见[version.yaml](version.yaml)。
不使用ZIP中列举181张图的train.txt；只按20份实际标签的stem精确匹配图片。
20图/20标签/20个class 0 blue_cube框，坐标、可解码性、配对、跨split哈希检查通过。
助手检查20张标注叠框图；可见框对应实际立方体，包括上缘部分出画目标，未修改人工标签。
本地审核回执区分用户人工标注和助手视觉核查，不声称用户审查了本次split。

| 原视频（2026-11-08） | 数量 | 用途 |
|---|---:|---|
| approach_20261108_211245_917119_raw.mp4 | 2 | train |
| approach_20261108_222156_798690_raw.mp4 | 6 | train |
| approach_20261108_222516_034701_raw.mp4 | 6 | train |
| approach_20261108_222842_367393_raw.mp4 | 6 | val |

合计14 train / 6 val；整段视频分组，相邻帧未跨split，没有test。
验证视频含小目标及上边缘截断，划分在训练前确定，未按结果重新划分。
这些视频原属同一lab_floor场景组；本次是用户授权的按视频pilot例外，不能视为独立场景验证。
正式数据集仍需按拍摄会话划分train/val/test，本次不改变正式审核规则。
原annotation_v1和181张图片不变，161张未标注图片未复制到训练/验证或伪装负样本。

本机图片、标签、审核回执、完整version.json及label_review.jpg留在此目录；媒体与JSON忽略。
version.yaml保存20条标签原文，以便核对版本；原ZIP无需上传。
重建时必须使用新输出目录，工具拒绝覆盖已有版本：

```bash
vision_training/.venv/bin/python vision_training/tools/prepare_pilot.py \
  --archive '/home/czx666/下载/First 30.zip' \
  --source vision_training/datasets/blue_cube/annotation_v1 \
  --output vision_training/datasets/blue_cube_pilot_v1 \
  --val-video approach_20261108_222842_367393_raw.mp4
```

首次准备后必须实际查看label_review.jpg再记录审核回执；工具不会自动签署人工审核。
现有版本已完成该步骤。更新数据用v2，禁止覆盖v1或借用v1的旧回执。
