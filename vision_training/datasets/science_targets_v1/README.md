# 正式三类别数据集 science_targets_v1

2026-11-11，基线 `bfa7425`。数据管理、标注与采集的操作见
[采集标注指南](../../SCIENCE_TARGETS_GUIDE.md)。现在可以开始三类别 Bounding Box 标注。

类别顺序固定：`0 color_block`（六色 5 cm 立方体统一类别）、`1 rock_compact`、
`2 rock_spiky`。两种岩石尺寸和材质 `Unknown / needs measurement`。

目前只迁移了历史 20 张蓝色立方体人工标注；没有正式岩石图、没有生成虚假标签，
没有正式 train/val/test、没有启动正式训练。历史框 ID 0 不变，名称映射为 color_block；
不能因此推断其他颜色已覆盖。新池中的框仍须复核三类别是否完整，`reviewed=false`。

[legacy_mapping.yaml](legacy_mapping.yaml) 记录逐图原视频、源帧号、原 pilot split、
图片/标签哈希与映射。原 blue_cube 和 pilot 图片、标签、划分均保留不变。
这 20 图来自同一实验室场景，正式版整体归 `legacy_lab_session_20261108`，
不沿用原 pilot 的视频级 14/6 划分。建议只放正式 train；另采独立 val/test 场景。

本机 `pool/images/`、`pool/labels/`、`pool/manifest.json` 及生成数据被 Git 忽略，
这里只提交类别、版本和可追溯映射；clone 后必须准备原始媒体。重建迁移：

```bash
# 仓库根目录执行；须先按原 pilot README 恢复其20图/标签。
# 已有新池时不要重复运行；迁移工具拒绝覆盖。
vision_training/.venv/bin/python vision_training/tools/migrate_science_targets.py
```

继续采集用独立会话 ID 与场景，添加到本版本尚未发布的数据池；审核/划分形成冻结快照后，
修改数据需建立新版本，不覆盖已发布 split 或借用旧审核回执。
