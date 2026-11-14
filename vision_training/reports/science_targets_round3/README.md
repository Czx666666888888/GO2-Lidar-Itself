# 第三轮科学目标候选数据与CVAT交付

2026-11-15。开始远端/本地均为cd6dc9cfac6ec4bc36c406313bded86a9267c2cc。
本轮只做数据审计、离线V2辅助筛选和图片打包；没有训练V3、覆盖权重、启动ROS或修改机器人控制。

## 原视频审计

四段指定视频均在本机微信视频目录找到，逐字节复制至`vision_training/video_sources/science_targets_round3/`，原件不动；复制前后及工作结束hash核对一致。
用户确认“手机不同场景”，并再次确认第2/3/4段为“不同地点或独立拍摄会话”；准确地点/时间未提供。

第1段包含露天地面/砂石/落叶与水泥区域；第2/3/4段灰色水泥纹理及岩石摆放相近。后者按用户确认保留独立capture_group，但不把相近背景当充分的跨场地泛化证据。

历史目录实际找到10段11月8～9日`approach_*_raw.mp4`，6～7日没有该后缀raw文件；不以带字/框录像补数。
已查看raw总览/候选原图并结合录制器原始写帧路径与对应JSONL，CVAT只使用原始RGB。

| 来源ID | 文件 | 完整解码帧 | 分辨率 | 审计中间候选 | 最终候选 | 建议用途 |
|---|---|---:|---|---:|---:|---|
| r3_new1 | e057c20fe375b09e167558519c87cdc1.mp4 | 3918 | 960×544 | 922 | 90 | 训练候选 |
| r3_new2 | 600de715a46f6849e2bc8096c9754fa8.mp4 | 2330 | 1280×720 | 448 | 90 | 训练候选 |
| r3_new3 | 34bc68fd1b0ed9fc6aadbd3099a03b3f.mp4 | 1182 | 1280×720 | 180 | 50 | 验证候选 |
| r3_new4 | 55bd8a6e98c0bd790f966933e0a2b8aa.mp4 | 2357 | 1280×720 | 329 | 86 | 最终测试候选 |
| r3_go2_01 | approach_20261108_211245_917119_raw.mp4 | 84 | 640×480 | 22 | 8 | 训练候选 |
| r3_go2_02 | approach_20261108_222156_798690_raw.mp4 | 233 | 640×480 | 49 | 10 | 训练候选 |
| r3_go2_03 | approach_20261108_222516_034701_raw.mp4 | 184 | 640×480 | 47 | 11 | 训练候选 |
| r3_go2_04 | approach_20261108_222842_367393_raw.mp4 | 203 | 640×480 | 53 | 13 | 训练候选 |
| r3_go2_05 | approach_20261108_223231_137141_raw.mp4 | 394 | 640×480 | 80 | 12 | 训练候选 |
| r3_go2_06 | approach_20261109_202905_232322_raw.mp4 | 237 | 640×480 | 87 | 9 | 训练候选 |
| r3_go2_07 | approach_20261109_203226_883196_raw.mp4 | 327 | 640×480 | 132 | 9 | 训练候选 |
| r3_go2_08 | approach_20261109_203609_327607_raw.mp4 | 560 | 640×480 | 203 | 42 | 训练候选 |
| r3_go2_09 | approach_20261109_204520_581746_raw.mp4 | 528 | 640×480 | 198 | 8 | 训练候选 |
| r3_go2_10 | approach_20261109_205725_163351_raw.mp4 | 367 | 640×480 | 139 | 8 | 训练候选 |

新视频9787帧，历史3117帧，合计12904帧完整解码；声明帧数与顺序解码计数一致。
历史每段raw与JSONL frame_index一一对应（合计3117条），保留container video_time及原始rgb_stamp_ns/source_rgb_elapsed_sec。固定15fps容器时间不冒充实际传感器连续时间。

## 数量、重复与模糊

最终446张新候选：训练候选310、验证候选50、最终测试候选86。候选均reviewed=false；441 pending、5 uncertain、0确认negative，没有正式train/val/test。

历史130张中，已逐图查看原图与局部质量预览，记录21张可辨认模糊方块，均有命令/速度日志运动支持；这是图像/日志质量审查，不是已画框或逐图三类别完整性验收，也不证明单一模糊成因。
5张严重拖影/边界难辨标为uncertain并排除CVAT；其余保留清晰对照、SEARCH、ALIGN、APPROACH、STOP前后及困难背景候选。

已有20张人工blue_cube图片/标签逐hash核对并复制到候选根`reused_legacy20/`，0号标签可映射为color_block，原图/原标签不变、不再送标。
旧标签只证明原蓝方块框，其他类别完整性未重新审核；后续三类别训练仍须检查是否有其它目标，不能凭旧单类框签署三类完整审核。

中间候选中6张与已人工标注历史图片为同一源视频同一帧，已排除；另63张处于旧标签±7帧上下文，保守排除（不把这63张全部声称为精确重复）。
同视频最终采样至少相隔0.5秒；第一版发现重点录像采样过密，修订后保留不同位姿/模糊对照，未为凑数保留所有相邻帧。
与已审核335张、旧20张和第二轮19张pending比对：最终候选没有相同源帧或图像hash，严格近重复核对未发现新增匹配。原19张pending未重复送标、未改负样本。

比较方法：视频SHA256+帧索引、图片hash、pHash门控后缩略图MAE和原尺寸局部差异；不是保证消除一切语义相似。数据/日志/原模型等9708个保护文件集合与SHA256不变，见preservation.json。

## V2挖掘证据

仅对第1、2段及10段历史raw逐帧离线推理，共9365帧；原始时间线未去重。第3/4段不做V2推理，避免用模型挑选留出样本。
V2 SHA256=e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1，imgsz640、rect=false、conf0.10、IoU0.45、RTX4060。每帧保存实际输入张量形状、全部阈值以上框/类别/confidence/尺寸、边缘指标、相邻框类变化和计数变化。
调用虽传rect=false，Ultralytics8.3.40 PT路径实际仍使用矩形padding：新视频3×384×640、历史3×480×640；以inference_shapes.json和逐帧记录为准，不声称640×640方形张量。

`datasets/science_targets_round3_candidates/mining/*_predictions.jsonl`保留每个源帧连续记录，`mining/summary.json`给出无预测时段。
“无class0预测”或class切换均不是GT证实漏检/错类；conf<0.10的框未返回，max_color_block_confidence_at_conf010=0不能解读为模型所有框真实最大分数为0。

310张被挖掘的训练候选中，V2框提示：小目标短边<16px（统一缩到640）17张、距图边<5%的候选93张；另有6张接近旧annotation_plan的边缘关注帧。这些都不是人工GT计数。

仅新视频1/2的模型提示：compact出现在18张、spiky出现在128张；含历史录像的总预测提示分别28/148张。可能含错类和背景误检，不能将其当真实岩石数量。

**新增真实compact和spiky的确认数量目前Unknown，需用户按实物身份标注。** 灰色普通岩石属于背景，不能凭外观/预测自动给正式类别。
136张留出图片未推理，不能当作“不含小目标/边缘目标”；真实像素尺寸与物理距离分布仍待人工框。

## 分组建议

- 第1段90、第2段90、历史130作为训练候选；历史全部归原lab_floor_20261108_09组，不把不同录像文件当独立场景。

- 第3段全视频保留验证候选，均匀覆盖50张；第4段全视频保留最终测试候选，均匀覆盖86张，未参与本轮训练/模型选择或V2困难挖掘。两者本轮暂不送标。

- 这是按用户确认的独立采集组提出的建议，不是已建立正式划分。若后续发现某组属于同一场地连续摆放，须合组或补独立场地；背景相似带来的外推限制仍存在。

- 保留旧视频B的57张作为V1/V2/V3相同条件开发验证，B已经用于模型选择，不是未触碰最终test。后续新增B帧也不可进入train。

- 旧20图blue_cube pilot的14/6来源场景已接触训练，不能作为V3最终独立test；复制标签不改变其来源组属性。

## CVAT包与顺序

目录：`vision_training/annotations/science_targets_round3/`。训练候选排除5张uncertain后305张，仅图片、无预测标签/空负标签。账户实际限制未知，当前每包≤160，未上传CVAT。

| ZIP | 张数 | SHA256 |
|---|---:|---|
| round3_priority_batch_001.zip | 153 | 44ef31a10256f4018eceb2cfe77bbb7959cd60d779d3cb677cdc69e94c5910e0 |
| round3_priority_batch_002.zip | 152 | 4572aac30053ca3f4d62bc9eb4e65455627c065f611f5d7438509364fd42dae1 |

建议先标注batch_001：含新视频1/2各45张及63张GO2图片，兼顾灰色目标身份确认、小目标/边缘/多目标、可辨认运动模糊和背景。先确认两种岩石与类别对应，能及早发现compact数据是否真正补齐，再标batch_002。

创建CVAT Detection任务，严格按以下名称/顺序：

```text
0 color_block
1 rock_compact
2 rock_spiky
```

六种颜色统一color_block；普通自然岩石不框为科学目标；无法区分compact/spiky或无法确定边界标uncertain。
人工确认无目标才标negative，之后导回时才可补空TXT。未画框/未检出均不等于negative。

保持原尺寸、稳定文件名；manifest.json绑定来源/视频hash/源帧/时间/group/图片hash/原因/blur_metric/reviewed=false。
review.csv包含全部446张状态，明确区分305待送标、136留出和5uncertain（角色查manifest）；各包另有原始batch审核表。
contact_sheets/含各源全量候选预览，priority_contact_sheets/含每包代表图与分页；Git仅保存两包代表、四段新视频总览和模糊/清晰对照。

导出后必须检查data.yaml实际names及TXT ID，复用import_cvat.py按**类别名称**映射0/1/2；发现导出顺序不同应重写标签ID，不能只改yaml。原ZIP必须留存。

根候选data.yaml只有names/status，不含train/val/test；pool/manifest.json保留reviewed=false便于后续人工审核导入，eligible_for_training始终false。

## 本地路径与复现

- 原新视频副本：vision_training/video_sources/science_targets_round3/。

- 中间自适应抽帧、完整原帧时间线：datasets/science_targets_round3_candidates/audit_new与audit_go2。

- 最终446张：datasets/science_targets_round3_candidates/selected/images，选图元数据selected/manifest.json。

- 旧20图复用：同根reused_legacy20/images、labels及manifest.json，未自动纳入任何split。

- 所有送标ZIP、完整manifest/review、联系表：annotations/science_targets_round3/。

- 版本代码/config：tools/prepare_round3.py、configs/science_targets_round3.yaml、configs/science_targets_round3_quality.yaml。

复现使用现有extract_training_frames.py的自适应间隔、全图/局部去重、原尺寸JPEG、源帧核验及CVAT打包；仅新增make_packages=False以避免给中间审计池制造重复ZIP，默认行为不改。

输出已存在时不重复执行：prepare_round3.py audit → mine（GPU）→ curate → package_final；mine只对配置inference=true来源，quality配置记录本轮目视质量判据。

## 验证结果

完整解码12904帧；原帧连续推理9365帧；全部446张独立重新解码/编码hash及原尺寸核对通过；两包ZIP CRC/解码/hash/跨包唯一性通过。
9项针对性测试通过（含原抽帧测试及审计不打包、无预测不作negative、预测尺度/边缘仅作提示），脚本py_compile/git diff --check通过。
9708个保护文件集合/hash不变；原微信视频与副本再次一致，旧20图/标签逐hash复用验证通过。
本轮没有新的人工框/负样本、训练、部署、ROS/GO2连接、MAP/Point-LIO/运动/抓取改动。真实类别完整性、GT小目标数量和V3性能均NOT VERIFIED。
