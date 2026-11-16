# 2026-11-16 YOLO V3接入

用户明确要求使用最新V3；以45e6b2f（本地/远端一致）为基线。
来源：主仓库vision_training/reports/science_targets_mixed_v3/README.md、run_summary.json，实际best.pt及本轮离线检查。

## 实现

approach/dock/full共用detector_assets，YOLO默认从原数据根目录读取：
vision_training/runs/science_targets_mixed_v3/baseline/weights/best.pt
SHA256=db74e03e4073574fb91b9fe3cb404c0f256b13c6451c014639818471e186e391。
原V2哈希e9bfab914e3412738b1ba9a45db24646a756ac75ab92abda8012aae3a37b46a1仍接受，显式--model-path指定；未知哈希/缺默认权重失败退出，不静默回退。
FULL manifest新增model_version，并记录真实选定哈希；所有链路probe（包括轮间恢复和只读startup-check）使用对应哈希，不再写死V2。
原权重、venv不复制、不覆盖、不纳入Git。通过原records链接解析主仓库资源；GO2_VISION_ROOT/显式路径继续支持。

沿用640输入、confidence0.25、IoU0.45、class_names=color_block（仅方块进入原适配器），0color_block/1rock_compact/2rock_spiky不变。
不改RGB-D时间配对、深度ROI、frame/TF、MAP、Manager、SEARCH停稳/关联、S1、相机交接和三轮链。
多目标仍使用原关联；本轮不启用岩石类别抓取，不改变唯一关联判断。真实多目标效果NOT VERIFIED。

## 验证及边界

- 53项定向离线测试通过，覆盖默认V3、显式V2、未知模型拒绝、缺模型拒绝、HSV无需权重、manifest/启动探针哈希和原交接/三轮回归。
- py_compile、git diff --check、FULL YOLO离线计划通过。仅Python入口修改，无ROS包变更，无需重新构建包。
- 实际V3模型加载：YOLO.names={0:color_block,1:rock_compact,2:rock_spiky}。
- 取full_1794832708501958436/approach_20261116_204016_049212_raw.mp4首帧（480x640），CPU预测1帧成功、1框，inference约47.34ms；单帧输出不证明目标正确性或实时连续吞吐。
- 无相机采流、ROS节点、NX部署/服务重启、GO2/S1动作。运行仍为PC推理+NX相机原链路，未迁移推理到NX。RGB-D/MAP在线及物理抓取NOT VERIFIED。
- 既有开发验证：V3方块P/R=.7646/.8702，V2=.8666/.6958；召回改善同时误检增多。spiky召回.30，真实compact训练0框；用户本轮选V3不改变这些验证边界。
- 上轮S1预抓取跟踪等待/USB掉线尚未修复；本轮接入模型不代表该故障恢复。

## 后续命令（仅提供，本轮未执行）

从主仓库规范入口显式使用YOLO（原命令若有--detector hsv应替换）：

```bash
bash /home/czx666/today_full_20260901/today_full/go2_test.sh --execute --mode full --detector yolo --view
```

回退V2：同命令追加
`--model-path /home/czx666/today_full_20260901/today_full/vision_training/runs/science_targets_mixed_v2/baseline/weights/best.pt`。
HSV继续显式--detector hsv。下一次动作前需确认S1故障恢复及现场条件，本轮没有自动重试。
