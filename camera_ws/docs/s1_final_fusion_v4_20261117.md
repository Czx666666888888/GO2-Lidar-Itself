# 2026-11-17 S1 final fusion v4 与全流程融合

## 来源与实际范围

基线：集成分支 `0b3159d`，未修改主视觉工作目录已有脏改动。
用户交付 `S1_arm_final_fusion_20261117_v4.zip` 已按原字节保存在
`camera_ws/s1_service/deliveries/`，SHA256：
`6950d2cb8135f6ee56ccc0535dc1da6dd6a68e17478fea8cacbd5ff0bc14c90b`。
包版本 `20261117_final_fusion_v4`。SDK版权/来源、原标定、绑定计划、示教轨迹、
运行记录、原测试均保存在原包，未重新写入配置或伪造实验结果。

以下为源码与离线验证结论，NX部署及真机行为均 **NOT VERIFIED**。

## 抓取与卸料接口

现有 `go2_test.sh` PC入口改用 `s1_v4_delivery.py`，目标目录为
`/home/unitree/s1-arm-control-v4`。上层默认arm-client与状态请求同步切换。
原v1/v2目录保留。ZIP整体哈希、包内逐文件哈希、适配文件哈希均核验。
本地不再依赖微信路径或额外环境变量寻找源包。
安装到独立临时目录，校验后原子发布；发现现存目录/修改或活动S1所有者即拒绝覆盖。
安装后的复用也核验原标定、轨迹、SDK与适配文件，不把旧服务冒充新版。
`S1_V2_COMPAT.json`和`.v2-compat-upstream`沿用既有兼容层元数据格式与备份名，
其中实际兼容版本为 `v4_multitarget_dump_1`，不表示运行的是v2。

保留既有唯一串口服务、USB0/1自动识别与设备身份变化检查、原零位容差、
原回零、状态读取、自动左右选择，以及多目标流程的显式安全漏抓处理。
新增grasp/dump请求前的实际反馈就绪检查；两类任务互斥，busy时不重复入队。
抓取协调器检查last_task，dump结果不能充当抓取结果触发起立。
运行manifest记录机械臂包版本与ZIP哈希。

原包的新参数随包接入：普通抓取/搬运1.5rad/s、下降0.75rad/s、下降全开、
接触0.20Nm/夹紧0.40Nm、预抓取高度容差10mm、J6映射边界修正。
本轮未另外放宽误差阈值、改写示教轨迹或设置未验证的载荷上限。

抓取仍为：原MAP停靠成功 → D435释放 → 原趴下反馈确认 →
单S1服务执行所选侧grasp → 新结果/ZERO_HOLD → 原起立反馈确认 →
D455释放/D435与完整定位链恢复 → 下一个目标。抓取不会自动触发dump。

新版服务保留 `status/grasp/dump` socket命令。dump必须在全部收集完成、
返回卸载区、停车并恢复示教支撑构型后，由上层明确请求。
当前三目标流程尚无“返回卸载区并反馈确认”的实现，本轮没有擅自新增导航路线，
也没有把第3轮结束当成卸载区到达。自动返航/卸料编排仍为待决策工作。
服务自身的dump执行及持夹故障保持代码已保留，不能据此称全流程自动卸料完成。
抓取placement_confirmed、倾倒unload_confirmed仍为null。

## 用户追加：横向停车32cm

左右实际overlay从 `+0.36/-0.36` 改为 `+0.32/-0.32` m；
离线计划同步。自动选侧及记忆目标停车距离通过shared_side_settings复用相同配置。
32cm为目标相对S1基座的横向位置，不是到位误差阈值。
最新基线的8cm到位容差、10度、0.5秒停稳及其余控制参数保持原值。
未修改相机外参中的0.36m，也未把S1基座测量注释中的0.36m改成停车距离。

## 离线证据

- 原包 `verify_handoff.py --tests`：左右预检及187项通过。
  临时Python环境补MuJoCo3.4.0，PYTHONPATH指向包内SDK；不接相机/串口。
  最初本地Unix socket回环被沙箱拒绝，获得权限后重跑整套通过。
- 已生成的v4适配包 `verify_handoff.py`：左右预检及适配/原包完整性通过。
- 全流程、相机交接、多目标、左右停靠、S1交接、安装/复用/篡改阻断：169 passed。
- 使用实际v2原ZIP验证共用兼容函数原行为：22 passed，无跳过。
- 新文件flake8、编译、shell语法和默认离线计划通过。
- `go2_science_perception` 单包colcon构建退出0；安装的左右Y=±0.32、
  配置逐字节一致、共享选侧几何及s1_task_sequence源码hash核对通过。
- 本机原始输出：`/tmp/s1_v4_upstream_validation_unrestricted.log`、
  `/tmp/s1_v4_regression.log`、`/tmp/s1_v2_compat_regression.log`、
  `/tmp/s1_v4_build.log`；长期副本放在主工作目录camera_ws/records/s1_v4_20261117/。

## 部署和待验证边界

本轮只commit，不push、不SSH、不部署、不重启S1、不使能/回零/抓取/倾倒。
现有入口要求本地commit与远端FETCH_HEAD相等；本地未push时会保留提交并阻断实机执行，
不能绕过该检查将本地提交当成远端最新。发布后仍沿用原 `bash go2_test.sh --execute` 入口；
它会执行真实运动，必须另有本轮运动授权及原现场条件。
旧S1服务仍存活时会阻断版本切换；服务关闭会失能，不能自动杀掉保持进程。
SDK目标平台、当前安装构型/盒口与把手位置、支撑稳定性及32cm实际抓取均待真机验证。

原包仍有pause_on_tracking_lag等待路径；本轮未声明修复历史J6长等待或USB掉线。
等待有界退出、SDK电机包级新鲜度与故障后的实际机械保持需单独评审，
不能用离线测试或软件ZERO_HOLD替代物理确认。
