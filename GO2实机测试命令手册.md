# GO2 实机测试命令手册（中文，从接线到关闭）

更新：2026-11-12。适用本机 PC + GO2 + D435 + S1 + 腕部 D455。
本文是手动操作说明，不是自动执行脚本。按顺序逐段复制；标注“真实运动”的命令才会启动运动。

**当前结论：旧 MAP 搜索接近已有实测成功记录；S1 抓取停靠实验版仍未通过末端停靠，不能当作已验证的一键抓取版本。**
历史实测为集成分支 `865946a` 的vx/wz版本。当前已加入vx/vy/wz三自由度停靠并取消固定总时限，**新控制尚未实机验证**。

## 0．先记住目录和终端

| 名称 | 本机实际值 |
|---|---|
| PC 主仓库（本文件所在最外层） | `/home/czx666/today_full_20260901/today_full` |
| S1 集成代码工作目录 | `/tmp/go2_map_s1_integration_20261111` |
| S1 集成分支 | `feature/map-approach-s1-grasp-integration` |
| 稳定 MAP 分支 | `feature/d435-perception-workspace`（已知基线 `e15c66c`） |
| 不用于本文的视觉伺服分支 | `feature/d435-visual-servo-optional` |
| PC 有线接口 / IP | `enp12s0` / `192.168.123.99` |
| GO2 SSH | `unitree@192.168.123.18` |
| SSH 私钥 | `/home/czx666/.ssh/id_ed25519_go2` |
| GO2 S1 交付包目录 | `/home/unitree/s1-arm-control` |
| GO2 S1 Python | `/home/unitree/s1-arm-py310/bin/python` |
| GO2 S1 串口 | `/dev/ttyUSB0` |
| GO2 S1 服务 socket | `/tmp/s1-arm-integration.sock` |
| PC 本轮构建 overlay | `/tmp/science_s1_install/setup.bash` |

建议开五个终端：A=PC准备/构建，B=GO2机械臂服务，C=GO2相机，D=PC定位感知，E=PC测试/日志。
文中“PC执行”是在PC本地终端；“GO2执行”是先SSH登录后的终端。后台启动过的节点不能再启动第二份。
`/tmp` 可能被清理；目录丢失时先看第12节恢复方法，不要在主仓库强制切分支或清掉已有改动。

## 1．接线、上电、检查网线（PC终端A）

1. GO2与PC接网线。D435接GO2；S1的USB串口和腕部D455也接GO2。
2. S1按交付要求放在初始零位附近，检查线束。路线空旷，只放一个蓝色目标，遥控急停在手。
3. 若要进行完整任务，目标放在当前朝向前方约2.5～3m附近；不要从上轮目标已在右后方的位置直接重新搜索。

```bash
ip -br addr show enp12s0
ip route get 192.168.123.18
ping -c 3 192.168.123.18
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18
```

SSH成功后，在GO2终端查看设备：

```bash
lsusb
ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM*
/home/unitree/s1-arm-py310/bin/python -c 'import pyrealsense2 as r; print([(d.get_info(r.camera_info.name), d.get_info(r.camera_info.serial_number)) for d in r.context().query_devices()])'
```

2026-11-12已枚举：D435=`213622070598`，D455=`231622302209`；后者与S1交付包标定记录一致。
若重新接线后序列号不同，应核实实际相机；不要改标定JSON来冒充原相机。
没有`/dev/ttyUSB0`时，先处理接线/供电/串口枚举，不用板载`ttyTHS*`替代。

## 2．对时：先对时，再开相机和SLAM（PC终端A）

若上次测试进程还在，先按第10节停止运动并关闭旧相机/SLAM，再改时间。
GO2重启后可能回到2022年；PC本身的日期也要正确。

```bash
date -Is
timedatectl status
cd /tmp/go2_map_s1_integration_20261111
/usr/bin/python3 camera_ws/scripts/sync_go2_clock.py
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'date -Is'
```

脚本会在本地终端询问 **GO2 sudo密码**，输入时不回显，不写进命令、文件或聊天。
如果时间刚改过，旧相机和定位链必须重新启动；不要沿用改时前的时间戳。

## 3．准备阶段关闭旧机械臂入口和旧S1服务（GO2终端B）

**先确认GO2已停住、当前没有抓取任务，机械臂有稳定支撑，再关服务。关闭服务会解除软件零位保持。**
GO2行走时必须保留S1零位保持，不能先关机械臂再让狗继续走。

先关闭旧的自定义遥控机械臂入口，防止争抢串口；这不是关闭GO2原厂遥控器：

```bash
sudo systemctl stop go2-remote-router.service
systemctl is-active go2-remote-router.service
```

检查新服务是否在运行：

```bash
pgrep -af '[r]un_arm_integration.py'
/usr/bin/python3 /home/unitree/s1-arm-control/scripts/arm_task_client.py status
```

如果是前台启动的服务，在它的原终端按`Ctrl+C`。
如果是本轮旧的nohup方式启动，下面命令只匹配该S1入口：

```bash
pkill -TERM -f '^/home/unitree/s1-arm-py310/bin/python -uB scripts/run_arm_integration.py( |$)'
pgrep -af '[r]un_arm_integration.py'
fuser /dev/ttyUSB0
```

服务收到SIGTERM会先等正在进行的抓取完成再退出；不要把它当成机械臂急停。
若`status`显示GRASPING/RETURNING_ZERO/FAULT_*，先处理当前任务/故障，不继续发新的任务。
`pgrep`无输出、`fuser`无占用，才表示旧进程退出。socket文件存在本身不表示服务健康。

**仅当你曾安装systemd常驻服务时**，用下面方法停止；普通前台启动不用运行它：

```bash
sudo systemctl stop s1-arm-integration.service
```

## 4．核对代码、构建本轮包（PC终端A）

不要在主仓库覆盖未提交改动。集成测试使用独立工作目录：

```bash
cd /tmp/go2_map_s1_integration_20261111
git status --short
git branch --show-current
git fetch origin
git log -1 --oneline
git log -1 --oneline origin/feature/map-approach-s1-grasp-integration
```

确认分支是`feature/map-approach-s1-grasp-integration`。需要更新且本地没有未提交改动时才执行：

```bash
git pull --ff-only origin feature/map-approach-s1-grasp-integration
```

已有导航底包位于主仓库的`deploy_ws/install`。首次恢复新工作目录时补这两个链接；已存在则跳过：

```bash
cd /tmp/go2_map_s1_integration_20261111
if [ ! -e deploy_ws ]; then
  ln -s /home/czx666/today_full_20260901/today_full/deploy_ws deploy_ws
fi
mkdir -p /home/czx666/today_full_20260901/today_full/camera_ws/records
if [ ! -e camera_ws/records ]; then
  ln -s /home/czx666/today_full_20260901/today_full/camera_ws/records camera_ws/records
fi
```

只构建本轮感知/接近包，不全仓库构建：

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
colcon --log-base /tmp/science_s1_log build --base-paths camera_ws/src \
  --build-base /tmp/science_s1_build \
  --install-base /tmp/science_s1_install \
  --packages-select go2_science_perception
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
source "$SCIENCE_APPROACH_OVERLAY"
```

底包`deploy_ws/install/setup.bash`若不存在，当前机器缺少原部署环境；上述单包构建不能代替导航底包安装。
没有改代码且overlay仍在时，不必每轮重新构建。改过包内Python/参数后必须用新构建结果。

同步GO2任务适配器（更新PC代码后执行，不会自行触发动作）：

```bash
ssh -i /home/czx666/.ssh/id_ed25519_go2 unitree@192.168.123.18 'mkdir -p /home/unitree/go2_map_s1'
scp -i /home/czx666/.ssh/id_ed25519_go2 \
  camera_ws/scripts/launch_go2_d435_foxy.sh \
  camera_ws/scripts/s1_task_host.py \
  camera_ws/src/go2_science_perception/go2_science_perception/s1_task_sequence.py \
  unitree@192.168.123.18:/home/unitree/go2_map_s1/
```

## 5．启动S1常驻零位保持（GO2终端B）

不要另开第二个串口程序。保持这个终端不关闭。
先离线预检，不开串口或相机：

```bash
cd /home/unitree/s1-arm-control
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -B scripts/run_arm_integration.py
```

应输出`RESULT = OFFLINE_PREFLIGHT_OK`。随后真实使能机械臂并回零保持：

```bash
OPENBLAS_NUM_THREADS=1 /home/unitree/s1-arm-py310/bin/python -uB \
  scripts/run_arm_integration.py --dev /dev/ttyUSB0 --execute
```

等待`INTEGRATION_READY = ZERO_HOLD`。在另一GO2终端查状态：

```bash
/usr/bin/python3 /home/unitree/s1-arm-control/scripts/arm_task_client.py status
```

行走前应为`state=ZERO_HOLD`、`busy=false`、`navigation_allowed=true`。
若提示`initial pose outside existing zero-start windows`，按原机械臂方法恢复初始位置；不改启动窗口。
本轮曾因J5=-0.126841rad超出±0.08rad被拒，恢复初始位置后成功启动。
若是FAULT_HOLD/FAULT_IO，不起立、不继续导航、不反复发送grasp。

## 6．启动D435（GO2终端C）

先查看是否已启动；已有一份就不要重复运行：

```bash
pgrep -af '[r]ealsense2_camera_node'
```

没有相机进程时启动，保持终端不关闭：

```bash
GO2_D435_SERIAL=213622070598 bash /home/unitree/go2_map_s1/launch_go2_d435_foxy.sh
```

这里明确绑定D435，腕部D455留给S1。不要额外给D455启动ROS相机驱动与抓取模块争抢。

## 7．启动PC定位、目标感知与TF（PC终端D）

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source /tmp/science_s1_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py \
  remote_camera:=true use_rviz:=true
```

不需要RViz时将`use_rviz:=true`改成`false`。
这一步启动Point-LIO、粗定位、camera_target_to_map、target_manager与TF，不启动行走。
必须使用`remote_camera:=true`，避免PC再起一份相机驱动。
等待雷达初始化完成；保持本终端运行。

## 8．打开状态终端，选择一种测试（PC终端E）

新PC终端需要ROS环境时，先执行：

```bash
cd /tmp/go2_map_s1_integration_20261111
source /opt/ros/humble/setup.bash
source deploy_ws/install/setup.bash
source /tmp/science_s1_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
```

另外开一个PC终端看实时状态（只读）：

```bash
cd /tmp/go2_map_s1_integration_20261111
bash camera_ws/scripts/watch_approach_logs.sh
```

以下测试每次只选一项，不能同时启动；都有SEARCH最大5m、0.25m/s、SEARCH总60s、STOP_CONFIRM总6s。
合成接近速度上限0.30m/s。普通MAP接近仍限25s；抓取停靠取消固定总时限，保留10s无进展停机及独立ODOM 5m/25s保护。没有自动避障，路线必须由人保证空旷。

### A．普通MAP搜索接近，不做S1右侧停靠、不趴下抓取【真实运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh \
  --execute --search --search-distance 5 --search-timeout 60 --timeout 25
```

这里复用集成分支中保留的普通MAP模式，沿用原相机XY 0.10m停车条件，不启用grasp-pose。
它与S1右侧抓取停车位置不同。若需要严格使用原稳定分支源码，见第11节。
S1安装在狗上时仍应保持ZERO_HOLD；这个旧入口本身没有新增S1服务监控，启动前手动确认状态。

### B．仅测试S1抓取停靠，不趴下、不抓取【真实运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_s1_task.sh \
  --execute --search-distance 5 --search-timeout 60
```

流程：SEARCH → STOP_CONFIRM → APPROACH_TO_GRASP_POSE → STOP。
位置≤4cm、航向≤10°且静止持续0.5s才完成；S1预测矩形仅记录参考。新控制同时允许前后、左右和转向，不能只看“离物块很近”。
停车后实际测量目标相对S1基座XY：+X是狗后方，+Y是狗右侧；与日志预测分别记录。

### C．完整单目标流程【真实运动＋趴下＋机械臂抓取＋起立】

```bash
cd /tmp/go2_map_s1_integration_20261111
export SCIENCE_APPROACH_OVERLAY=/tmp/science_s1_install/setup.bash
bash camera_ws/scripts/run_map_s1_task.sh --execute --full \
  --search-distance 5 --search-timeout 60 \
  --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json
```

流程：SEARCH → STOP_CONFIRM → 抓取停靠 → STOP → StandDown → S1原完整抓取 → ZERO_HOLD → StandUp。
无需再手填四个参数。本机已测：趴下反馈高度上限0.084m、起立下限0.304m，两种mode均0。
这些是本机反馈判据，不是S1基座高度。更改机械安装/支撑条件后不能直接视为仍有效。
停靠失败时不会趴下或抓取。`accepted=true`只代表接收任务，必须等待任务完成及ZERO_HOLD。
当前最后一次该流程停在停靠超时，未实际触发抓取，完整一键任务仍未验证通过。

### D．仅打印完整计划【不打开硬件、不运动】

```bash
cd /tmp/go2_map_s1_integration_20261111
/usr/bin/python3 camera_ws/scripts/run_map_s1_task.py --full \
  --posture-profile camera_ws/src/go2_science_perception/config/science_s1_posture_go2_20261112.json
```

## 9．最少量只读检查和日志查看（按需执行）

先加载第8节PC环境。下面`hz`命令持续输出，查看后按`Ctrl+C`结束；不要一次全部长期运行。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /utlidar/cloud
ros2 topic hz /utlidar/imu
ros2 topic hz /state_estimation
ros2 topic info /cmd_vel_raw --verbose
ros2 run tf2_ros tf2_echo map vehicle
```

相机内参单帧：

```bash
ros2 topic echo --once --qos-reliability best_effort /camera/camera/color/camera_info
```

测试程序会自动记录状态JSON、控制/安全门日志、原始和叠加画面；本轮录像正常关闭后才退出。
数据实际保存在主仓库，不应只找/tmp：

```bash
cd /home/czx666/today_full_20260901/today_full
ls -lt camera_ws/records/ | head -25
```

查看最近一次结果（Python不会发送任何机器人命令）：

```bash
/usr/bin/python3 - <<'PY'
import json
from pathlib import Path
base = Path('/home/czx666/today_full_20260901/today_full/camera_ws/records')
files = list(base.glob('run_*.json'))
if not files:
    raise SystemExit('还没有run_*.json记录')
p = max(files, key=lambda x: x.stat().st_mtime)
data = json.loads(p.read_text())
print('记录：', p)
print(json.dumps(data.get('final', {}), ensure_ascii=False, indent=2))
PY
```

实验说明：`camera_ws/docs/experiments.md`；集成原理及接口：`camera_ws/docs/map_s1_grasp_integration.md`。
软件状态、MAP误差不等于独立物理测量；入盒也不能仅靠last_exit_code=0判定。

## 10．停止一次测试、关闭节点、重新开始

### 10.1 先停止运动（PC已加载第8节环境的终端）

物理遥控急停始终保留。软件停止：

```bash
ros2 topic pub --once /stop std_msgs/msg/Int8 '{data: 1}'
```

测试终端也可按`Ctrl+C`，程序会请求停车、关闭本轮控制器和安全门、结束录像。
先确认机器人实际停止，再做下面清理。抓取中的软件停止不会自动松爪或起立，也不等价于切断机械臂动力。

### 10.2 停止定位和相机

在PC终端D按`Ctrl+C`，关闭本轮定位/感知链。
在GO2相机终端C按`Ctrl+C`，关闭D435。
本轮若还是旧的后台相机启动方式，可在GO2终端执行：

```bash
pgrep -af '[r]ealsense2_camera_node'
pkill -INT -f '^/home/unitree/go2_d435_foxy_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node( |$)'
```

该命令针对本目录的RealSense节点；不要对其他相机任务或S1视觉进程执行宽泛`pkill python`。
定位若也是旧后台启动，先在PC查对应launch PID：

```bash
pgrep -af '[r]os2 launch go2_science_perception d435_slam_target_validation.launch.py'
```

只对上面查出的本轮launch PID执行`kill -INT 实际PID`；不要复制历史PID或直接杀所有ROS节点。

### 10.3 最后才关闭机械臂服务

GO2停止、抓取任务结束且机械臂有支撑后，按第3节关闭S1服务；此后不再让GO2行走。
只想重新测导航、且S1仍正常ZERO_HOLD时，可以保留S1服务，不必每轮关开。

### 10.4 再次测试的顺序

停止本轮运动 → 结束日志/录像 → 需要时重启定位感知 → 把目标和机器人恢复到测试起点 → 确认S1 ZERO_HOLD → 第8节选一个入口。
不要直接从FAIL/FAULT继续发grasp；不要把失败位置当作新的已确认抓取位姿。

## 11．严格使用原稳定MAP分支（可选，与第8节二选一）

仅在确实要回退原稳定源码时执行。下面建立独立只读提交工作树，不修改稳定分支：

```bash
cd /home/czx666/today_full_20260901/today_full
git fetch origin
git rev-parse origin/feature/d435-perception-workspace
git worktree add --detach /tmp/go2_stable_map_test origin/feature/d435-perception-workspace
```

目录已存在时跳过worktree add，直接进入已有目录，先核对`git log -1 --oneline`。

```bash
cd /tmp/go2_stable_map_test
source /opt/ros/humble/setup.bash
source /home/czx666/today_full_20260901/today_full/deploy_ws/install/setup.bash
colcon --log-base /tmp/science_stable_log build --base-paths camera_ws/src \
  --build-base /tmp/science_stable_build \
  --install-base /tmp/science_stable_install \
  --packages-select go2_science_perception
```

第6节GO2上的D435仍按明确序列号启动。先停掉旧的PC定位链，在新PC终端启动稳定版本：

```bash
cd /tmp/go2_stable_map_test
source /opt/ros/humble/setup.bash
source /home/czx666/today_full_20260901/today_full/deploy_ws/install/setup.bash
source /tmp/science_stable_install/setup.bash
source camera_ws/scripts/science_approach_pc_env.bash
ros2 launch go2_science_perception d435_slam_target_validation.launch.py \
  remote_camera:=true use_rviz:=true
```

另一个PC终端运行稳定搜索接近【真实运动，不抓取】：

```bash
cd /tmp/go2_stable_map_test
if [ ! -e deploy_ws ]; then
  ln -s /home/czx666/today_full_20260901/today_full/deploy_ws deploy_ws
fi
export SCIENCE_APPROACH_OVERLAY=/tmp/science_stable_install/setup.bash
bash camera_ws/scripts/run_map_anchor_experiment.sh \
  --execute --search --search-distance 5 --search-timeout 60 --timeout 25
```

稳定工作树默认日志写在`/tmp/go2_stable_map_test/camera_ws/records`；保存到主仓库后再清理/tmp。
稳定分支没有S1完整任务入口；不能在这个目录运行`run_map_s1_task.sh`。

## 12．常见问题和恢复

### 集成目录或/tmp构建目录不见了

只有在`/tmp/go2_map_s1_integration_20261111`确实不存在时，才在PC重新取一份：

```bash
git clone --branch feature/map-approach-s1-grasp-integration --single-branch \
  https://github.com/Czx666666888888/GO2-Lidar-Itself.git \
  /tmp/go2_map_s1_integration_20261111
```

然后执行第4节链接/构建/同步脚本。原始数据仍在主仓库`camera_ws/records`。
此命令不会修复缺失的导航底包或安装GO2的S1交付包。

### 全新GO2尚未部署S1交付包（本机已经部署，不要覆盖）

PC复制原包：

```bash
scp -i /home/czx666/.ssh/id_ed25519_go2 \
  '/home/czx666/文档/xwechat_files/wxid_kt48eys7r67z22_c5b4/msg/file/2026-11/S1_arm_integration_20261111_auto_zero_hold_v1.zip' \
  unitree@192.168.123.18:/home/unitree/
```

GO2校验包；已部署目录存在时不要重复解压覆盖：

```bash
sha256sum /home/unitree/S1_arm_integration_20261111_auto_zero_hold_v1.zip
```

正确SHA256：`a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e`。

```bash
/usr/bin/python3 - <<'PY'
from pathlib import Path
import hashlib
import zipfile
archive = Path('/home/unitree/S1_arm_integration_20261111_auto_zero_hold_v1.zip')
assert not Path('/home/unitree/s1-arm-control').exists(), '目录已存在，不覆盖'
assert hashlib.sha256(archive.read_bytes()).hexdigest() == 'a7e08d875366dc777e2a41e19401f537fa929584c9c89ae6a0ceaea5aab6596e'
with zipfile.ZipFile(archive) as z:
    z.extractall('/home/unitree')
PY
```

这只负责原包部署，不等于全新Jetson依赖安装完成。新环境按交付包`docs/TEAM_INTEGRATION.md`安装，
不要覆盖已验证的SDK/标定；本机继续使用现有S1 Python环境。

### SDK2段错误 / 起立被StopMove=-1挡住

最新完整任务入口已处理本机这两项问题。姿态进程专用环境如下，**不要全局写入.bashrc，也不要给相机/S1抓取环境一起套用**：

```bash
CYCLONEDDS_HOME=/home/unitree/cyclonedds_ws/install/cyclonedds
LD_LIBRARY_PATH=/home/unitree/cyclonedds_ws/install/cyclonedds/lib
```

只写变量不会发送任何动作。协调器会为姿态子进程自动设置它们。
最新程序只在确认趴下静止和ZERO_HOLD后直接起立；趴下前仍保留StopMove要求。

### localization jump / TF持续失败 / 地图位置数十米异常

停止本轮运动；保留日志；在GO2站稳后关闭PC定位链，再按第7节原参数重启。
不能靠放宽定位跳变阈值继续走。本轮曾在独立趴下/起立采样后遇到旧定位链失稳，因果未确定。

### 多次STOP_CONFIRM后没有接近

看`stop_confirm_attempts`、有效样本数、时间跨度、散布和是否真的静止。
原总预算6s、至少5个样本、至少0.6s跨度等条件仍保留；不要只看状态名就判断已经确认。

### 抓取停靠超时与卡滞

历史vx/wz版本最近一次位置误差约9.94cm、航向约8.64°，未达到当时2cm/0.05rad条件。
当前三自由度版本使用4cm/10°，已取消固定25s总任务时限；旧`--timeout 25`在S1入口仅保留兼容、不会限制停靠。
若新版本出现`odometry continuation limit`，是独立的视觉丢失ODOM 5m/25s约束；
若出现`grasp docking stalled`，是10s没有有效位姿/停稳进展。两者都需保留日志分析，不自动扩大保护。
完整新控制与抓取仍需实机验证，原S1模块继续自行检查可观测性和可达性。
