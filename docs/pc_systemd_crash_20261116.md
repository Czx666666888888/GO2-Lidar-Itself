# 2026-11-16 PC校时查询超时：PID1崩溃证据

本轮仅只读查询，不执行full、不重启服务、不改系统包。来源为本机journalctl、apt/unattended-upgrades/dpkg日志、dpkg-query和busctl输出。

## 已确认的直接原因

2026-11-16 16:29:40，PID1重新执行后发生general protection fault，系统管理器冻结：

```text
systemd[1]: Reexecuting.
systemd[1]: systemd 249.11-0ubuntu3.23 running in system mode
kernel: traps: systemd[1] general protection fault ... in libsystemd-shared-249.so
systemd[1]: Caught <SEGV>, dumped core as pid 6786.
systemd[1]: Freezing execution.
```

随后systemd-oomd报告Connection refused；D-Bus中org.freedesktop.systemd1只有activatable，没有连接的PID。校时服务PID809仍运行且拥有org.freedesktop.timesync1，说明不能把管理器查询失败当作校时未运行。PC入口的3秒超时是对该系统故障的检测，不是造成PID1崩溃的原因。

用户报告pc_time_1794818677624651151在3.004秒记录查询超时，未进入机器人流程。

## 同时段系统更新与因果边界

- unattended-upgrade在16:29:38开始写dpkg日志；16:29:39～40正在把libc6从2.35-0ubuntu3.15升级到2.35-0ubuntu3.18，PID1在16:29:40重新执行并崩溃。
- 可确认两者同窗发生；自动更新/库替换是排查线索，不能仅据时间相关就断言具体崩溃缺陷。精确函数/触发链需分析/var/crash/_usr_lib_systemd_systemd.0.crash（633509字节），本轮没有读取core内容。
- systemd和libsystemd0当前249.11-0ubuntu3.23，dpkg-query均为installed；不是据此证明运行管理器健康。

## 恢复前需要考虑的实际问题

自动更新还在16:31:44配置新内核6.8.0-149对应NVIDIA595模块时发生ld.bfd断言/segmentation fault，缺少nvidia.ko。dpkg-query确认linux-modules-nvidia-595-6.8.0-149-generic为half-configured，linux-modules-nvidia-595-generic-hwe-22.04为unpacked。当前uname -r仍为6.8.0-138-generic。该后续包失败不直接证明16:29 PID1崩溃原因，但恢复系统时应考虑新内核驱动不完整，不能假定直接重启默认新内核无影响。

本轮dpkg --audit因非root权限不足，未执行成功；包状态来自只读dpkg-query。sudo -n没有有效凭证，未进行管理员操作。系统恢复及恢复后查询/机器人实机效果NOT VERIFIED。
