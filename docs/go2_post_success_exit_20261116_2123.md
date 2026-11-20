# 2026-11-16 21:23 FULL首轮成功后退出（只读）

证据full_1794835426811702994（7c17021/V3/left）、pc_time_1794835426744396016/report.json。

- 导航最终STOP，grasp_docking_complete=true；S1本轮alignment_20261116-212433结果last_exit_code=0。
- 21:25:29.745返回ZERO_HOLD；21:25:32.325起立反馈confirmed=true，TASK_COMPLETE及RESULT均task_motion_complete=true，stand_up.confirmed=true。
- 21:25:42.659恢复相机ack ok=true，probe ready/camera_ok/slam_ok均true、errors=[]。
- 没有round_01_result.json，three_grasps计数仍0，无advance_yolo_round阶段、无第二轮run。PC包装器记录localization_exit=1、总118.808秒、校时恢复成功。

确定范围：第一轮抓取/起立软件完成，失败位于PC run_map_s1_task收尾/返回及结果落盘之前，不是YOLO第二轮解锁拒绝，也不是相机恢复未就绪。S1 RESULT本身通过源码的完成条件。

日志未保存PC协调器stdout/stderr/traceback及远端SSH子进程returncode，无法确认具体异常；不能用S1 RESULT推断SSH传输退出码必为0。源码中的心跳write/flush或finally close存在与远端退出相遇的BrokenPipe可能，非本轮已证实原因；亦需核对completed.returncode的错误路径。
control.log中publisher context invalid发生在导航退出收尾，随后仍运行抓取与起立，不能当作21:25的退出原因。
应补存协调器异常与子进程退出码，或取得本轮终端Traceback，再定点修复；不自动继续第二轮/补计数/重抓。本轮未修改生产代码、未重启/动作。
