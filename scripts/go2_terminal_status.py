"""Relay complete status lines from files written by the task processes."""
import json


STATUS_PREFIX = 'FULL_STATUS '
STATUS_LABELS = {
    'MISSION_PHASE': '导航阶段',
    'MISSION_HEARTBEAT': '当前状态',
    'MISSION_GOAL': '导航目标',
    'MISSION_FAILURE': '导航停止',
    'TARGET_DEFERRED': '目标暂缓',
    'DIRECT_GRASP_ADMITTED': '墙距符合，暂停探索并直接接近',
    'GRASP_SIDE_LOCKED': '抓取侧已确认',
    'STAGING_PROBE': '停靠试算',
    'STAGING_SIDE_FALLBACK': '切换抓取侧',
    'STAGING_SIDE_SELECTED': '已选抓取侧',
    'STAGING_PROBE_TIMEOUT': '停靠试算超时',
    'STAGING_REPLAN': '重新规划停靠',
    'CONTROL_PHASE': '接近控制',
    'DOCK_STOP_CONFIRMED': '停靠停稳已确认',
    'NAVIGATION_RELEASED': '导航控制权已释放',
    'NAVIGATION_RELEASE_FAILED': '导航控制权释放失败',
    'POST_DOCK_HANDOFF_READY': '准备交接抓取',
    'POSTURE_HOST_STARTED': '机械臂任务已启动',
    'POSTURE_STATUS': '机械臂任务',
    'TASK_ERROR': '任务出错',
}
POSTURE_LABELS = {
    'SDK_CONTROL_READY': '姿态控制已就绪',
    'STAND_DOWN_REQUEST': '请求趴下',
    'STAND_DOWN_CONFIRMED': '趴下已确认',
    'GRASP_START_REQUEST': '请求抓取',
    'GRASP_REQUEST': '抓取请求已提交',
    'GRASP_RUNNING_OBSERVED': '正在抓取',
    'GRASP_RETURNED_ZERO_HOLD': '机械臂已回零',
    'GRASP_FAILED_SAFE_RETURN': '抓取失败，已安全回零',
    'STAND_UP_RPC_START': '请求起立',
    'POSTURE_FEEDBACK_CONFIRMED': '姿态反馈已确认',
    'ARM_STATUS': '机械臂反馈',
    'RESULT': '抓取轮次结果',
}


def complete_lines(stream):
    """Read only complete lines, retaining any partial line for the next poll."""
    while True:
        offset = stream.tell()
        try:
            line = stream.readline()
        except UnicodeDecodeError:
            stream.seek(offset)
            return
        if not line.endswith('\n'):
            stream.seek(offset)
            return
        yield line


def show_status(stream, round_label):
    for line in complete_lines(stream):
        if not line.startswith(STATUS_PREFIX):
            continue
        try:
            row = json.loads(line[len(STATUS_PREFIX):])
        except ValueError:
            continue
        if not isinstance(row, dict):
            continue
        event = row.get('event')
        label = STATUS_LABELS.get(event)
        if label is None:
            continue
        if event == 'POSTURE_STATUS':
            label = POSTURE_LABELS.get(row.get('step'), label)
        fields = []
        for key, title in (('state', '状态'), ('phase', '子阶段'), ('target_id', '目标'),
                           ('side', '侧别'), ('outcome', '结果'), ('xyz', 'MAP目标'),
                           ('direction', '姿态方向'), ('distance_m', '墙距m'), ('reason', '原因'), ('rejected', '候选排除')):
            val = row.get(key)
            if val is None:
                continue
            if key == 'xyz':
                val = '(' + ', '.join(f'{item:.2f}' for item in val[:2]) + ')'
            if key == 'distance_m':
                val = f'{val:.3f}'
            fields.append(f'{title}={val}')
        print(f'[{round_label}] {label}' + (' | ' + ' | '.join(fields) if fields else ''), flush=True)


def follow_status(stream, round_label, stop):
    """Keep terminal updates independent of camera handoff and job supervision."""
    while not stop.wait(.2):
        show_status(stream, round_label)
    show_status(stream, round_label)
