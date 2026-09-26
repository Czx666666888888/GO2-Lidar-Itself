import sys
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped

RES = 0.1; OBSTACLE_THRE = 0.2; UNKNOWN_INT = 1.45

def render(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'),
           ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    grid = {}; goals = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/terrain_map':
            for p in point_cloud2.read_points(deserialize_message(data, PointCloud2), field_names=['x','y','intensity'], skip_nans=True):
                x, y, it = p[0], p[1], p[2]
                ix, iy = int(round(x/RES)), int(round(y/RES))
                s = 0 if it >= UNKNOWN_INT else (2 if it >= OBSTACLE_THRE else 1)
                grid[(ix,iy)] = max(grid.get((ix,iy), 0), s)
        elif topic == '/goal_point':
            m = deserialize_message(data, PointStamped)
            goals.append((m.point.x, m.point.y))

    xs = [k[0] for k in grid]; ys = [k[1] for k in grid]
    ixmin, ixmax = min(xs), max(xs); iymin, iymax = min(ys), max(ys)
    # 去重目标 + 序号
    uniq = []; goal_idx = {}
    for x, y in goals:
        k = (round(x, 2), round(y, 2))
        if k not in goal_idx:
            goal_idx[k] = len(uniq); uniq.append(k)
    # 目标栅格 -> 序号(去重)
    gc = {}
    for (rx, ry), idx in goal_idx.items():
        gc[(int(round(rx/RES)), int(round(ry/RES)))] = idx

    name = bag.split('/')[-1]
    print(f'\n===== {name} =====')
    print(f'栅格 {ixmax-ixmin+1}x{iymax-iymin+1}  terrain点={len(grid)}  目标消息={len(goals)}  去重目标={len(uniq)}')
    print('图例: #=障碍 .=可行 空格=未知  [数字]=WP5去重目标序号')
    for j in range(iymax, iymin-1, -1):
        row = ''
        for i in range(ixmin, ixmax+1):
            s = grid.get((i, j), 0)
            if (i, j) in gc:
                c = str(gc[(i, j)] % 10)
            else:
                c = {0: ' ', 1: '.', 2: '#'}[s]
            row += c
        print(row.rstrip())
    print('去重目标坐标:')
    for (rx, ry), idx in sorted(goal_idx.items(), key=lambda kv: kv[1]):
        print(f'  [{idx:2d}] ({rx:6.2f},{ry:6.2f})')

for b in sys.argv[1:]:
    render(b)
