from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="go2_keyboard_teleop",
            executable="go2_keyboard_teleop_node",
            name="keyboard_teleop",
            output="screen",
            parameters=[{
                "topic_cmd_vel_out": "cmd_vel_out",   # go2_ros2_sdk 驱动订阅
                "topic_cmd_vel": "cmd_vel",           # 标准调试话题
                "max_vx": 0.6,
                "max_vy": 0.4,
                "max_vyaw": 0.8,
                "publish_rate": 20.0,
            }],
        ),
    ])
