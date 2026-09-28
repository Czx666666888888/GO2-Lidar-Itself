from setuptools import find_packages, setup

package_name = "go2_keyboard_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/go2_keyboard_teleop.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="go2-dev",
    maintainer_email="dev@example.com",
    description="Keyboard teleoperation for Unitree Go2 (Twist only, no path planning)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "go2_keyboard_teleop_node = go2_keyboard_teleop.keyboard_teleop:main",
            "go2_keyboard_teleop_dds = go2_keyboard_teleop.keyboard_teleop_dds:main",
            "go2_cmd_monitor = go2_keyboard_teleop.cmd_monitor:main",
            "go2_sensor_recorder = go2_keyboard_teleop.sensor_recorder:main",
            "go2_sensor_recorder_dds = go2_keyboard_teleop.sensor_recorder_dds:main",
            "go2_safety_gate = go2_keyboard_teleop.safety_gate:main",
            "go2_debug_control = go2_keyboard_teleop.debug_control:main",
            "wp5_explore_node = go2_keyboard_teleop.wp5_explore_node:main",
            "nbv_explore_node = go2_keyboard_teleop.nbv_explore_node:main",
            "base_odom_node = go2_keyboard_teleop.base_odom_node:main",
        ],
    },
)
