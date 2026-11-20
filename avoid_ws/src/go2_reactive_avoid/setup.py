from setuptools import setup

package = "go2_reactive_avoid"
setup(
    name=package,
    version="0.1.0",
    packages=[package],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package}"]),
        (f"share/{package}", ["package.xml"]),
    ],
    entry_points={"console_scripts": ["avoid_test = go2_reactive_avoid.node:main"]},
)
