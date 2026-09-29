from glob import glob
from setuptools import find_packages, setup

package_name = "go2_science_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="czx666",
    maintainer_email="czx666@example.com",
    description="Standalone read-only D435 diagnostics for science perception.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_diagnostics = go2_science_perception.camera_diagnostics:main",
            "camera_info_inspector = go2_science_perception.camera_info_inspector:main",
            "depth_viewer = go2_science_perception.depth_viewer:main",
            "blue_surface_center = go2_science_perception.blue_surface_center:main",
            "coarse_target_locator = go2_science_perception.coarse_target_locator:main",
        ],
    },
)
