#!/bin/bash
# ============================================================
# 编译 Go2 工作区 (colcon build)
# 用法: scripts/build.sh [package1 package2 ...]  (默认全部)
# ============================================================
set -e
source "$(dirname "$0")/env.sh"

cd /home/lch/dog/go2_ws
mkdir -p build install log

EXTRA_CMAKE_ARGS=""
if [ -n "$CONDA_PREFIX" ]; then
    # 在 conda 环境内编译:
    # 1) 确保找到 conda 的头文件/库
    # 2) 用 conda 的 GCC 链接(conda 的 librclcpp 需要 GLIBCXX_3.4.31+, 系统 gcc12 不够)
    EXTRA_CMAKE_ARGS="-DCMAKE_PREFIX_PATH=$CONDA_PREFIX"
    if [ -x "$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++" ]; then
        export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc"
        export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++"
        echo "[build] 使用 conda 编译器: $CXX"
    fi
    # 3) 确保编译器能搜到 conda 的 include/lib
    #    (PCL 等包只导出自身子目录, boost/fastcdr 等需要前缀级搜索)
    export CPATH="$CONDA_PREFIX/include${CPATH:+:$CPATH}"
    export LIBRARY_PATH="$CONDA_PREFIX/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
fi

# livox_ros_driver2 需要 ROS_EDITION/DISTRO_ROS 参数
EXTRA_CMAKE_ARGS="$EXTRA_CMAKE_ARGS -DROS_EDITION=ROS2 -DDISTRO_ROS=humble"

echo ">>> colcon build ..."
# 支持: scripts/build.sh            (全部)
#       scripts/build.sh pkg1 pkg2  (指定包)
SELECT_ARGS=()
if [ "$#" -gt 0 ]; then
    SELECT_ARGS=(--packages-select "$@")
fi
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release $EXTRA_CMAKE_ARGS \
    --parallel-workers 6 \
    "${SELECT_ARGS[@]}"

echo ">>> 完成。可用 source /home/lch/dog/go2_ws/install/setup.bash 载入编译产物"
