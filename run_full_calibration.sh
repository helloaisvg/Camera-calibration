#!/bin/bash
# 完整标定流程（包含安装wxPython和运行标定）

set -e

PASSWORD="5423496"

echo "========================================"
echo "=== 完整标定流程 ==="
echo "========================================"
echo ""

cd /home/sss/cameraproj/Camera-calibration

echo "步骤1: 安装wxPython依赖..."
echo "$PASSWORD" | sudo -S docker run -i --rm \
    -v "$(pwd):/workspace" \
    -v "$(pwd)/kalibr_ws:/kalibr_ws" \
    kalibr-calibration \
    bash -c "
apt-get update -qq
apt-get install -y python3-wxgtk4.0 > /dev/null 2>&1
echo '✓ wxPython安装完成'
"

echo ""
echo "步骤2: 运行标定任务..."
echo "$PASSWORD" | sudo -S docker run -i --rm \
    -v "$(pwd):/workspace" \
    -v "$(pwd)/kalibr_ws:/kalibr_ws" \
    -w /workspace \
    kalibr-calibration \
    bash -c "
source /opt/ros/noetic/setup.bash

# 安装wxPython（如果还没安装）
apt-get update -qq
apt-get install -y python3-wxgtk4.0 > /dev/null 2>&1

# 加载kalibr
source /kalibr_ws/devel/setup.bash
export PATH=\$PATH:/kalibr_ws/devel/.private/kalibr/lib/kalibr

# 运行标定
./run_calib.sh
"

echo ""
echo "========================================"
echo "=== 标定完成 ==="
echo "========================================"

