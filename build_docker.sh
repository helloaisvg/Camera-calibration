#!/bin/bash
# 构建Docker镜像（需要sudo权限）

set -e

echo "=== 构建Kalibr标定Docker镜像 ==="
echo ""

# 检查Docker
if ! command -v docker &> /dev/null && ! command -v sudo &> /dev/null; then
    echo "错误: 需要安装Docker"
    exit 1
fi

# 解压IMU数据
if [ ! -f "AccelerometerUncalibrated.csv" ] || [ ! -f "GyroscopeUncalibrated.csv" ]; then
    echo "解压IMU数据..."
    unzip -o 2ok-2025-11-05_16-52-37.zip "AccelerometerUncalibrated.csv" "GyroscopeUncalibrated.csv" > /dev/null 2>&1
fi

# 尝试构建（使用sudo）
echo "构建Docker镜像（需要sudo权限，请输入密码）..."
echo "这可能需要5-10分钟，请耐心等待..."
echo ""

sudo docker build -t kalibr-calibration .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Docker镜像构建成功！"
    echo ""
    echo "下一步：运行标定任务"
    echo "  ./docker_run.sh \"./run_calib.sh\""
else
    echo ""
    echo "✗ Docker镜像构建失败"
    exit 1
fi

