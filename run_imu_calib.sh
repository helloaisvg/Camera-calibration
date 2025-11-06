#!/bin/bash
set -e

echo "=== IMU噪声参数标定 ==="
echo ""
echo "此脚本用于标定IMU的噪声参数（需要静止数据）"
echo ""

# 检查是否有IMU bag文件
if [ ! -f "imu.bag" ]; then
    echo "错误：找不到imu.bag文件"
    echo "请先运行: python3 csv_to_imu.py -a AccelerometerUncalibrated.csv -g GyroscopeUncalibrated.csv -o imu.bag"
    exit 1
fi

# 检查imu_utils是否安装
if ! command -v rosrun &> /dev/null || ! rosrun imu_utils imu_calib --help &> /dev/null; then
    echo "错误：未安装imu_utils"
    echo "安装方法: sudo apt install ros-noetic-imu-utils"
    exit 1
fi

echo "开始IMU标定..."
echo "注意：需要静止数据（设备静止放置5分钟）"
echo ""

rosrun imu_utils imu_calib \
    -i imu.bag \
    -t /imu0 \
    -n imu0 \
    -s 100

echo ""
echo "=== IMU标定完成 ==="
echo "结果文件：imu0_imu_param.yaml"
echo "此文件将在联合标定时使用"

