#!/bin/bash
set -e

# 检查并初始化ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "已初始化ROS 1 Noetic环境"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "警告: 检测到ROS 2环境，kalibr需要ROS 1 Noetic"
    echo "请参考INSTALL.md安装ROS 1或使用Docker方案"
    if [ "$FORCE_RUN" != "1" ]; then
        echo "设置环境变量FORCE_RUN=1可以强制继续（可能会失败）"
        exit 1
    fi
fi

echo "=== 相机和IMU标定全流程 ==="

# 解压IMU数据（如果还没解压）
if [ ! -f "AccelerometerUncalibrated.csv" ] || [ ! -f "GyroscopeUncalibrated.csv" ]; then
    echo "解压IMU数据..."
    unzip -o 2ok-2025-11-05_16-52-37.zip "AccelerometerUncalibrated.csv" "GyroscopeUncalibrated.csv"
fi

# 1. 转视频为ROS bag
echo ""
echo "步骤1: 转换视频为ROS bag..."
VIDEO_FILE="7fa2315596266b48a36d5575ece01b77.mp4"
if [ ! -f "$VIDEO_FILE" ]; then
    echo "错误: 找不到视频文件 $VIDEO_FILE"
    exit 1
fi
python3 video_to_bag.py -i "$VIDEO_FILE" -o cam.bag

# 2. 转IMU CSV为ROS bag
echo ""
echo "步骤2: 转换IMU CSV为ROS bag..."
if [ -f "AccelerometerUncalibrated.csv" ] && [ -f "GyroscopeUncalibrated.csv" ]; then
    python3 csv_to_imu.py -a AccelerometerUncalibrated.csv -g GyroscopeUncalibrated.csv -o imu.bag
    
    # 合并bag文件
    echo ""
    echo "步骤3: 合并相机和IMU bag文件..."
    if [ -f "cam.bag" ] && [ -f "imu.bag" ]; then
        # 使用Python脚本合并（rosbag merge命令不可用）
        python3 merge_bags.py -o calib.bag cam.bag imu.bag
        echo "合并完成: calib.bag"
    else
        echo "警告: 未找到cam.bag或imu.bag文件"
        if [ -f "cam.bag" ]; then
            cp cam.bag calib.bag
            echo "使用cam.bag作为calib.bag"
        fi
    fi
else
    echo "警告: 未找到IMU CSV文件，跳过IMU转换"
    cp cam.bag calib.bag
fi

# 3. 相机内参标定
echo ""
echo "步骤4: 相机内参标定..."

# 安装依赖（如果需要）
echo "安装 kalibr 运行依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq > /dev/null 2>&1
apt-get install -y -qq libsuitesparse-dev libspqr2 python3-igraph xvfb python3-tk > /dev/null 2>&1

# 设置库路径
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
ldconfig > /dev/null 2>&1

# 设置虚拟显示（无头环境）
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &
sleep 2

# 验证wxPython（如果未安装，给出提示）
if ! python3 -c "import wx" 2>/dev/null; then
    echo "安装 wxPython..."
    apt-get install -y -qq python3-wxgtk4.0 > /dev/null 2>&1
fi

# 加载kalibr（如果工作空间存在）
if [ -f "/kalibr_ws/devel/setup.bash" ]; then
    source /kalibr_ws/devel/setup.bash
    export PATH=$PATH:/kalibr_ws/devel/.private/kalibr/lib/kalibr
elif [ -f "/kalibr_ws/install/setup.bash" ]; then
    source /kalibr_ws/install/setup.bash
fi

if command -v kalibr_calibrate_cameras &> /dev/null; then
    kalibr_calibrate_cameras \
        --bag cam.bag \
        --topics /cam0/image_raw \
        --target target.yaml \
        --models pinhole-radtan \
        --show-extraction
    echo ""
    echo "相机内参标定完成！"
    echo "结果文件: cam0.yaml"
    echo "报告文件: report-cam.pdf"
    
    # 生成cam_chain.yaml
    if [ -f "cam0.yaml" ]; then
        echo ""
        echo "步骤5: 生成cam_chain.yaml..."
        python3 generate_cam_chain.py -i cam0.yaml -o cam_chain.yaml
    fi
else
    echo "警告: 未安装kalibr工具，请先安装:"
    echo "  sudo apt install ros-noetic-kalibr"
fi

echo ""
echo "=== 基础标定流程完成 ==="
echo ""
echo "下一步（可选）: 联合标定IMU和相机"
echo ""
echo "步骤A: IMU噪声参数标定（需要静止数据）"
echo "  ./run_imu_calib.sh"
echo ""
echo "步骤B: 相机-IMU联合标定"
echo "  kalibr_calibrate_imu_camera \\"
echo "      --bag calib.bag \\"
echo "      --cam cam_chain.yaml \\"
echo "      --imu imu0_imu_param.yaml \\"
echo "      --target target.yaml \\"
echo "      --show-extraction"

