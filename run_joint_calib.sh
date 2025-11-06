#!/bin/bash
set -e

echo "=== 相机-IMU联合标定 ==="
echo ""

# 检查必要文件
if [ ! -f "calib.bag" ]; then
    echo "错误：找不到calib.bag文件"
    echo "请先运行: ./run_calib.sh"
    exit 1
fi

if [ ! -f "cam_chain.yaml" ]; then
    echo "错误：找不到cam_chain.yaml文件"
    if [ -f "cam0.yaml" ]; then
        echo "正在生成cam_chain.yaml..."
        python3 generate_cam_chain.py -i cam0.yaml -o cam_chain.yaml
    else
        echo "请先完成相机内参标定"
        exit 1
    fi
fi

if [ ! -f "imu0_imu_param.yaml" ]; then
    echo "警告：找不到imu0_imu_param.yaml文件"
    echo "请先运行IMU噪声参数标定: ./run_imu_calib.sh"
    echo "使用默认IMU参数..."
fi

if [ ! -f "target.yaml" ]; then
    echo "错误：找不到target.yaml文件"
    exit 1
fi

# 安装依赖（如果需要）
echo "安装 kalibr 运行依赖..."
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq > /dev/null 2>&1
apt-get install -y -qq libsuitesparse-dev libspqr2 python3-igraph xvfb python3-tk python3-scipy > /dev/null 2>&1

# 设置库路径
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH
# 查找并添加SuiteSparse库路径
if [ -d "/usr/lib/x86_64-linux-gnu" ]; then
    export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
fi
# 查找libcholmod.so.3的实际位置
CHOLMOD_LIB=$(find /usr/lib -name "libcholmod.so*" 2>/dev/null | head -1)
if [ -n "$CHOLMOD_LIB" ]; then
    CHOLMOD_DIR=$(dirname "$CHOLMOD_LIB")
    export LD_LIBRARY_PATH=$CHOLMOD_DIR:$LD_LIBRARY_PATH
    echo "找到SuiteSparse库: $CHOLMOD_DIR"
fi
ldconfig > /dev/null 2>&1

# 设置虚拟显示（无头环境）
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &
sleep 2

# 运行联合标定
echo "开始相机-IMU联合标定..."
echo ""

if command -v kalibr_calibrate_imu_camera &> /dev/null; then
    kalibr_calibrate_imu_camera \
        --bag calib.bag \
        --cam cam_chain.yaml \
        --imu imu0_imu_param.yaml \
        --target target.yaml \
        --timeoffset-padding 0.1
    
    echo ""
    echo "=== 联合标定完成 ==="
    echo "结果文件：camchain-imucam-calib_results.yaml"
else
    echo "错误：未安装kalibr工具"
    echo "安装方法: sudo apt install ros-noetic-kalibr"
    exit 1
fi

