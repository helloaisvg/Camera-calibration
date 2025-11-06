#!/bin/bash
# Docker运行脚本 - 在容器中运行标定任务

set -e

IMAGE_NAME="kalibr-calibration"
CONTAINER_NAME="kalibr-calib-$(date +%s)"

# 检查Docker是否安装
if ! command -v docker &> /dev/null; then
    echo "错误: 未安装Docker"
    echo "安装方法: sudo apt install docker.io"
    exit 1
fi

# 检测是否需要使用sudo
DOCKER_CMD="docker"
if ! docker ps &> /dev/null; then
    echo "检测到权限问题，尝试使用sudo..."
    DOCKER_CMD="sudo docker"
    if ! $DOCKER_CMD ps &> /dev/null; then
        echo "错误: 无法访问Docker，请检查权限"
        echo "运行: sudo usermod -aG docker $USER"
        echo "然后重新登录或运行: newgrp docker"
        exit 1
    fi
fi

# 检查镜像是否存在，不存在则构建
if ! $DOCKER_CMD images | grep -q "$IMAGE_NAME"; then
    echo "构建Docker镜像..."
    $DOCKER_CMD build -t $IMAGE_NAME .
fi

# 运行容器
echo "启动Docker容器..."
echo "容器名称: $CONTAINER_NAME"
echo "工作目录: $(pwd)"
echo ""

# 检查是否有TTY
if [ -t 0 ]; then
    TTY_FLAG="-it"
else
    TTY_FLAG="-i"
fi

$DOCKER_CMD run $TTY_FLAG --rm \
    --name $CONTAINER_NAME \
    -v "$(pwd):/workspace" \
    -v "$(pwd)/kalibr_ws:/kalibr_ws" \
    -w /workspace \
    $IMAGE_NAME \
    bash -c "source /opt/ros/noetic/setup.bash && if [ -f /kalibr_ws/devel/setup.bash ]; then source /kalibr_ws/devel/setup.bash; export PATH=\$PATH:/kalibr_ws/devel/.private/kalibr/lib/kalibr; elif [ -f /kalibr_ws/install/setup.bash ]; then source /kalibr_ws/install/setup.bash; fi && $@"

echo ""
echo "容器已退出"

