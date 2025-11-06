#!/bin/bash
# Docker安装脚本

echo "=== 安装Docker ==="
echo ""
echo "正在安装Docker..."

# 检查是否已有docker
if command -v docker &> /dev/null; then
    echo "✓ Docker已安装: $(docker --version)"
    exit 0
fi

# 安装Docker
echo "执行安装命令..."
echo "请手动运行以下命令:"
echo ""
echo "sudo apt update"
echo "sudo apt install -y docker.io docker-compose"
echo ""
echo "或者:"
echo "sudo snap install docker"
echo ""
echo "安装完成后，运行以下命令启动Docker服务:"
echo "sudo systemctl start docker"
echo "sudo systemctl enable docker"
