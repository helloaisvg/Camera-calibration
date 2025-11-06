# Kalibr标定环境Docker镜像
FROM osrf/ros:noetic-desktop-full

# 设置工作目录
WORKDIR /workspace

# 安装基础依赖
# 设置非交互式环境，避免安装过程中的交互提示
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-noetic-rosbag \
    python3-pip \
    python3-opencv \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-wxgtk4.0 \
    build-essential \
    git \
    wget \
    libsuitesparse-dev \
    libcholmod3 \
    libspqr2 \
    python3-igraph \
    xvfb \
    python3-tk \
    python3-scipy \
    && rm -rf /var/lib/apt/lists/*

# 验证 wxPython 安装
RUN python3 -c "import wx; print('wxPython version:', wx.version())" || \
    (apt-get update && apt-get install -y --reinstall python3-wxgtk4.0 && \
     python3 -c "import wx; print('wxPython version:', wx.version())")

# 初始化rosdep
RUN rosdep init || true && \
    rosdep update || true

# 安装kalibr（从源码编译）
RUN mkdir -p /kalibr_ws/src && \
    cd /kalibr_ws/src && \
    git clone https://github.com/ethz-asl/kalibr.git || \
    (cd kalibr && git pull) || true && \
    cd /kalibr_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin build -DCMAKE_BUILD_TYPE=Release -j4 || \
    (catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
     catkin build -j4) || true

# 安装imu_utils（从源码编译）
RUN mkdir -p /imu_utils_ws/src && \
    cd /imu_utils_ws/src && \
    git clone https://github.com/gaowenliang/imu_utils.git || \
    (cd imu_utils && git pull) || true && \
    cd /imu_utils_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make || catkin build || true

# 安装Python依赖
RUN pip3 install pyyaml

# 初始化ROS环境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /kalibr_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "source /imu_utils_ws/devel/setup.bash" >> /root/.bashrc

# 默认命令
CMD ["bash"]

