# 相机和IMU标定工具

基于Kalibr的相机内参标定和相机-IMU联合标定工具集。

##  功能

-  相机内参标定
-  IMU噪声参数标定
-  相机-IMU联合标定（重投影误差：1.10px）

##  快速开始

### 使用Docker

```bash
# 1. 构建Docker镜像
docker build -t kalibr-calibration .

# 2. 运行完整标定流程
./docker_run.sh "./run_full_calibration.sh"
```

### 分步运行

```bash
# 1. 相机内参标定
./docker_run.sh "./run_calib.sh"

# 2. 相机-IMU联合标定
./docker_run.sh "./run_joint_calib.sh"
```

##  项目结构

### 核心脚本

| 文件 | 说明 |
|------|------|
| `video_to_bag.py` | 将视频文件转换为ROS bag格式 |
| `csv_to_imu.py` | 将IMU CSV数据转换为ROS bag格式 |
| `merge_bags.py` | 合并多个ROS bag文件，统一时间基准 |
| `generate_cam_chain.py` | 从相机内参生成cam_chain.yaml文件 |

### 运行脚本

| 文件 | 说明 |
|------|------|
| `run_calib.sh` | 相机内参标定（视频转bag + IMU转bag + 相机标定） |
| `run_joint_calib.sh` | 相机-IMU联合标定 |
| `run_full_calibration.sh` | 完整标定流程（一键运行） |

### 配置文件

| 文件 | 说明 |
|------|------|
| `target.yaml` | 标定板配置（AprilGrid，6x6，2.5cm标签） |
| `cam_chain_fixed.yaml` | 相机-IMU联合标定初始配置 |
| `imu0_imu_param.yaml` | IMU噪声参数 |

### 标定结果

#### 相机内参标定结果
- `cam-camchain.yaml` - 相机内参配置
- `cam-results-cam.txt` - 详细标定结果
- `cam-report-cam.pdf` - 可视化标定报告

#### 相机-IMU联合标定结果（最终结果）
- `calib_fixed-camchain-imucam.yaml` - **最终标定结果**（相机-IMU外参、时间偏移）
- `calib_fixed-results-imucam.txt` - 详细联合标定结果
- `calib_fixed-report-imucam.pdf` - 可视化联合标定报告

**标定精度：**
- 重投影误差：1.10 px（均值），1.03 px（中位数）
- 时间偏移：-0.050 秒

##  手动步骤

### 步骤1: 数据转换

```bash
# 视频转ROS bag
python3 video_to_bag.py -i video.mp4 -o cam.bag --offset 0.05

# IMU CSV转ROS bag
python3 csv_to_imu.py -a AccelerometerUncalibrated.csv \
                      -g GyroscopeUncalibrated.csv \
                      -o imu.bag

# 合并bag文件（统一时间基准）
python3 merge_bags.py -o calib_fixed.bag cam.bag imu.bag
```

### 步骤2: 相机内参标定

```bash
kalibr_calibrate_cameras \
    --bag cam.bag \
    --topics /cam0/image_raw \
    --target target.yaml \
    --models pinhole-radtan
```

### 步骤3: 生成相机链配置

```bash
python3 generate_cam_chain.py -i cam0.yaml -o cam_chain.yaml
```

### 步骤4: 相机-IMU联合标定

```bash
kalibr_calibrate_imu_camera \
    --bag calib_fixed.bag \
    --cam cam_chain_fixed.yaml \
    --imu imu0_imu_param.yaml \
    --target target.yaml \
    --max-iter 20 \
    --timeoffset-padding 0.15
```

##  依赖

### Docker环境

项目使用Docker容器提供完整的ROS Noetic + Kalibr环境：

```bash
# 构建镜像
docker build -t kalibr-calibration .

# 运行容器
./docker_run.sh "command"
```

### 本地环境（Ubuntu 20.04 + ROS Noetic）

```bash
# ROS依赖
sudo apt install ros-noetic-kalibr \
                 ros-noetic-rosbag \
                 ros-noetic-imu-utils

# Python依赖
pip3 install opencv-python pyyaml rospkg

# 初始化ROS环境
source /opt/ros/noetic/setup.bash
```

##  配置说明

### 标定板配置（target.yaml）

```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.025      # 标签边长（米）
tagSpacing: 0.3     # 标签间距比例
```

### 相机链配置（cam_chain_fixed.yaml）

包含相机内参、初始外参矩阵和时间偏移：

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [fx, fy, cx, cy]
  distortion_coeffs: [k1, k2, p1, p2]
  timeshift_cam_to_imu: 0.05
  T_cam_imu: [[...], [...], [...], [...]]
```

##  注意事项

1. **时间同步**：视频和IMU数据需要时间戳同步，`merge_bags.py`会自动统一时间基准
2. **标定板参数**：根据实际标定板调整`target.yaml`中的`tagSize`和`tagSpacing`
3. **初始外参**：联合标定需要合理的初始外参估计，见`cam_chain_fixed.yaml`
4. **时间偏移**：如果标定不收敛，尝试调整`timeshift_cam_to_imu`和视频转换时的`--offset`参数



##  标定结果示例

### 相机内参
- 焦距：fx=653.42, fy=646.94
- 主点：cx=260.20, cy=259.47
- 畸变系数：[0.235, -0.693, 0.0014, 0.0010]

### 相机-IMU外参
```
T_cam_imu = [[ 0.9999, -0.0031, -0.0106,  0.0  ]
             [-0.0030, -0.9999,  0.0025,  0.0  ]
             [-0.0106, -0.0025, -0.9999,  0.0  ]
             [ 0.0,     0.0,     0.0,     1.0  ]]
```

### 时间偏移
- 相机到IMU：-0.050 秒

