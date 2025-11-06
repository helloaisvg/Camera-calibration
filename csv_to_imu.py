#!/usr/bin/env python3

import csv
import rosbag
from sensor_msgs.msg import Imu
from rospy import Time as ROSTime
import argparse
import os
import bisect

parser = argparse.ArgumentParser(description='将IMU CSV文件转换为ROS bag格式')
parser.add_argument('-a', '--accel', help='加速度计CSV文件路径')
parser.add_argument('-g', '--gyro', help='陀螺仪CSV文件路径')
parser.add_argument('-o', '--output', default='imu.bag', help='输出bag文件路径')
parser.add_argument('-t', '--topic', default='/imu0', help='IMU话题名称')
args = parser.parse_args()

# 读取加速度计数据
accel_data = {}
if args.accel and os.path.exists(args.accel):
    print(f"读取加速度计数据: {args.accel}")
    with open(args.accel, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # 时间戳（纳秒）
                t_ns = int(row['time'])
                t_sec = t_ns / 1e9
                
                # x, y, z 加速度（注意CSV中列的顺序可能是z,y,x）
                ax = float(row.get('x', 0))
                ay = float(row.get('y', 0))
                az = float(row.get('z', 0))
                
                accel_data[t_ns] = [ax, ay, az]
            except (ValueError, KeyError) as e:
                continue
    print(f"  读取到 {len(accel_data)} 条加速度计数据")

# 读取陀螺仪数据
gyro_data = {}
if args.gyro and os.path.exists(args.gyro):
    print(f"读取陀螺仪数据: {args.gyro}")
    with open(args.gyro, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                # 时间戳（纳秒）
                t_ns = int(row['time'])
                t_sec = t_ns / 1e9
                
                # x, y, z 角速度
                gx = float(row.get('x', 0))
                gy = float(row.get('y', 0))
                gz = float(row.get('z', 0))
                
                gyro_data[t_ns] = [gx, gy, gz]
            except (ValueError, KeyError) as e:
                continue
    print(f"  读取到 {len(gyro_data)} 条陀螺仪数据")

if not accel_data and not gyro_data:
    print("错误：未找到有效的IMU数据")
    exit(1)

# 合并时间戳 - 使用更智能的策略
# 如果两个数据集都有数据，使用时间戳的并集
# 如果只有一个数据集，使用该数据集的时间戳
if accel_data and gyro_data:
    all_timestamps = sorted(set(accel_data.keys()) | set(gyro_data.keys()))
elif accel_data:
    all_timestamps = sorted(accel_data.keys())
elif gyro_data:
    all_timestamps = sorted(gyro_data.keys())
else:
    print("错误：没有可用数据")
    exit(1)

print(f"合并后的时间戳数量: {len(all_timestamps)}")
if all_timestamps:
    print(f"时间范围: {min(all_timestamps)/1e9:.3f}s ~ {max(all_timestamps)/1e9:.3f}s")

bag = rosbag.Bag(args.output, 'w')

# 协方差（Kalibr要求）
cov_acc = 1e-3
cov_gyro = 1e-5

# 为了更高效地查找最近的时间戳，创建排序列表
accel_keys_sorted = sorted(accel_data.keys()) if accel_data else []
gyro_keys_sorted = sorted(gyro_data.keys()) if gyro_data else []

count = 0
for t_ns in all_timestamps:
    try:
        t_sec = t_ns / 1e9
        
        imu = Imu()
        imu.header.stamp = ROSTime.from_sec(t_sec)
        imu.header.frame_id = "imu0"
        
        # 获取最近的加速度计数据（使用二分查找优化）
        if accel_data:
            if t_ns in accel_data:
                ax, ay, az = accel_data[t_ns]
            else:
                # 使用二分查找找最近的时间戳
                idx = bisect.bisect_left(accel_keys_sorted, t_ns)
                candidates = []
                if idx > 0:
                    candidates.append(accel_keys_sorted[idx - 1])
                if idx < len(accel_keys_sorted):
                    candidates.append(accel_keys_sorted[idx])
                
                if candidates:
                    closest_ts = min(candidates, key=lambda x: abs(x - t_ns))
                    time_diff = abs(closest_ts - t_ns)
                    if time_diff < 5e8:  # 500ms内接受
                        ax, ay, az = accel_data[closest_ts]
                    else:
                        ax, ay, az = 0, 0, 0
                else:
                    ax, ay, az = 0, 0, 0
        else:
            ax, ay, az = 0, 0, 0
        
        # 获取最近的陀螺仪数据
        if gyro_data:
            if t_ns in gyro_data:
                gx, gy, gz = gyro_data[t_ns]
            else:
                idx = bisect.bisect_left(gyro_keys_sorted, t_ns)
                candidates = []
                if idx > 0:
                    candidates.append(gyro_keys_sorted[idx - 1])
                if idx < len(gyro_keys_sorted):
                    candidates.append(gyro_keys_sorted[idx])
                
                if candidates:
                    closest_ts = min(candidates, key=lambda x: abs(x - t_ns))
                    time_diff = abs(closest_ts - t_ns)
                    if time_diff < 5e8:  # 500ms内接受
                        gx, gy, gz = gyro_data[closest_ts]
                    else:
                        gx, gy, gz = 0, 0, 0
                else:
                    gx, gy, gz = 0, 0, 0
        else:
            gx, gy, gz = 0, 0, 0
        
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        
        # 协方差矩阵
        imu.linear_acceleration_covariance = [cov_acc, 0, 0, 
                                               0, cov_acc, 0, 
                                               0, 0, cov_acc]
        imu.angular_velocity_covariance = [cov_gyro, 0, 0,
                                            0, cov_gyro, 0,
                                            0, 0, cov_gyro]
        
        bag.write(args.topic, imu, imu.header.stamp)
        count += 1
        
        if count % 1000 == 0:
            print(f"  已处理 {count}/{len(all_timestamps)} 条数据")
            
    except Exception as e:
        continue

bag.close()
print(f"完成！已保存 {args.output}")
print(f"总数据条数: {count}")

