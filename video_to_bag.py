#!/usr/bin/env python3

import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rospy import Time as ROSTime
import argparse
import os

parser = argparse.ArgumentParser(description='将视频文件转换为ROS bag格式')
parser.add_argument('-i', '--input', required=True, help='输入视频文件路径')
parser.add_argument('-o', '--output', default='cam.bag', help='输出bag文件路径')
parser.add_argument('-t', '--topic', default='/cam0/image_raw', help='图像话题名称')
parser.add_argument('-f', '--fps', type=float, default=30.0, help='视频帧率（默认30fps）')
parser.add_argument('--offset', type=float, default=0.0, help='时间戳偏移（秒）')
args = parser.parse_args()

if not os.path.exists(args.input):
    print(f"错误：找不到视频文件 {args.input}")
    exit(1)

cap = cv2.VideoCapture(args.input)
if not cap.isOpened():
    print(f"错误：无法打开视频文件 {args.input}")
    exit(1)

# 获取实际帧率
fps = cap.get(cv2.CAP_PROP_FPS)
if fps > 0:
    args.fps = fps
    print(f"检测到视频帧率: {fps} fps")

bridge = CvBridge()
bag = rosbag.Bag(args.output, 'w')

frame_idx = 0
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"总帧数: {total_frames}")

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # 时间戳：从 offset 开始
        stamp = ROSTime.from_sec(frame_idx / args.fps + args.offset)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "cam0"
        
        bag.write(args.topic, msg, stamp)
        
        if frame_idx % 30 == 0:
            print(f"处理中: 帧 {frame_idx}/{total_frames} @ {stamp.to_sec():.3f}s")
        
        frame_idx += 1
except Exception as e:
    print(f"错误：{e}")
finally:
    cap.release()
    bag.close()

print(f"完成！已保存 {args.output}")
print(f"总帧数: {frame_idx}")

