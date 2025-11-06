#!/usr/bin/env python3

"""
从相机内参标定结果生成cam_chain.yaml文件
用于相机-IMU联合标定
"""

import yaml
import sys
import argparse

parser = argparse.ArgumentParser(description='从相机内参生成cam_chain.yaml')
parser.add_argument('-i', '--input', default='cam0.yaml', help='相机内参YAML文件')
parser.add_argument('-o', '--output', default='cam_chain.yaml', help='输出cam_chain.yaml文件')
args = parser.parse_args()

try:
    with open(args.input, 'r') as f:
        cam_data = yaml.safe_load(f)
except FileNotFoundError:
    print(f"错误：找不到文件 {args.input}")
    print("请先运行相机内参标定：kalibr_calibrate_cameras")
    sys.exit(1)

# 提取相机参数
if 'cam0' in cam_data:
    cam0 = cam_data['cam0']
elif len(cam_data) > 0:
    # 如果没有cam0键，取第一个键
    first_key = list(cam_data.keys())[0]
    cam0 = cam_data[first_key]
else:
    print("错误：无法解析相机内参文件")
    sys.exit(1)

# 构建cam_chain.yaml
cam_chain = {
    'cam0': {
        'camera_model': cam0.get('camera_model', 'pinhole'),
        'intrinsics': cam0.get('intrinsics', [458.1, 457.9, 321.5, 241.8]),
        'distortion_coefficients': cam0.get('distortion_coeffs', [0.0, 0.0, 0.0, 0.0]),
        'distortion_model': cam0.get('distortion_model', 'radtan'),
        'T_cam_imu': [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
    }
}

# 保存文件
with open(args.output, 'w') as f:
    yaml.dump(cam_chain, f, default_flow_style=False, allow_unicode=True)

print(f"已生成 {args.output}")
print("注意：T_cam_imu 是单位矩阵，联合标定时会优化这个值")

