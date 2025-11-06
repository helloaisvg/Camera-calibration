#!/usr/bin/env python3
# 合并ROS bag文件

import rosbag
import argparse
import sys
from rospy import Time

def merge_bags(input_bags, output_bag):
    """合并多个bag文件到一个bag文件，统一时间基准"""
    print(f"合并 {len(input_bags)} 个bag文件到: {output_bag}")
    
    # 第一步：扫描所有bag文件，找到最早的时间戳
    print("扫描时间戳范围...")
    min_times = {}
    for bag_file in input_bags:
        try:
            with rosbag.Bag(bag_file, 'r') as inbag:
                min_time = None
                for topic, msg, t in inbag.read_messages():
                    if min_time is None or t < min_time:
                        min_time = t
                if min_time is not None:
                    min_times[bag_file] = min_time
                    print(f"  {bag_file}: 最早时间戳 = {min_time.to_sec():.6f}s")
        except Exception as e:
            print(f"  ✗ 扫描错误: {bag_file} - {e}")
    
    if not min_times:
        print("错误：没有找到有效的时间戳")
        return
    
    # 找到全局最早的时间戳
    global_min_time = min(min_times.values())
    print(f"全局最早时间戳: {global_min_time.to_sec():.6f}s")
    
    # 第二步：合并bag文件，统一时间基准
    with rosbag.Bag(output_bag, 'w') as outbag:
        for bag_file in input_bags:
            print(f"  处理: {bag_file}")
            try:
                with rosbag.Bag(bag_file, 'r') as inbag:
                    count = 0
                    bag_min_time = min_times.get(bag_file)
                    if bag_min_time is None:
                        continue
                    time_offset = global_min_time - bag_min_time
                    offset_sec = time_offset.to_sec()
                    
                    for topic, msg, t in inbag.read_messages():
                        # 计算相对于全局最早时间戳的偏移
                        new_time_sec = t.to_sec() + offset_sec
                        new_time = Time.from_sec(new_time_sec)
                        
                        # 更新消息头中的时间戳
                        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                            msg.header.stamp = new_time
                        
                        outbag.write(topic, msg, new_time)
                        count += 1
                    print(f"  ✓ 完成: {bag_file} ({count} 条消息)")
            except Exception as e:
                print(f"  ✗ 错误: {bag_file} - {e}")
    
    print(f"合并完成: {output_bag}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='合并ROS bag文件')
    parser.add_argument('-o', '--output', required=True, help='输出bag文件路径')
    parser.add_argument('inputs', nargs='+', help='输入bag文件路径')
    
    args = parser.parse_args()
    merge_bags(args.inputs, args.output)

