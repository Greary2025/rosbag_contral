#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import sys

"""
检查bag文件中点云和位姿数据的时间戳
"""

def main():
    if len(sys.argv) < 2:
        print("用法: python3 check_timestamps.py <bag_file>")
        return
    
    bag_file = sys.argv[1]
    
    # 打开bag文件
    bag = rosbag.Bag(bag_file, 'r')
    
    # 获取可用话题
    topics = bag.get_type_and_topic_info()[1].keys()
    print(f"可用话题: {list(topics)}")
    
    # 查找点云和位姿话题
    lidar_topic = '/xianfeng/lidar'
    pose_topic = '/xianfeng/pose'
    
    # 检查话题是否存在
    if lidar_topic not in topics:
        print(f"错误: 点云话题 {lidar_topic} 不存在")
        return
    
    if pose_topic not in topics:
        print(f"错误: 位姿话题 {pose_topic} 不存在")
        return
    
    # 读取前10个点云帧的时间戳
    print("\n前10个点云帧的时间戳:")
    lidar_count = 0
    lidar_timestamps = []
    for _, _, t in bag.read_messages(topics=[lidar_topic]):
        lidar_timestamps.append(t.to_sec())
        print(f"点云帧 {lidar_count}: 时间戳 = {t.to_sec():.9f}")
        lidar_count += 1
        if lidar_count >= 10:
            break
    
    # 读取前15个位姿帧的时间戳
    print("\n前15个位姿帧的时间戳:")
    pose_count = 0
    pose_timestamps = []
    for _, _, t in bag.read_messages(topics=[pose_topic]):
        pose_timestamps.append(t.to_sec())
        print(f"位姿帧 {pose_count}: 时间戳 = {t.to_sec():.9f}")
        pose_count += 1
        if pose_count >= 15:
            break
    
    # 关闭bag文件
    bag.close()
    
    # 检查CSV文件中的时间戳
    print("\n请使用以下命令查看CSV文件中的时间戳:")
    print(f"head -n 15 <csv_file_path>")

if __name__ == "__main__":
    main()