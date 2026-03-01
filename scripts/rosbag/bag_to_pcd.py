#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
import os
import sys
import shutil
from pcd_utils import extract_valid_points, write_pcd_ascii
import json
from datetime import datetime

def find_closest_pose(poses, target_time):
    """
    找到最接近给定时间戳的位姿数据
    
    Args:
        poses (list): 位姿数据列表，每个元素为(timestamp, pose_msg)
        target_time (float): 目标时间戳
    
    Returns:
        tuple: 位姿数据(x, y, z, qw, qx, qy, qz)
    """
    closest_pose = min(poses, key=lambda x: abs(x[0] - target_time))
    pose_msg = closest_pose[1]
    position = pose_msg.pose.pose.position
    orientation = pose_msg.pose.pose.orientation
    return (
        position.x, position.y, position.z,
        orientation.w, orientation.x, orientation.y, orientation.z
    )

def convert_rosbag_to_pcd(bag_file, output_base_dir, lidar_topic, pose_topic):
    """
    将ROS bag文件中指定话题的点云数据转换为按序号命名的PCD文件
    
    Args:
        bag_file (str): ROS bag文件的路径
        output_base_dir (str): 输出基础目录
        lidar_topic (str): 要提取的点云话题名称
        pose_topic (str): 要提取的位姿话题名称
    
    Returns:
        bool: 转换是否成功
    """
    try:
        # 创建输出目录结构
        pcd_output_dir = os.path.join(output_base_dir, "pcd")
        if os.path.exists(pcd_output_dir):
            shutil.rmtree(pcd_output_dir)
        os.makedirs(pcd_output_dir, exist_ok=True)
        print(f"创建PCD输出目录: {pcd_output_dir}")
        
        # 打开ROS bag文件
        bag = rosbag.Bag(bag_file, 'r')
        print(f"打开bag文件: {bag_file}")
        
        # 检查话题是否存在
        topics = bag.get_type_and_topic_info()[1].keys()
        if lidar_topic not in topics or pose_topic not in topics:
            print(f"错误：话题不存在")
            print(f"可用话题: {list(topics)}")
            bag.close()
            return False
            
        # 先收集所有的位姿数据
        poses = []
        print("收集位姿数据...")
        for topic, msg, t in bag.read_messages(topics=[pose_topic]):
            poses.append((t.to_sec(), msg))
        print(f"共收集到 {len(poses)} 个位姿数据")
        
        # 处理点云数据和对应的位姿
        pose_data = {}
        frame_count = 0
        
        print(f"开始处理点云数据: {lidar_topic}")
        for topic, msg, t in bag.read_messages(topics=[lidar_topic]):
            # 生成5位数字格式的文件名(00000, 00001等)
            pcd_filename = f"{frame_count:05d}.pcd"
            pcd_path = os.path.join(pcd_output_dir, pcd_filename)
            
            # 获取最近的位姿数据
            closest_pose = find_closest_pose(poses, t.to_sec())
            pose_data[f"{frame_count:05d}"] = {
                "x": closest_pose[0],
                "y": closest_pose[1],
                "z": closest_pose[2],
                "qw": closest_pose[3],
                "qx": closest_pose[4],
                "qy": closest_pose[5],
                "qz": closest_pose[6]
            }
            
            try:
                valid_points, has_intensity = extract_valid_points(msg)
            except Exception as e:
                print(f"警告：无法读取第{frame_count}帧点云数据: {e}")
                frame_count += 1
                continue

            if len(valid_points) == 0:
                print(f"警告：第{frame_count}帧点云数据为空，跳过")
                frame_count += 1
                continue

            write_pcd_ascii(pcd_path, valid_points, has_intensity)
            
            frame_count += 1

        # 保存位姿数据到文本文件
        pose_file_path = os.path.join(output_base_dir, "pose_origin.json")
        with open(pose_file_path, 'w') as f:
            for i in range(frame_count):
                frame_id = f"{i:05d}"
                if frame_id in pose_data:
                    p = pose_data[frame_id]
                    f.write(f"{p['x']:.16f} {p['y']:.16f} {p['z']:.16f} {p['qw']:.16f} {p['qx']:.16f} {p['qy']:.16f} {p['qz']:.16f}\n")
        
        bag.close()
        return True
    
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        return False

def main():
    """
    主函数：将指定rosbag文件中的点云数据转换为PCD文件，并保存对应的位姿数据
    """
    # 配置参数(固定参数)
    bag_files = [
        "/mnt/d/rosbag/hba/double_0722.bag",    # 第一个bag文件
        # 如果有更多bag文件，可以在这里添加
    ]
    
    # 每个bag文件的输出目录
    output_dirs = [
        "/mnt/d/rosbag/hba/0722_xianfeng",      # 第一个bag文件的输出目录
        # 如果有更多bag文件，可以在这里添加对应的输出目录
    ]
    
    # 固定的topic配置
    lidar_topic = "/xianfeng/lidar"  # 要提取的点云话题
    pose_topic = "/xianfeng/pose"    # 要提取的位姿话题
    
    print("=== ROS Bag 转 PCD 文件工具 ===")
    print(f"点云话题: {lidar_topic}")
    print(f"位姿话题: {pose_topic}")
    print("="*50)
    
    # 处理每个bag文件
    for bag_file, output_base_dir in zip(bag_files, output_dirs):
        print(f"\n处理bag文件: {bag_file}")
        print(f"输出目录: {output_base_dir}")
        
        # 检查输入文件是否存在
        if not os.path.exists(bag_file):
            print(f"错误：输入文件不存在: {bag_file}")
            continue
        
        # 执行转换
        success = convert_rosbag_to_pcd(bag_file, output_base_dir, lidar_topic, pose_topic)
        
        if success:
            print(f"\n{os.path.basename(bag_file)} 转换成功！")
            print(f"数据已保存到: {output_base_dir}")
        else:
            print(f"\n{os.path.basename(bag_file)} 转换失败！")
            
    print("\n所有文件处理完成！")

if __name__ == "__main__":
    main()