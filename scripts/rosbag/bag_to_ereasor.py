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
import random
from datetime import datetime

def convert_rosbag_to_pcd(bag_file, output_dir, topic_name):
    """
    将ROS bag文件中指定话题的点云数据转换为按序号命名的PCD文件
    
    Args:
        bag_file (str): ROS bag文件的路径
        output_dir (str): 输出PCD文件的目录
        topic_name (str): 要提取的点云话题名称
    
    Returns:
        bool: 转换是否成功
        int: 处理的帧数
    """
    try:
        # 清空输出目录（如果存在）
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
            print(f"已清空输出目录: {output_dir}")
        
        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)
        print(f"创建输出目录: {output_dir}")
        
        # 打开ROS bag文件
        bag = rosbag.Bag(bag_file, 'r')
        print(f"打开bag文件: {bag_file}")
        
        # 检查话题是否存在
        topics = bag.get_type_and_topic_info()[1].keys()
        if topic_name not in topics:
            print(f"错误：话题 {topic_name} 在bag文件中不存在")
            print(f"可用话题: {list(topics)}")
            bag.close()
            return False, 0
        
        print(f"开始提取话题: {topic_name}")
        
        # 处理每一帧点云数据
        frame_count = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # 生成6位数字格式的文件名
            pcd_filename = f"{frame_count:06d}.pcd"
            pcd_path = os.path.join(output_dir, pcd_filename)
            
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
            
            print(f"保存第{frame_count}帧: {pcd_filename} ({len(valid_points)}个点)")
            frame_count += 1
        
        bag.close()
        print(f"转换完成！共处理{frame_count}帧点云数据")
        return True, frame_count
    
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        return False, 0

def process_rosbag_data(bag_file, pcd_output_dir, pose_output_path, pcd_topic_name, pose_topic_name):
    """
    从ROS bag文件中提取点云数据并转换为PCD文件，同时提取对应的里程计数据保存为CSV
    
    Args:
        bag_file (str): ROS bag文件的路径
        pcd_output_dir (str): 输出PCD文件的目录
        pose_output_path (str): 输出CSV文件的路径
        pcd_topic_name (str): 要提取的点云话题名称
        pose_topic_name (str): 要提取的里程计话题名称
    
    Returns:
        bool: 处理是否成功
        int: 处理的点云帧数
        int: 匹配的里程计帧数
    """
    try:
        # 清空点云输出目录（如果存在）
        if os.path.exists(pcd_output_dir):
            shutil.rmtree(pcd_output_dir)
            print(f"已清空输出目录: {pcd_output_dir}")
        
        # 创建输出目录
        os.makedirs(pcd_output_dir, exist_ok=True)
        print(f"创建输出目录: {pcd_output_dir}")
        
        # 确保位姿输出目录存在
        pose_output_dir = os.path.dirname(pose_output_path)
        os.makedirs(pose_output_dir, exist_ok=True)
        
        # 打开ROS bag文件
        bag = rosbag.Bag(bag_file, 'r')
        print(f"打开bag文件: {bag_file}")
        
        # 检查话题是否存在
        topics = bag.get_type_and_topic_info()[1].keys()
        if pcd_topic_name not in topics:
            print(f"错误：点云话题 {pcd_topic_name} 在bag文件中不存在")
            print(f"可用话题: {list(topics)}")
            bag.close()
            return False, 0, 0
        
        if pose_topic_name not in topics:
            print(f"错误：里程计话题 {pose_topic_name} 在bag文件中不存在")
            print(f"可用话题: {list(topics)}")
            bag.close()
            return False, 0, 0
        
        print(f"开始提取点云话题: {pcd_topic_name}")
        print(f"开始提取里程计话题: {pose_topic_name}")
        
        # 首先读取所有里程计数据并按时间戳排序
        pose_data = []
        for topic, msg, t in bag.read_messages(topics=[pose_topic_name]):
            # 检查消息类型 - 支持标准Odometry和_nav_msgs__Odometry
            if not (isinstance(msg, Odometry) or type(msg).__name__ == '_nav_msgs__Odometry'):
                print(f"警告：消息类型不是Odometry或_nav_msgs__Odometry，而是{type(msg).__name__}，跳过")
                continue
            
            # 提取位置和姿态数据
            pos = msg.pose.pose.position
            quat = msg.pose.pose.orientation
            
            # 获取时间戳（秒级）
            timestamp = t.to_sec()
            
            # 存储里程计数据
            pose_data.append({
                'timestamp': timestamp,
                'pos_x': pos.x,
                'pos_y': pos.y,
                'pos_z': pos.z,
                'quat_x': quat.x,
                'quat_y': quat.y,
                'quat_z': quat.z,
                'quat_w': quat.w
            })
        
        # 按时间戳排序
        pose_data.sort(key=lambda x: x['timestamp'])
        print(f"读取了{len(pose_data)}帧里程计数据")
        
        if len(pose_data) == 0:
            print("错误：未读取到任何里程计数据")
            bag.close()
            return False, 0, 0
        
        # 打开CSV输出文件
        with open(pose_output_path, 'w') as f:
            # 写入CSV文件头 - 确保格式与数据行一致（无空格）
            f.write("index,timestamp,x,y,z,qx,qy,qz,qw\n")
            
            # 处理点云数据并匹配最接近的里程计数据
            pcd_frame_count = 0
            pose_frame_count = 0
            
            for topic, msg, t in bag.read_messages(topics=[pcd_topic_name]):
                # 生成6位数字格式的文件名
                pcd_filename = f"{pcd_frame_count:06d}.pcd"
                pcd_path = os.path.join(pcd_output_dir, pcd_filename)
                
                # 获取点云时间戳
                pcd_timestamp = t.to_sec()
                
                try:
                    valid_points, has_intensity = extract_valid_points(msg)
                except Exception as e:
                    print(f"警告：无法读取第{pcd_frame_count}帧点云数据: {e}")
                    pcd_frame_count += 1
                    continue

                if len(valid_points) == 0:
                    print(f"警告：第{pcd_frame_count}帧点云数据为空，跳过")
                    pcd_frame_count += 1
                    continue

                if has_intensity:
                    print(f"第{pcd_frame_count}帧: 点云包含强度信息（intensity）")
                else:
                    print(f"第{pcd_frame_count}帧: 点云不包含强度信息")

                write_pcd_ascii(pcd_path, valid_points, has_intensity)
                
                # 查找最接近的里程计数据
                closest_pose = None
                min_time_diff = float('inf')
                
                for pose in pose_data:
                    time_diff = abs(pose['timestamp'] - pcd_timestamp)
                    if time_diff < min_time_diff:
                        min_time_diff = time_diff
                        closest_pose = pose
                
                # 写入对应的里程计数据到CSV
                if closest_pose:
                    f.write(f"{pcd_frame_count},{closest_pose['timestamp']:.9f},{closest_pose['pos_x']:.9f},{closest_pose['pos_y']:.9f},{closest_pose['pos_z']:.9f},{closest_pose['quat_x']:.9f},{closest_pose['quat_y']:.9f},{closest_pose['quat_z']:.9f},{closest_pose['quat_w']:.9f}\n")
                    pose_frame_count += 1
                    
                    # 打印调试信息
                    if pcd_frame_count % 100 == 0:
                        print(f"保存第{pcd_frame_count}帧: {pcd_filename} ({len(valid_points)}个点), 匹配时间戳={closest_pose['timestamp']:.3f}, 位置=({closest_pose['pos_x']:.3f}, {closest_pose['pos_y']:.3f}, {closest_pose['pos_z']:.3f})")
                else:
                    print(f"警告：无法为第{pcd_frame_count}帧点云找到对应的里程计数据")
                
                pcd_frame_count += 1
            
            bag.close()
            print(f"处理完成！共处理{pcd_frame_count}帧点云数据，匹配了{pose_frame_count}帧里程计数据")
            return True, pcd_frame_count, pose_frame_count
    
    except Exception as e:
        print(f"处理过程中发生错误: {e}")
        return False, 0, 0

def extract_pose_from_rosbag(bag_file, output_path, topic_name):
    """
    从ROS bag文件中提取里程计信息并保存为CSV格式的位姿数据
    (已弃用，请使用process_rosbag_data函数)
    
    Args:
        bag_file (str): ROS bag文件的路径
        output_path (str): 输出CSV文件的路径
        topic_name (str): 要提取的里程计话题名称
    
    Returns:
        bool: 提取是否成功
        int: 处理的帧数
    """
    try:
        # 确保输出目录存在
        output_dir = os.path.dirname(output_path)
        os.makedirs(output_dir, exist_ok=True)
        
        # 打开ROS bag文件
        bag = rosbag.Bag(bag_file, 'r')
        print(f"打开bag文件: {bag_file}")
        
        # 检查话题是否存在
        topics = bag.get_type_and_topic_info()[1].keys()
        if topic_name not in topics:
            print(f"错误：话题 {topic_name} 在bag文件中不存在")
            print(f"可用话题: {list(topics)}")
            bag.close()
            return False, 0
        
        print(f"开始提取话题: {topic_name}")
        
        # 打开输出文件
        with open(output_path, 'w') as f:
            # 写入CSV文件头 - 确保格式与数据行一致（无空格）
            f.write("index,timestamp,x,y,z,qx,qy,qz,qw\n")
            
            # 处理每一帧里程计数据
            frame_count = 0
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                # 检查消息类型 - 支持标准Odometry和_nav_msgs__Odometry
                if not (isinstance(msg, Odometry) or type(msg).__name__ == '_nav_msgs__Odometry'):
                    print(f"警告：消息类型不是Odometry或_nav_msgs__Odometry，而是{type(msg).__name__}，跳过")
                    continue
                
                # 打印消息类型信息（调试用）
                if frame_count == 0:
                    print(f"消息类型: {type(msg).__name__}")
                    print(f"消息结构: {dir(msg)}")
                    try:
                        print(f"位置信息: {msg.pose.pose.position}")
                        print(f"姿态信息: {msg.pose.pose.orientation}")
                    except Exception as e:
                        print(f"无法访问位置或姿态信息: {e}")
                
                # 提取位置和姿态数据
                pos = msg.pose.pose.position
                quat = msg.pose.pose.orientation
                
                # 获取时间戳（秒级）
                timestamp = t.to_sec()
                
                # 写入CSV行 - 确保没有空格
                f.write(f"{frame_count},{timestamp:.9f},{pos.x:.9f},{pos.y:.9f},{pos.z:.9f},{quat.x:.9f},{quat.y:.9f},{quat.z:.9f},{quat.w:.9f}\n")
                
                # 打印调试信息
                if frame_count % 100 == 0:
                    print(f"处理第{frame_count}帧里程计数据: 时间戳={timestamp:.3f}, 位置=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                
                frame_count += 1
            
            bag.close()
            print(f"位姿数据提取完成！共处理{frame_count}帧")
            return True, frame_count
    
    except Exception as e:
        print(f"提取位姿数据过程中发生错误: {e}")
        return False, 0

def main():
    """
    主函数：将指定rosbag文件中的点云数据转换为PCD文件，并提取位姿数据
    
    使用方法：
        python bag_to_ereasor.py [bag_file] [output_dir]
        
        参数：
            bag_file: 可选，ROS bag文件路径，默认为当前目录下的dynamic_synchronous.bag
            output_dir: 可选，输出目录路径，默认为/mnt/d/rosbag/dynamic/synchronous
    """
    # 直接指定 bag 文件与输出目录（无需通过字典与 next 获取）
    bag_file = "/mnt/d/rosbag/big_exam/synchronous_1014.bag"
    base_output_dir = "/mnt/d/rosbag/big_exam/synchronous_1014_gensui"
    # 如需批量处理多个 bag，可改为列表/字典并使用 for 循环逐个处理。
    
    # 点云数据参数
    pcd_topic_name = "/gensui/lidar"  # 要提取的点云话题
    pcd_output_dir = os.path.join(base_output_dir, "pcds")  # PCD文件输出目录
    
    # 里程计数据参数
    pose_topic_name = "/gensui/pose"  # 要提取的里程计话题
    pose_output_dir = base_output_dir  # 位姿数据输出目录
    pose_output_file = os.path.join(pose_output_dir, "poses_lidar2body_origin.csv")  # 位姿数据输出文件
    
    print("=== ROS Bag 转 PCD 和里程计数据提取工具 ===")
    print(f"输入文件: {bag_file}")
    print(f"点云输出目录: {pcd_output_dir}")
    print(f"里程计位姿输出文件: {pose_output_file}")
    print("="*50)
    
    # 检查输入文件是否存在
    if not os.path.exists(bag_file):
        print(f"错误：输入文件不存在: {bag_file}")
        return
    
    # 执行点云和里程计数据处理
    print("\n开始点云数据转换和里程计数据提取...")
    success, pcd_count, pose_count = process_rosbag_data(
        bag_file, 
        pcd_output_dir, 
        pose_output_file, 
        pcd_topic_name, 
        pose_topic_name
    )
    
    if success:
        print(f"\n处理成功完成！")
        print(f"共处理{pcd_count}帧点云数据，匹配了{pose_count}帧里程计数据")
        print(f"PCD文件已保存到: {pcd_output_dir}")
        print(f"与点云对应的里程计位姿数据已保存到: {pose_output_file}")
    else:
        print("\n处理过程中出现错误，请检查日志")

if __name__ == "__main__":
    main()