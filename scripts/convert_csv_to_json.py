#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv

def convert_csv_to_json_format(input_csv_path, output_json_path):
    """
    将poses_lidar2body.csv的数据转换为pose.json格式
    
    Args:
        input_csv_path (str): 输入的CSV文件路径
        output_json_path (str): 输出的JSON格式文件路径
    """
    try:
        # 确保输出目录存在
        os.makedirs(os.path.dirname(output_json_path), exist_ok=True)
        
        # 读取CSV文件
        pose_data = []
        with open(input_csv_path, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # 提取需要的数据并转换为float类型
                pose = {
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                    'qw': float(row['qw']),
                    'qx': float(row['qx']),
                    'qy': float(row['qy']),
                    'qz': float(row['qz'])
                }
                pose_data.append(pose)
        
        # 写入JSON格式文件
        with open(output_json_path, 'w') as f:
            for pose in pose_data:
                # 按照指定顺序写入数据，保留16位小数
                line = f"{pose['x']:.16f} {pose['y']:.16f} {pose['z']:.16f} {pose['qw']:.16f} {pose['qx']:.16f} {pose['qy']:.16f} {pose['qz']:.16f}\n"
                f.write(line)
                
        return True, len(pose_data)
    
    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        return False, 0

def main():
    """
    主函数：将CSV格式的位姿数据转换为JSON格式
    """
    # 配置文件路径
    # input_csv_path = "/mnt/d/rosbag/hba/synchronous/poses_lidar2body.csv"  # 输入CSV文件路径
    input_csv_path = "/mnt/d/rosbag/hba/synchronous/poses_lidar2body_origin.csv"  # 输入CSV文件路径
    output_json_path = "/mnt/d/rosbag/hba/synchronous/pose.json"  # 输出JSON格式文件路径
    
    print("=== 位姿数据格式转换工具 ===")
    print(f"输入文件: {input_csv_path}")
    print(f"输出文件: {output_json_path}")
    print("="*50)
    
    # 检查输入文件是否存在
    if not os.path.exists(input_csv_path):
        print(f"错误：输入文件不存在: {input_csv_path}")
        return
    
    # 执行转换
    success, count = convert_csv_to_json_format(input_csv_path, output_json_path)
    
    if success:
        print("\n转换成功完成！")
        print(f"共处理 {count} 帧位姿数据")
        print(f"数据已保存到: {output_json_path}")
    else:
        print("\n转换失败！")

if __name__ == "__main__":
    main()