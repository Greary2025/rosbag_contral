#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv

def convert_pose_format(input_json_path, input_csv_path, output_csv_path):
    """
    将pose.json的数据转换并替换到poses_lidar2body.csv中
    
    Args:
        input_json_path (str): 输入的json文件路径
        input_csv_path (str): 输入的csv文件路径
        output_csv_path (str): 输出的csv文件路径
    """
    # 读取json文件中的位姿数据
    pose_data = []
    with open(input_json_path, 'r') as f:
        for line in f:
            # 分割每行数据（以空格分隔）
            values = line.strip().split()
            if len(values) == 7:  # 确保数据完整性
                x, y, z, qw, qx, qy, qz = map(float, values)
                # 注意这里调整了顺序，以匹配目标格式
                pose_data.append([x, y, z, qx, qy, qz, qw])

    # 读取原始CSV文件的header和时间戳信息
    csv_data = []
    with open(input_csv_path, 'r') as f:
        csv_reader = csv.reader(f)
        header = next(csv_reader)  # 读取header行
        csv_data.append(header)
        
        # 读取所有行
        rows = list(csv_reader)

    # 确保数据长度匹配
    min_length = min(len(pose_data), len(rows))
    
    # 创建新的CSV数据
    for i in range(min_length):
        new_row = rows[i].copy()  # 复制原始行（保留索引和时间戳）
        # 替换位姿数据（x, y, z, qx, qy, qz, qw）
        for j, value in enumerate(pose_data[i]):
            # 根据poses_lidar2body.csv的格式，从第3列开始替换
            new_row[j + 2] = f"{value:.8f}"  # 保留8位小数
        csv_data.append(new_row)

    # 写入新的CSV文件
    os.makedirs(os.path.dirname(output_csv_path), exist_ok=True)
    with open(output_csv_path, 'w', newline='') as f:
        csv_writer = csv.writer(f)
        csv_writer.writerows(csv_data)

def main():
    # 配置文件路径
    input_json_path = "/mnt/d/rosbag/hba/synchronous/pose.json"  # pose.json的路径
    input_csv_path = "/mnt/d/rosbag/hba/synchronous/poses_lidar2body_origin.csv"  # 原始CSV文件路径
    output_csv_path = "/mnt/d/rosbag/hba/synchronous/poses_lidar2body.csv"  # 输出CSV文件路径
    
    print("=== 位姿数据转换工具 ===")
    print(f"输入JSON文件: {input_json_path}")
    print(f"输入CSV文件: {input_csv_path}")
    print(f"输出CSV文件: {output_csv_path}")
    print("=" * 50)
    
    # 检查输入文件是否存在
    if not os.path.exists(input_json_path):
        print(f"错误：输入JSON文件不存在: {input_json_path}")
        return
    if not os.path.exists(input_csv_path):
        print(f"错误：输入CSV文件不存在: {input_csv_path}")
        return
        
    try:
        convert_pose_format(input_json_path, input_csv_path, output_csv_path)
        print("\n转换成功完成！")
        print(f"数据已保存到: {output_csv_path}")
    except Exception as e:
        print(f"\n转换过程中发生错误: {e}")

if __name__ == "__main__":
    main()