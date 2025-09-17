#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿数据变换脚本
功能：读取CSV格式的位姿文件，进行如下变换：
1. X轴数据和Y轴数据交换
2. Z轴数据取反
3. 交换后的X轴数据取反
注意：不进行四元数旋转操作，保持原始方向不变
"""

import pandas as pd
import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation as R


def transform_quaternion_z_clockwise_90(qx, qy, qz, qw):
    """
    将四元数绕Z轴顺时针旋转90度
    
    Args:
        qx, qy, qz, qw: 输入四元数的分量
    
    Returns:
        tuple: 变换后的四元数 (qx_new, qy_new, qz_new, qw_new)
    """
    # 输入四元数
    q_input = np.array([qw, qx, qy, qz])  # scipy使用 [w, x, y, z] 格式
    
    # 绕Z轴顺时针旋转90度的四元数
    # 顺时针旋转90度 = -90度 = -π/2 弧度
    angle = -np.pi / 2
    axis = np.array([0, 0, 1])
    q_rotation = R.from_rotvec(angle * axis).as_quat()  # 返回 [x, y, z, w]
    q_rotation = np.array([q_rotation[3], q_rotation[0], q_rotation[1], q_rotation[2]])  # 转换为 [w, x, y, z]
    
    # 四元数乘法：q_result = q_rotation * q_input
    r_rot = R.from_quat([q_rotation[1], q_rotation[2], q_rotation[3], q_rotation[0]])  # [x, y, z, w]
    r_input = R.from_quat([q_input[1], q_input[2], q_input[3], q_input[0]])  # [x, y, z, w]
    
    # 计算复合旋转
    r_result = r_rot * r_input
    q_result = r_result.as_quat()  # [x, y, z, w]
    
    return q_result[0], q_result[1], q_result[2], q_result[3]  # 返回 (qx, qy, qz, qw)


def transform_poses(input_file, output_file=None):
    """
    对位姿数据进行变换
    
    Args:
        input_file (str): 输入CSV文件路径
        output_file (str): 输出CSV文件路径，如果为None则覆盖原文件
    """
    # 读取CSV文件
    print(f"正在读取文件: {input_file}")
    df = pd.read_csv(input_file)
    
    # 检查必要的列是否存在
    required_cols = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    for col in required_cols:
        if col not in df.columns:
            raise ValueError(f"CSV文件中缺少必要的列: {col}")
    
    print(f"共读取到 {len(df)} 条位姿数据")
    
    # 创建数据副本进行变换
    df_transformed = df.copy()
    
    # 步骤1和3：X轴和Y轴数据交换，然后交换后的X轴数据取反
    # 原始: x, y, z -> 交换: y, x, z -> X取反: -y, x, z
    print("正在进行位置变换...")
    df_transformed['x'] = -df['y']  # 新X = -原Y
    df_transformed['y'] = df['x']   # 新Y = 原X
    
    # 步骤2：Z轴数据取反
    df_transformed['z'] = -df['z']  # 新Z = -原Z
    
    # 注意：不进行四元数旋转变换，保持原始方向不变
    print("跳过四元数变换，保持原始方向...")
    
    # 确定输出文件路径
    if output_file is None:
        output_file = input_file
    
    # 保存变换后的数据
    print(f"正在保存变换后的数据到: {output_file}")
    df_transformed.to_csv(output_file, index=False)
    print("变换完成！")
    
    # 打印一些统计信息
    print("\n变换前后对比 (前5行):")
    print("原始数据 (x, y, z):")
    print(df[['x', 'y', 'z']].head())
    print("\n变换后数据 (x, y, z):")
    print(df_transformed[['x', 'y', 'z']].head())
    
    print("\n原始数据 (qx, qy, qz, qw) - 保持不变:")
    print(df[['qx', 'qy', 'qz', 'qw']].head())
    print("\n变换后数据 (qx, qy, qz, qw) - 与原始相同:")
    print(df_transformed[['qx', 'qy', 'qz', 'qw']].head())


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='变换位姿数据文件')
    parser.add_argument('input_file', help='输入CSV文件路径')
    parser.add_argument('-o', '--output', help='输出CSV文件路径（可选，默认覆盖原文件）')
    
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.input_file):
        print(f"错误：输入文件 {args.input_file} 不存在！")
        return
    
    try:
        transform_poses(args.input_file, args.output)
    except Exception as e:
        print(f"错误：{e}")


if __name__ == "__main__":
    # 如果直接运行脚本，可以处理默认文件
    # default_file = "/home/john/rosbag_evo/src/rosbag_contral/config/poses_lidar2body.csv"
    default_file = "/mnt/d/rosbag/hba/0722_xianfeng/poses_lidar2body_origin.csv"
    
    if len(os.sys.argv) == 1:  # 没有命令行参数
        if os.path.exists(default_file):
            print(f"使用默认文件: {default_file}")
            transform_poses(default_file)
        else:
            print("请提供CSV文件路径作为参数，或者将文件放在:")
            print(default_file)
    else:
        main()