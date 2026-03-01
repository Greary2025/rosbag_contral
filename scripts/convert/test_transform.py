#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
位姿变换测试脚本
用于验证transform_poses.py的变换结果是否正确
"""

import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R


def test_position_transform():
    """测试位置变换"""
    print("=" * 50)
    print("测试位置变换")
    print("=" * 50)
    
    # 测试数据
    original_positions = [
        (1.0, 2.0, 3.0),
        (0.145576, -0.323060, 0.380333),
        (-1.5, 2.5, -3.0)
    ]
    
    for i, (x, y, z) in enumerate(original_positions):
        # 应用变换：(x, y, z) → (-y, x, -z)
        x_new = -y
        y_new = x
        z_new = -z
        
        print(f"测试 {i+1}:")
        print(f"  原始: ({x:8.6f}, {y:8.6f}, {z:8.6f})")
        print(f"  变换: ({x_new:8.6f}, {y_new:8.6f}, {z_new:8.6f})")
        print()


def test_quaternion_transform():
    """测试四元数变换"""
    print("=" * 50)
    print("测试四元数变换（绕Z轴顺时针旋转90度）")
    print("=" * 50)
    
    # 测试数据
    test_quaternions = [
        (0, 0, 0, 1),  # 单位四元数
        (1, 0, 0, 0),  # 绕X轴180度
        (0, 1, 0, 0),  # 绕Y轴180度
        (-0.001788, 0.006879, 0.518897, -0.854807)  # 实际数据示例
    ]
    
    for i, (qx, qy, qz, qw) in enumerate(test_quaternions):
        # 原始四元数
        q_original = R.from_quat([qx, qy, qz, qw])
        
        # 绕Z轴顺时针旋转90度
        angle = -np.pi / 2
        axis = np.array([0, 0, 1])
        q_rotation = R.from_rotvec(angle * axis)
        
        # 复合旋转
        q_result = q_rotation * q_original
        qx_new, qy_new, qz_new, qw_new = q_result.as_quat()
        
        print(f"测试 {i+1}:")
        print(f"  原始: qx={qx:8.6f}, qy={qy:8.6f}, qz={qz:8.6f}, qw={qw:8.6f}")
        print(f"  变换: qx={qx_new:8.6f}, qy={qy_new:8.6f}, qz={qz_new:8.6f}, qw={qw_new:8.6f}")
        
        # 验证四元数长度
        orig_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        new_norm = np.sqrt(qx_new**2 + qy_new**2 + qz_new**2 + qw_new**2)
        print(f"  原始长度: {orig_norm:.6f}, 变换后长度: {new_norm:.6f}")
        print()


def verify_csv_transformation(csv_file):
    """验证CSV文件的变换结果"""
    print("=" * 50)
    print(f"验证CSV文件变换结果: {csv_file}")
    print("=" * 50)
    
    try:
        df = pd.read_csv(csv_file)
        print(f"文件包含 {len(df)} 条记录")
        
        # 检查前几条记录的四元数长度
        print("\n检查四元数长度（应该接近1.0）:")
        for i in range(min(5, len(df))):
            qx, qy, qz, qw = df.loc[i, ['qx', 'qy', 'qz', 'qw']]
            norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            print(f"  第{i+1}条: 长度 = {norm:.6f}")
        
        # 统计四元数长度
        norms = np.sqrt(df['qx']**2 + df['qy']**2 + df['qz']**2 + df['qw']**2)
        print(f"\n四元数长度统计:")
        print(f"  最小值: {norms.min():.6f}")
        print(f"  最大值: {norms.max():.6f}")
        print(f"  平均值: {norms.mean():.6f}")
        print(f"  标准差: {norms.std():.6f}")
        
    except Exception as e:
        print(f"错误: {e}")


def main():
    """主函数"""
    print("位姿变换验证测试")
    print("=" * 70)
    
    # 测试位置变换
    test_position_transform()
    
    # 测试四元数变换
    test_quaternion_transform()
    
    # 验证实际CSV文件
    csv_file = "/home/john/rosbag_evo/src/rosbag_contral/config/poses_lidar2body.csv"
    verify_csv_transformation(csv_file)


if __name__ == "__main__":
    main()