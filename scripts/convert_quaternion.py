#!/usr/bin/env python3
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation

def rotate_position_90_degrees_ccw(x, y, z):
    """将位置绕Z轴逆时针旋转90度"""
    new_x = -y
    new_y = x
    return new_x, new_y, z

def rotate_quaternion_90_degrees_ccw(qx, qy, qz, qw):
    """将姿态四元数绕Z轴逆时针旋转90度"""
    # 创建原始旋转
    r_orig = Rotation.from_quat([qx, qy, qz, qw])
    
    # 创建绕Z轴旋转90度的旋转
    r_z90 = Rotation.from_euler('z', np.pi/2)
    
    # 组合旋转（先应用原始旋转，再旋转90度）
    r_combined = r_z90 * r_orig
    
    # 返回新的四元数
    return r_combined.as_quat()

def main():
    # 读取CSV文件
    csv_path = '/mnt/d/rosbag/hba/0722_xianfeng/poses_lidar2body_origin.csv'
    df = pd.read_csv(csv_path)
    
    # 删除列名中的空格
    df.columns = df.columns.str.strip()
    
    print("正在处理数据...")
    # 处理每一行数据
    for index in range(len(df)):
        # 获取位置
        x = df.loc[index, 'x']
        y = df.loc[index, 'y']
        z = df.loc[index, 'z']
        
        # 获取四元数
        qx = df.loc[index, 'qx']
        qy = df.loc[index, 'qy']
        qz = df.loc[index, 'qz']
        qw = df.loc[index, 'qw']
        
        # 旋转位置
        new_x, new_y, new_z = rotate_position_90_degrees_ccw(x, y, z)
        
        # 旋转姿态
        new_quat = rotate_quaternion_90_degrees_ccw(qx, qy, qz, qw)
        
        # 更新DataFrame中的值
        df.loc[index, 'x'] = new_x
        df.loc[index, 'y'] = new_y
        df.loc[index, 'z'] = new_z
        df.loc[index, 'qx'] = new_quat[0]
        df.loc[index, 'qy'] = new_quat[1]
        df.loc[index, 'qz'] = new_quat[2]
        df.loc[index, 'qw'] = new_quat[3]
    
    print("保存修改后的数据...")
    # 保存回CSV文件，保持相同的格式
    df.to_csv(csv_path, index=False)
    print("完成！")

if __name__ == '__main__':
    main()