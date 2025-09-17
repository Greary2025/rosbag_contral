#!/usr/bin/env python3
import pandas as pd
import json
import numpy as np
import plotly.graph_objects as go
from pathlib import Path
import quaternion  # 需要安装 numpy-quaternion 包

def load_poses_from_csv(file_path):
    """从CSV文件加载位姿数据"""
    df = pd.read_csv(file_path)
    # 移除列名中的空格
    df.columns = df.columns.str.strip()
    return df[['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']].values

def load_poses_from_json(file_path):
    """从类JSON格式文件加载位姿数据，每行包含7个值：x y z qx qy qz qw"""
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            # 分割每一行的数据
            values = line.strip().split()
            if len(values) == 7:  # 确保每行有7个值
                # 将字符串转换为浮点数
                pose = [float(x) for x in values]
                poses.append(pose)
    return np.array(poses)

def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵"""
    quat = np.quaternion(q[3], q[0], q[1], q[2])
    return quaternion.as_rotation_matrix(quat)

def create_arrow_points(position, rotation_matrix, scale=0.5):
    """创建表示方向的箭头点"""
    # 箭头方向（使用旋转矩阵的x轴方向）
    direction = rotation_matrix[:, 0]
    
    # 箭头终点
    end_point = position + direction * scale
    
    return position, end_point

def visualize_poses(poses):
    """创建交互式3D可视化"""
    # 创建图形
    fig = go.Figure()

    # 添加轨迹点
    fig.add_trace(go.Scatter3d(
        x=poses[:, 0],
        y=poses[:, 1],
        z=poses[:, 2],
        mode='markers',
        marker=dict(size=2),
        name='Trajectory'
    ))

    # 添加方向箭头
    for pose in poses:
        position = pose[:3]
        quat = pose[3:]
        rot_matrix = quaternion_to_rotation_matrix(quat)
        start, end = create_arrow_points(position, rot_matrix)
        
        # 添加箭头
        fig.add_trace(go.Scatter3d(
            x=[start[0], end[0]],
            y=[start[1], end[1]],
            z=[start[2], end[2]],
            mode='lines',
            line=dict(color='red', width=2),
            showlegend=False
        ))

    # 设置布局
    fig.update_layout(
        scene=dict(
            aspectmode='data',  # 保持真实比例
            camera=dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=1.5, y=1.5, z=1.5)
            ),
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'
        ),
        title='Pose Visualization',
        showlegend=True
    )

    # 显示图形
    fig.show()

def main():
    # 设置文件路径
    # base_path = Path(__file__).parent.parent
    base_path = Path('/mnt/d/rosbag/hba/0722_xianfeng')
    # csv_path = base_path / 'config' / 'poses_lidar2body.csv'
    # csv_path = base_path / 'poses_lidar2body.csv'
    csv_path = base_path / 'poses_lidar2body_origin.csv'
    # json_path = base_path / 'config' / 'pose.json'
    json_path = base_path / 'pose.json'

    # 尝试加载数据
    poses = None
    if csv_path.exists():
        print(f"从CSV文件加载数据: {csv_path}")
        poses = load_poses_from_csv(csv_path)
    elif json_path.exists():
        print(f"从JSON文件加载数据: {json_path}")
        poses = load_poses_from_json(json_path)
    else:
        print(f"错误: 在config目录下未找到poses_lidar2body.csv或pose.json文件！")
        print(f"请确保以下任一文件存在:")
        print(f"  - {csv_path}")
        print(f"  - {json_path}")
        return

    # 可视化位姿
    visualize_poses(poses)

if __name__ == '__main__':
    main()