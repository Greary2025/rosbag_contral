#!/usr/bin/env python
import rospy
import rosbag
import math
import bisect
import numpy as np
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

def load_config():
    with open('config/params.yaml', 'r') as f:
        params = yaml.safe_load(f)
    return params

def process_point_cloud(msg, sorted_angles, sorted_indices):
    # 预处理字段
    original_fields = msg.fields
    original_field_names = [f.name for f in original_fields]
    
    # 读取所有点
    points = list(point_cloud2.read_points(msg, skip_nans=True))
    if not points:
        return None
    
    # 提取x, y, z并计算垂直角度
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    z = [p[2] for p in points]
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    
    distance_xy = np.sqrt(x**2 + y**2)
    theta_rad = np.arctan2(z, distance_xy)
    theta_deg = np.degrees(theta_rad)
    
    # 处理distance_xy为0的情况
    zero_mask = (distance_xy == 0)
    theta_deg[zero_mask] = np.where(z[zero_mask] > 0, 90.0, -90.0)
    
    # 查找最近的ring索引
    rings = []
    for td in theta_deg:
        pos = bisect.bisect_left(sorted_angles, td)
        if pos == 0:
            ring = sorted_indices[0]
        elif pos == len(sorted_angles):
            ring = sorted_indices[-1]
        else:
            before = sorted_angles[pos-1]
            after = sorted_angles[pos]
            if (after - td) < (td - before):
                ring = sorted_indices[pos]
            else:
                ring = sorted_indices[pos-1]
        rings.append(ring)
    
    # 构造新的点数据
    new_points = []
    for i, p in enumerate(points):
        new_point = list(p) + [rings[i]]
        new_points.append(new_point)
    
    # 构造新的字段
    new_fields = list(original_fields)
    new_fields.append(PointField(name='ring', offset=msg.point_step, datatype=PointField.UINT32, count=1))
    
    # 创建新的PointCloud2消息
    new_msg = point_cloud2.create_cloud(msg.header, new_fields, new_points)
    new_msg.is_dense = msg.is_dense
    return new_msg

def main():
    params = load_config()
    input_bag = params['rosbag']['input_path']
    output_bag = params['rosbag']['output_path']
    vertical_angles = params['lidar_params']['vertical_angles']
    
    # 预处理垂直角度
    angle_index_pairs = sorted((angle, idx) for idx, angle in enumerate(vertical_angles))
    sorted_angles = [p[0] for p in angle_index_pairs]
    sorted_indices = [p[1] for p in angle_index_pairs]
    
    with rosbag.Bag(input_bag, 'r') as in_bag, rosbag.Bag(output_bag, 'w') as out_bag:
        for topic, msg, t in in_bag.read_messages():
            if topic == '/xianfeng_lidar':
                new_msg = process_point_cloud(msg, sorted_angles, sorted_indices)
                if new_msg:
                    out_bag.write(topic, new_msg, t)
            else:
                out_bag.write(topic, msg, t)

if __name__ == '__main__':
    main()