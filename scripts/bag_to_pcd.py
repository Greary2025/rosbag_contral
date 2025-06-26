#!/usr/bin/env python

import rospy
import rosbag
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
import sys
from datetime import datetime
import argparse

def convert_rosbag_to_pcd(bag_file, output_dir=None):
    """
    将ROS bag文件中的点云数据转换为PCD文件
    
    Args:
        bag_file (str): ROS bag文件的路径
        output_dir (str, optional): 输出PCD文件的目录。如果为None，则使用bag文件所在的目录。
    
    Returns:
        bool: 转换是否成功
    """
    # 如果未指定输出目录，则使用bag文件所在的目录
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(bag_file))
    
    # 创建输出目录（如果不存在）
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 获取不带扩展名的bag文件名
    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    
    try:
        # 打开ROS bag文件
        bag = rosbag.Bag(bag_file, 'r')
        
        # 获取bag中的所有话题
        topics = bag.get_type_and_topic_info()[1].keys()
        
        # 查找点云话题
        pointcloud_topics = [topic for topic in topics if 'pointcloud' in topic.lower()]
        
        if not pointcloud_topics:
            rospy.logerr("No point cloud topics found in the bag file.")
            return False
        
        # 创建输出文件名基础
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"{bag_name}_{timestamp}"
        
        # 处理每个点云话题
        for topic in pointcloud_topics:
            # 创建该话题的输出目录
            topic_output_dir = os.path.join(output_dir, os.path.basename(topic).replace('/', '_'))
            if not os.path.exists(topic_output_dir):
                os.makedirs(topic_output_dir)
            
            # 创建输出PCD文件名
            pcd_filename = f"{base_filename}_{topic.replace('/', '_')}.pcd"
            pcd_path = os.path.join(topic_output_dir, pcd_filename)
            
            # 写入PCD文件的头部
            with open(pcd_path, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z rgb\n")
                f.write("SIZE 4 4 4 4\n")
                f.write("TYPE F F F F\n")
                f.write("COUNT 1 1 1 1\n")
                
                # 计算点云中的点数
                msg = None
                for topic_msg, msg, _ in bag.read_messages(topics=[topic]):
                    pass
                
                if msg is None:
                    rospy.logerr(f"No messages found on topic {topic}")
                    return False
                
                # 获取点云中的点数
                num_points = msg.height * msg.width if msg.height > 0 and msg.width > 0 else msg.width
                
                if num_points == 0:
                    num_points = msg.height * msg.width
                
                f.write(f"WIDTH {num_points}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {num_points}\n")
                f.write("DATA ascii\n")
            
            # 写入点云数据
            point_count = 0
            with open(pcd_path, 'a') as f:
                for topic_msg, msg, _ in bag.read_messages(topics=[topic]):
                    # 将ROS PointCloud2消息转换为NumPy数组
                    pc_data = pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
                    
                    # 将NumPy数组转换为列表
                    points_list = list(pc_data)
                    
                    # 写入每个点的数据
                    for point in points_list:
                        # 确保point有4个元素(xyz和rgb)
                        if len(point) < 4:
                            # 如果rgb值缺失，添加默认值0
                            point = list(point) + [0.0]
                        
                        f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} {point[3]:.6f}\n")
                    
                    point_count += len(points_list)
            
            rospy.loginfo(f"Saved {point_count} points from topic {topic} to {pcd_path}")
        
        bag.close()
        return True
    
    except Exception as e:
        rospy.logerr(f"Error converting ROS bag to PCD: {e}")
        return False

def main():
    # 初始化ROS节点
    rospy.init_node('rosbag_to_pcd_converter', anonymous=True)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Convert ROS bag files to PCD files.')
    parser.add_argument('bag_file', help='Path to the ROS bag file')
    parser.add_argument('--output_dir', help='Output directory for PCD files', default=None)
    args = parser.parse_args(rospy.myargv()[1:])
    
    # 执行转换
    success = convert_rosbag_to_pcd(args.bag_file, args.output_dir)
    
    if success:
        rospy.loginfo("Conversion completed successfully.")
    else:
        rospy.logerr("Conversion failed.")
    
    # 保持节点运行，直到手动终止
    rospy.spin()

if __name__ == "__main__":
    main()