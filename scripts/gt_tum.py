#!/usr/bin/env python3
import rosbag
from nav_msgs.msg import Odometry
import os

# 自定义路径字符串（可修改为你的实际路径）
# input_bag_path = "/mnt/c/Users/Greary/Documents/rosbag/lidar_1_0630.bag"  # 输入bag路径
input_bag_path = "/mnt/c/Users/Greary/Documents/rosbag/double2/double_0722.bag"  # 输入bag路径
# output_txt_name = "true_06_30.txt"                                        # 输出txt文件名
# output_txt_name = "xianfeng_07_22.txt"                                        # 输出txt文件名（xianfeng位姿）
output_txt_name = "gensui_07_22.txt"                                        # 输出txt文件名（gensui位姿）
output_dir = "/mnt/c/Users/Greary/Documents/rosbag/Examdata/double_evo/true"                            # 输出目录
# output_dir = "/mnt/c/Users/Greary/Documents/rosbag/Examdata/lidar_1_evo/true"                            # 输出目录
# output_dir = "/mnt/c/Users/Greary/Documents/rosbag/Examdata/lidar_1_evo/true"                            # 输出目录
# 拼接完整输出路径（使用os.path.join保证跨平台兼容性）
output_txt_path = os.path.join(output_dir, output_txt_name)
# 确保输出目录存在（若不存在则创建）
os.makedirs(output_dir, exist_ok=True)  # exist_ok=True避免目录已存在时报错

def extract_pose_from_rosbag(bag_path, output_path):
    """
    从 ROS Bag 文件中提取 /gensui/pose 话题数据并保存到 TXT 文件
    （原来提取 /xianfeng/pose 话题数据的代码已注释保留）
    
    参数:
        bag_path (str): ROS Bag 文件路径
        output_path (str): 输出 TXT 文件路径
    """
    # 打开 ROS Bag 文件
    with rosbag.Bag(bag_path, 'r') as bag:
        # 打开输出文件（追加模式，若文件存在则覆盖）
        with open(output_path, 'w') as f:
            # 写入文件头（可选，方便后续查看）
            # f.write("time\tposition.x\tposition.y\tposition.z\torientation.w\torientation.x\torientation.y\torientation.z\n")
            print("开始提取数据\r\n")
            # 遍历 Bag 中所有消息，筛选目标话题
            first = True
            first_time = 0
            # for topic, msg, t in bag.read_messages(topics=['/xianfeng/pose']):  # xianfeng位姿数据
            for topic, msg, t in bag.read_messages(topics=['/gensui/pose']):    # gensui位姿数据
                # 调试：打印当前处理的话题和消息类型
                # print(f"正在处理话题 [{topic}]，消息类型：{type(msg)}")  # 关键调试语句
                # 确保消息类型正确（理论上不需要，但防御性编程）
                # if not isinstance(msg, Odometry):
                    # print(f"跳过非 Odometry 类型的消息（实际类型：{type(msg)}）")
                    # continue
                
                # 提取时间戳（转换为秒级浮点数）
                timestamp = msg.header.stamp.to_sec()
                if first:
                    first = False
                    first_time = timestamp
                timestamp = timestamp - first_time
                
                # 提取位置信息（来自 pose.pose.position）
                position = msg.pose.pose.position
                pos_x = position.x
                # print(pos_x)
                pos_y = position.y
                pos_z = position.z
                
                # 提取姿态信息（来自 pose.pose.orientation）
                orientation = msg.pose.pose.orientation
                ori_x = orientation.x
                ori_y = orientation.y
                ori_z = orientation.z
                ori_w = orientation.w
                
                # 按格式写入 TXT 文件（使用制表符分隔，兼容性更好）
                line = f"{timestamp} {pos_x} {pos_y} {pos_z} {ori_x} {ori_y} {ori_z} {ori_w}\n"
                f.write(line)

if __name__ == "__main__":
    # 配置参数（根据实际路径修改）
    # input_bag_path = "lidar_1_intensity_06_20_4x.bag"  # ROS Bag 文件路径
    # output_txt_path = "xianfeng_pose_data.txt"         # 输出 TXT 文件路径
    
    # 执行数据提取
    extract_pose_from_rosbag(input_bag_path, output_txt_path)
    print(f"数据提取完成！结果保存至：{output_txt_path}")