#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从指定的ROS bag文件中提取IMU（sensor_msgs/Imu）话题数据，保存为TXT文本文件。

输出文本为空格分隔，首行表头为：
    timestamp ax ay az gx gy gz

字段说明：
- timestamp：时间戳（秒，ROS时间）
- ax, ay, az：线加速度（单位：m/s^2）
- gx, gy, gz：角速度（单位：rad/s）

使用方式：
- 本脚本不再通过命令行传参，请在 main() 中直接修改变量：
  bag_file（bag文件路径）、topic_name（IMU话题）、out_path（输出文件路径）。
- 修改保存后，直接运行：python scripts/imu_to_txt.py

说明：
- 需要在安装并配置好ROS的Python环境中运行（依赖rosbag与sensor_msgs）。
- 输出文件为纯文本，采用空格分隔，并包含表头，便于NumPy/MATLAB直接读取。
"""

import os
import sys
from typing import Optional

try:
    import rosbag
    from sensor_msgs.msg import Imu
except Exception as e:
    rosbag = None  # type: ignore
    Imu = None  # type: ignore
    _import_err = e
else:
    _import_err = None


def export_imu(bag_file: str, topic_name: str, out_path: str) -> int:
    """将IMU消息导出到文本文件。

    返回写入的消息条数。遇到致命错误会抛出异常。
    """
    if rosbag is None or Imu is None:
        # 当未在ROS Python环境中运行或未安装相关依赖时，给出友好提示
        raise RuntimeError(
            f"未检测到ROS Python依赖（rosbag/sensor_msgs）：{_import_err}。请在已source ROS环境的终端中运行此脚本。"
        )

    if not os.path.exists(bag_file):
        # 检查bag文件是否存在
        raise FileNotFoundError(f"未找到bag文件：{bag_file}")

    out_dir = os.path.dirname(os.path.abspath(out_path)) or "."
    os.makedirs(out_dir, exist_ok=True)

    count = 0
    with rosbag.Bag(bag_file, "r") as bag, open(out_path, "w") as f:
        # 写入表头（与字段说明一致）
        f.write("timestamp ax ay az gx gy gz\n")

        # quick topic existence check
        topics = set(bag.get_type_and_topic_info()[1].keys())
        if topic_name not in topics:
            # 如果指定话题不存在，列出可用话题帮助排查
            raise ValueError(
                f"指定的IMU话题不存在：{topic_name}\n可用话题：{sorted(topics)}"
            )

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # 兼容判断：既支持标准Imu类型，也支持底层类型名匹配
            if not isinstance(msg, Imu) and type(msg).__name__ != "_sensor_msgs__Imu":
                # 如果连接中混入了非Imu类型消息，则直接跳过
                continue

            ts = t.to_sec()
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z

            # 空格分隔；为确保精度，时间戳与数据保留9位小数
            f.write(f"{ts:.9f} {ax:.9f} {ay:.9f} {az:.9f} {gx:.9f} {gy:.9f} {gz:.9f}\n")
            count += 1

    return count


def parse_args(argv: Optional[list] = None):
    # 保留空壳以兼容可能的外部调用，但不再使用命令行参数
    return None


def main(argv: Optional[list] = None) -> None:
    # 直接在这里指定参数（按需修改为你的实际路径与话题）
    bag_file = "/mnt/d/rosbag/big_exam/synchronous_1014.bag"  # ROS bag文件路径
    topic_name = "/gensui/imu"                              # IMU话题名称
    out_path = "/mnt/d/rosbag/big_exam/synchronous_1014_gensui/gensui_imu.txt"   # 输出TXT文件路径

    try:
        n = export_imu(bag_file, topic_name, out_path)
    except Exception as e:
        # 统一中文错误输出
        print(f"错误：{e}")
        sys.exit(1)
    print(f"完成。已写入 {n} 条IMU数据到：{out_path}")


if __name__ == "__main__":
    main()
