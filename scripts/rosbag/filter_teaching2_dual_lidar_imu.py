#!/usr/bin/env python3

"""
Filter teaching2_dual.bag to keep only LiDAR + IMU (+ tf_static).

Output: teaching2_dual_lidar_imu.bag
"""

import os
import sys

try:
    import rosbag
except Exception as e:
    rosbag = None  # type: ignore
    _import_err = e
else:
    _import_err = None


INPUT_BAG = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2_dual.bag"
OUTPUT_BAG = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2_dual_lidar_imu.bag"
CLEAR_OUTPUT_BAG = True

KEEP_TOPICS = {
    "/xianfeng/lidar",
    "/xianfeng/imu",
    "/xianfeng/mag",
    "/xianfeng/scan",
    "/xianfeng/velodyne_packets",
    "/gensui/lidar",
    "/gensui/imu",
    "/gensui/mag",
    "/gensui/scan",
    "/gensui/velodyne_packets",
    "/tf_static",
}

IMU_FRAME_MAP = {
    "/xianfeng/imu": "xianfeng/imu_link",
    "/gensui/imu": "gensui/imu_link",
}


def main() -> None:
    if rosbag is None:
        print(f"错误：未检测到ROS Python依赖（rosbag）：{_import_err}")
        sys.exit(1)

    if not os.path.exists(INPUT_BAG):
        print(f"错误：输入bag不存在: {INPUT_BAG}")
        sys.exit(1)

    if CLEAR_OUTPUT_BAG and os.path.exists(OUTPUT_BAG):
        os.remove(OUTPUT_BAG)

    written = 0
    with rosbag.Bag(INPUT_BAG, "r") as in_bag, rosbag.Bag(OUTPUT_BAG, "w") as out_bag:
        for i, (topic, msg, t) in enumerate(in_bag.read_messages(), start=1):
            if topic not in KEEP_TOPICS:
                continue
            if topic in IMU_FRAME_MAP and hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
                msg.header.frame_id = IMU_FRAME_MAP[topic]
            out_bag.write(topic, msg, t)
            written += 1
            if i % 50000 == 0:
                print(f"已处理输入消息 {i} 条，已写出 {written} 条")

    print("完成。")
    print(f"写出消息: {written}")
    print(f"输出: {OUTPUT_BAG}")


if __name__ == "__main__":
    main()
