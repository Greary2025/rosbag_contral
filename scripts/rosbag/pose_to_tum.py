#!/usr/bin/env python3
"""
从ROS bag中读取位姿话题（支持 nav_msgs/Odometry 或 geometry_msgs/PoseStamped），
导出为TUM轨迹格式（timestamp tx ty tz qx qy qz qw）。

TUM格式字段说明：
- timestamp：时间戳（秒，双精度浮点）
- tx, ty, tz：位置（米）
- qx, qy, qz, qw：单位四元数（w为实部，顺序为 qx qy qz qw）

使用方式：
- 不使用命令行传参，直接在脚本顶部“配置参数”位置修改 bag_file / topic_name / output_file。
"""

import os
import sys
from typing import Optional

try:
    import rosbag
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
except Exception as e:
    rosbag = None  # type: ignore
    Odometry = None  # type: ignore
    PoseStamped = None  # type: ignore
    _import_err = e
else:
    _import_err = None

# ==================== 配置参数 ====================
# 可在此处直接修改参数，无需命令行传参

# 输入：ROS bag 文件路径
bag_file = '/mnt/d/rosbag/big_exam/synchronous_1014.bag'

# 输入：位姿话题（支持 Odometry 或 PoseStamped）
topic_name = '/gensui/pose'

# 输出：TUM格式文件路径
output_file = '/mnt/d/rosbag/big_exam/synchronous_1014_gensui/gensui_gt_tum.txt'

# ================================================


def _tum_line_from_odometry(msg, timestamp: float) -> str:
    """从 Odometry 消息构造一行 TUM 字符串。"""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return f"{timestamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}"


def _tum_line_from_posestamped(msg, timestamp: float) -> str:
    """从 PoseStamped 消息构造一行 TUM 字符串。"""
    p = msg.pose.position
    q = msg.pose.orientation
    return f"{timestamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}"


def export_tum_from_bag(bag_file: str, topic_name: str, out_path: str) -> int:
    """从指定 bag 的话题中导出 TUM 轨迹，返回写入的行数。"""
    if rosbag is None or (Odometry is None and PoseStamped is None):
        raise RuntimeError(
            f"未检测到ROS Python依赖（rosbag/nav_msgs/geometry_msgs）：{_import_err}。请在已source ROS环境的终端中运行此脚本。"
        )

    if not os.path.exists(bag_file):
        raise FileNotFoundError(f"未找到bag文件：{bag_file}")

    out_dir = os.path.dirname(os.path.abspath(out_path)) or "."
    os.makedirs(out_dir, exist_ok=True)

    count = 0
    with rosbag.Bag(bag_file, "r") as bag, open(out_path, "w") as f:
        # 可选：写一行注释作为表头（大多数评估工具会忽略以#开头的行）
        f.write("# timestamp tx ty tz qx qy qz qw\n")

        topics = set(bag.get_type_and_topic_info()[1].keys())
        if topic_name not in topics:
            raise ValueError(f"指定的话题不存在：{topic_name}\n可用话题：{sorted(topics)}")

        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            ts = t.to_sec()
            msg_type_name = type(msg).__name__

            if (Odometry is not None and isinstance(msg, Odometry)) or msg_type_name == "_nav_msgs__Odometry":
                line = _tum_line_from_odometry(msg, ts)
            elif (PoseStamped is not None and isinstance(msg, PoseStamped)) or msg_type_name == "_geometry_msgs__PoseStamped":
                line = _tum_line_from_posestamped(msg, ts)
            else:
                # 非支持类型，跳过
                continue

            f.write(line + "\n")
            count += 1

    return count


def main():
    try:
        n = export_tum_from_bag(bag_file, topic_name, output_file)
    except Exception as e:
        print(f"错误：{e}")
        sys.exit(1)
    print(f"完成。已写入 {n} 行到：{output_file}")


if __name__ == '__main__':
    main()