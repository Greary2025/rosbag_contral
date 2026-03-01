#!/usr/bin/env python3

"""
用途：从 asynchronous_0115_D3.bag 中提取 xianfeng 和 gensui 的真值轨迹。
输出：分别写入两个目录，格式为 TUM（timestamp tx ty tz qx qy qz qw）。
时间戳：保留绝对时间戳（优先 header.stamp，不做首帧归零）。
最后修改：2026-02-19
"""

import os
import shutil
import sys
from typing import Any, Dict, List, Tuple

try:
    import rosbag
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
except Exception as e:
    rosbag = None  # type: ignore
    PoseStamped = None  # type: ignore
    Odometry = None  # type: ignore
    _import_err = e
else:
    _import_err = None


# ==================== 可直接修改的固定参数 ====================
INPUT_BAG = "/mnt/c/Users/Greary/Documents/rosbag/D3/asynchronous_0115_D3.bag"

XIANFENG_OUTPUT_DIR = "/mnt/c/Users/Greary/Documents/rosbag/D3/asynchronous_0115_D3_xianfeng"
GENSUI_OUTPUT_DIR = "/mnt/c/Users/Greary/Documents/rosbag/D3/asynchronous_0115_D3_gensui"

XIANFENG_TOPIC_CANDIDATES = [
    "/xianfeng/pose",
    "/xianfeng/odom",
    "/xianfeng/ground_truth",
    "/xianfeng/gt_pose",
]

GENSUI_TOPIC_CANDIDATES = [
    "/gensui/pose",
    "/gensui/odom",
    "/gensui/ground_truth",
    "/gensui/gt_pose",
]

# True: 每次运行前先删除目标输出目录
CLEAR_OUTPUT_DIR = True
# ==========================================================


def _is_supported_pose_type(msg_type: str) -> bool:
    return msg_type in ("nav_msgs/Odometry", "geometry_msgs/PoseStamped")


def _get_timestamp(msg, bag_t) -> float:
    # 保留绝对时间，不做相对时间归零。
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        stamp = msg.header.stamp.to_sec()
        if stamp > 0.0:
            return stamp
    return bag_t.to_sec()


def _tum_line_from_msg(msg, timestamp: float) -> str:
    if (Odometry is not None and isinstance(msg, Odometry)) or type(msg).__name__ == "_nav_msgs__Odometry":
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
    elif (PoseStamped is not None and isinstance(msg, PoseStamped)) or type(msg).__name__ == "_geometry_msgs__PoseStamped":
        p = msg.pose.position
        q = msg.pose.orientation
    else:
        raise TypeError(f"不支持的位姿消息类型: {type(msg)}")

    return f"{timestamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}"


def _write_topic_summary(topic_info: Dict[str, Any], out_path: str) -> None:
    with open(out_path, "w") as f:
        f.write("topic,type,messages\n")
        for topic_name in sorted(topic_info.keys()):
            t = topic_info[topic_name]
            f.write(f"{topic_name},{t.msg_type},{t.message_count}\n")


def _resolve_gt_topic(
    topic_info: Dict[str, Any],
    vehicle_name: str,
    candidates: List[str],
) -> str:
    available_topics = list(topic_info.keys())

    for topic in candidates:
        if topic in topic_info and _is_supported_pose_type(topic_info[topic].msg_type):
            return topic

    vehicle_topics = [t for t in available_topics if vehicle_name in t]
    gt_keywords = ("ground_truth", "groundtruth", "gt", "true")
    pose_keywords = ("pose", "odom")

    scored: List[Tuple[int, str]] = []
    for topic in vehicle_topics:
        msg_type = topic_info[topic].msg_type
        if not _is_supported_pose_type(msg_type):
            continue

        score = 0
        if any(k in topic for k in gt_keywords):
            score += 100
        if any(k in topic for k in pose_keywords):
            score += 20
        scored.append((score, topic))

    if not scored:
        raise ValueError(
            f"未找到 {vehicle_name} 的真值位姿话题。\n"
            f"{vehicle_name} 相关话题: {sorted(vehicle_topics)}"
        )

    scored.sort(key=lambda x: (-x[0], x[1]))
    return scored[0][1]


def _export_vehicle_gt(bag: Any, topic_name: str, output_dir: str) -> int:
    os.makedirs(output_dir, exist_ok=True)
    tum_path = os.path.join(output_dir, "gt_tum.txt")
    topic_path = os.path.join(output_dir, "used_topic.txt")

    count = 0
    with open(tum_path, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for _, msg, t in bag.read_messages(topics=[topic_name]):
            try:
                line = _tum_line_from_msg(msg, _get_timestamp(msg, t))
            except TypeError:
                continue
            f.write(line + "\n")
            count += 1

    with open(topic_path, "w") as f:
        f.write(topic_name + "\n")

    return count


def main() -> None:
    if rosbag is None:
        print(
            f"错误：未检测到ROS Python依赖（rosbag/nav_msgs/geometry_msgs）：{_import_err}。请在source过ROS环境的终端运行。"
        )
        sys.exit(1)

    if not os.path.exists(INPUT_BAG):
        print(f"错误：bag文件不存在: {INPUT_BAG}")
        sys.exit(1)

    if CLEAR_OUTPUT_DIR and os.path.isdir(XIANFENG_OUTPUT_DIR):
        shutil.rmtree(XIANFENG_OUTPUT_DIR)
    if CLEAR_OUTPUT_DIR and os.path.isdir(GENSUI_OUTPUT_DIR):
        shutil.rmtree(GENSUI_OUTPUT_DIR)

    print("=== asynchronous_0115_D3 真值提取开始 ===")
    print(f"输入bag: {INPUT_BAG}")

    with rosbag.Bag(INPUT_BAG, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        topic_csv = os.path.join(os.path.dirname(INPUT_BAG), "asynchronous_0115_D3_topic_summary.csv")
        _write_topic_summary(topic_info, topic_csv)

        xianfeng_topic = _resolve_gt_topic(topic_info, "xianfeng", XIANFENG_TOPIC_CANDIDATES)
        gensui_topic = _resolve_gt_topic(topic_info, "gensui", GENSUI_TOPIC_CANDIDATES)

        xianfeng_count = _export_vehicle_gt(bag, xianfeng_topic, XIANFENG_OUTPUT_DIR)
        gensui_count = _export_vehicle_gt(bag, gensui_topic, GENSUI_OUTPUT_DIR)

    print("处理完成。")
    print(f"- xianfeng 真值话题: {xianfeng_topic}")
    print(f"- xianfeng 输出目录: {XIANFENG_OUTPUT_DIR}")
    print(f"- xianfeng 写入条数: {xianfeng_count}")
    print(f"- gensui 真值话题: {gensui_topic}")
    print(f"- gensui 输出目录: {GENSUI_OUTPUT_DIR}")
    print(f"- gensui 写入条数: {gensui_count}")
    print(f"- 话题统计: {topic_csv}")


if __name__ == "__main__":
    main()
