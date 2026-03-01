#!/usr/bin/env python3

"""
将 teaching2.bag 按时间等长切成两段，并重组为双车同步风格数据。

- 前半段 -> xianfeng
- 后半段 -> gensui（时间戳按相对时间对齐到前半段起点）

输出规则：
- 传感器话题（camera/imu/lidar/tf）按双车区分
- 其中 tf 保持在 /tf 与 /tf_static，上下游 frame id 加车名前缀
- /clock、/rosout 等系统话题仅保留前半段原样

最后修改：2026-02-19
"""

import copy
import os
import sys
import subprocess
from typing import Any, Tuple

try:
    import rosbag
    import rospy
except Exception as e:
    rosbag = None  # type: ignore
    rospy = None  # type: ignore
    _import_err = e
else:
    _import_err = None


INPUT_BAG = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2.bag"
OUTPUT_BAG = "/mnt/c/Users/Greary/Documents/rosbag/real2_bag/teaching2_dual.bag"
CLEAR_OUTPUT_BAG = True


def _is_sensor_topic(topic: str) -> bool:
    if topic.startswith("/camera/"):
        return True
    return topic in {
        "/handsfree/imu",
        "/handsfree/mag",
        "/scan",
        "/velodyne_points",
        "/velodyne_packets",
        "/tf",
        "/tf_static",
    }


def _prefix_frame(frame_id: str, vehicle: str) -> str:
    if not frame_id:
        return frame_id
    f = frame_id[1:] if frame_id.startswith("/") else frame_id
    if f.startswith(vehicle + "/"):
        return f
    return f"{vehicle}/{f}"


def _remap_topic(topic: str, vehicle: str) -> str:
    if topic == "/handsfree/imu":
        return f"/{vehicle}/imu"
    if topic == "/handsfree/mag":
        return f"/{vehicle}/mag"
    if topic == "/scan":
        return f"/{vehicle}/scan"
    if topic == "/velodyne_points":
        return f"/{vehicle}/lidar"
    if topic == "/velodyne_packets":
        return f"/{vehicle}/velodyne_packets"
    if topic.startswith("/camera/"):
        return f"/{vehicle}{topic}"
    if topic in {"/tf", "/tf_static"}:
        return topic
    return topic


def _rewrite_message(msg: Any, vehicle: str, new_ts: float, is_tf_topic: bool) -> Any:
    out = copy.deepcopy(msg)

    if hasattr(out, "header"):
        if hasattr(out.header, "stamp"):
            out.header.stamp = rospy.Time.from_sec(new_ts)
        if hasattr(out.header, "frame_id"):
            out.header.frame_id = _prefix_frame(out.header.frame_id, vehicle)

    if is_tf_topic and hasattr(out, "transforms"):
        for tfm in out.transforms:
            tfm.header.stamp = rospy.Time.from_sec(new_ts)
            tfm.header.frame_id = _prefix_frame(tfm.header.frame_id, vehicle)
            tfm.child_frame_id = _prefix_frame(tfm.child_frame_id, vehicle)

    return out


def _pick_vehicle_and_time(
    topic: str,
    t_sec: float,
    split_sec: float,
    first_start: float,
    second_start: float,
) -> Tuple[str, float, bool]:
    first_half = t_sec < split_sec
    vehicle = "xianfeng" if first_half else "gensui"

    if first_half:
        new_t = t_sec
    else:
        new_t = first_start + (t_sec - second_start)

    write_this = _is_sensor_topic(topic) or first_half
    return vehicle, new_t, write_this


def _write_dual_bag(
    output_bag: str,
    split_sec: float,
    first_start_sec: float,
    second_start_sec: float,
    max_duration_sec: float = 0.0,
) -> int:
    written = 0
    with rosbag.Bag(INPUT_BAG, "r") as in_bag, rosbag.Bag(output_bag, "w") as out_bag:
        for i, (topic, msg, t) in enumerate(in_bag.read_messages(), start=1):
            t_sec = t.to_sec()

            if max_duration_sec > 0.0 and (t_sec - first_start_sec) > max_duration_sec:
                break

            if topic == "/tf_static":
                for vehicle in ("xianfeng", "gensui"):
                    out_msg = _rewrite_message(msg, vehicle, t_sec, True)
                    out_bag.write(topic, out_msg, t=rospy.Time.from_sec(t_sec))
                    written += 1
                if i % 50000 == 0:
                    print(f"已处理输入消息 {i} 条，已写出 {written} 条")
                continue

            vehicle, new_t, write_this = _pick_vehicle_and_time(
                topic, t_sec, split_sec, first_start_sec, second_start_sec
            )
            if not write_this:
                continue

            if not _is_sensor_topic(topic):
                continue

            out_topic = _remap_topic(topic, vehicle)
            is_tf = topic in {"/tf", "/tf_static"}
            out_msg = _rewrite_message(msg, vehicle, new_t, is_tf)
            out_bag.write(out_topic, out_msg, t=rospy.Time.from_sec(new_t))
            written += 1

            if i % 50000 == 0:
                print(f"已处理输入消息 {i} 条，已写出 {written} 条")

    return written


def _reindex_bag(bag_path: str) -> None:
    orig_path = bag_path.replace(".bag", ".orig.bag")
    if os.path.exists(orig_path):
        os.remove(orig_path)
    subprocess.run(["rosbag", "reindex", bag_path], check=True)


def main() -> None:
    if rosbag is None or rospy is None:
        print(f"错误：未检测到ROS Python依赖（rosbag/rospy）：{_import_err}")
        sys.exit(1)

    if not os.path.exists(INPUT_BAG):
        print(f"错误：输入bag不存在: {INPUT_BAG}")
        sys.exit(1)

    if CLEAR_OUTPUT_BAG and os.path.exists(OUTPUT_BAG):
        os.remove(OUTPUT_BAG)

    with rosbag.Bag(INPUT_BAG, "r") as in_bag:
        start_sec = in_bag.get_start_time()
        end_sec = in_bag.get_end_time()
        split_sec = start_sec + 0.5 * (end_sec - start_sec)

        second_iter = in_bag.read_messages(start_time=rospy.Time.from_sec(split_sec))
        try:
            _, _, t2 = next(second_iter)
            second_start_sec = t2.to_sec()
        except StopIteration:
            print("错误：未找到后半段数据，无法切分")
            sys.exit(1)

        first_start_sec = start_sec

    print("=== teaching2 双车重组开始 ===")
    print(f"输入bag: {INPUT_BAG}")
    print(f"输出bag: {OUTPUT_BAG}")
    print(f"切分时间: {split_sec:.6f}")
    print(f"前半段起点: {first_start_sec:.6f}")
    print(f"后半段起点: {second_start_sec:.6f}")

    print("=== 生成 teaching2_dual.bag ===")
    written_full = _write_dual_bag(
        OUTPUT_BAG, split_sec, first_start_sec, second_start_sec, max_duration_sec=0.0
    )
    print("完成。")
    print(f"写出消息: {written_full}")

    print("=== 写入索引 (rosbag reindex) ===")
    _reindex_bag(OUTPUT_BAG)
    print("索引完成。")


if __name__ == "__main__":
    main()
