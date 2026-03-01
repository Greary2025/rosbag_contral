#!/usr/bin/env python3

"""
从 long2 数据目录合成 ROS bag（long2.bag）。

输入目录结构（固定命名）：
- xianfeng_velodyne/*.bin
- gensui_velodyne/*.bin
- xianfeng_image_02/*.png
- gensui_image_02/*.png
- xianfeng_lidar.txt / gensui_lidar.txt
- xianfeng_oxts.txt / gensui_oxts.txt
- xianfeng_pose.txt / gensui_pose.txt

输出话题：
- /xianfeng/lidar, /gensui/lidar                  (sensor_msgs/PointCloud2)
- /xianfeng/imu, /gensui/imu                      (sensor_msgs/Imu)
- /xianfeng/pose, /gensui/pose                    (nav_msgs/Odometry)
- /xianfeng/image_02/compressed, /gensui/image_02/compressed (sensor_msgs/CompressedImage)

最后修改：2026-02-19
"""

import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, List, Tuple

import numpy as np

try:
    import rosbag
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
except Exception as e:
    rosbag = None  # type: ignore
    rospy = None  # type: ignore
    Odometry = None  # type: ignore
    Imu = None  # type: ignore
    PointCloud2 = None  # type: ignore
    PointField = None  # type: ignore
    CompressedImage = None  # type: ignore
    _import_err = e
else:
    _import_err = None


BASE_DIR = Path("/mnt/c/Users/Greary/Documents/rosbag/long2")
OUTPUT_BAG = BASE_DIR / "long2.bag"

CLEAR_OUTPUT_BAG = True


@dataclass
class VehicleData:
    name: str
    lidar_dir: Path
    image_dir: Path
    lidar_ts: List[float]
    imu_rows: List[Tuple[float, float, float, float, float, float, float, float, float, float, float]]
    pose_rows: List[Tuple[float, float, float, float, float, float, float, float]]


def _read_time_list(path: Path) -> List[float]:
    out: List[float] = []
    with path.open("r", encoding="utf-8-sig") as f:
        for line in f:
            s = line.strip().replace("\ufeff", "")
            if not s:
                continue
            out.append(float(s.split()[0]))
    return out


def _read_imu_rows(path: Path) -> List[Tuple[float, float, float, float, float, float, float, float, float, float, float]]:
    rows: List[Tuple[float, float, float, float, float, float, float, float, float, float, float]] = []
    with path.open("r", encoding="utf-8-sig") as f:
        for line in f:
            s = line.strip().replace("\ufeff", "")
            if not s:
                continue
            vals = [float(v) for v in s.split()]
            if len(vals) < 7:
                continue
            if len(vals) < 11:
                vals += [0.0] * (11 - len(vals))
            rows.append((
                vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], vals[9], vals[10]
            ))
    rows.sort(key=lambda x: x[0])
    return rows


def _read_pose_rows(path: Path) -> List[Tuple[float, float, float, float, float, float, float, float]]:
    rows: List[Tuple[float, float, float, float, float, float, float, float]] = []
    with path.open("r", encoding="utf-8-sig") as f:
        for line in f:
            s = line.strip().replace("\ufeff", "")
            if not s:
                continue
            vals = [float(v) for v in s.split()]
            if len(vals) < 8:
                continue
            rows.append((vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7]))
    rows.sort(key=lambda x: x[0])
    return rows


def _load_vehicle(base_dir: Path, name: str) -> VehicleData:
    lidar_dir = base_dir / f"{name}_velodyne"
    image_dir = base_dir / f"{name}_image_02"
    lidar_ts_file = base_dir / f"{name}_lidar.txt"
    imu_file = base_dir / f"{name}_oxts.txt"
    pose_file = base_dir / f"{name}_pose.txt"

    if not lidar_dir.is_dir() or not image_dir.is_dir():
        raise FileNotFoundError(f"目录不存在: {lidar_dir} 或 {image_dir}")
    if not lidar_ts_file.is_file() or not imu_file.is_file() or not pose_file.is_file():
        raise FileNotFoundError(f"文件不存在: {lidar_ts_file} / {imu_file} / {pose_file}")

    return VehicleData(
        name=name,
        lidar_dir=lidar_dir,
        image_dir=image_dir,
        lidar_ts=_read_time_list(lidar_ts_file),
        imu_rows=_read_imu_rows(imu_file),
        pose_rows=_read_pose_rows(pose_file),
    )


def _build_pointcloud2(points_xyzi: np.ndarray, stamp: float, frame_id: str) -> Any:
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points_xyzi.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = np.asarray(points_xyzi, dtype=np.float32).tobytes()
    return msg


def _write_vehicle_lidar_and_image(bag: Any, vehicle: VehicleData) -> None:
    lidar_topic = f"/{vehicle.name}/lidar"
    image_topic = f"/{vehicle.name}/image_02/compressed"
    frame_lidar = f"{vehicle.name}_lidar"
    frame_cam = f"{vehicle.name}_camera"

    bins = sorted(vehicle.lidar_dir.glob("*.bin"))
    imgs = sorted(vehicle.image_dir.glob("*.png"))

    n = min(len(bins), len(imgs), len(vehicle.lidar_ts))
    if n == 0:
        print(f"警告: {vehicle.name} 没有可写入的雷达/图像数据")
        return

    if len(bins) != len(vehicle.lidar_ts) or len(imgs) != len(vehicle.lidar_ts):
        print(
            f"警告: {vehicle.name} 数据数量不一致, bin={len(bins)}, png={len(imgs)}, ts={len(vehicle.lidar_ts)}, 按最小数量 {n} 写入"
        )

    for i in range(n):
        ts = vehicle.lidar_ts[i]

        cloud = np.fromfile(str(bins[i]), dtype=np.float32)
        if cloud.size % 4 != 0:
            print(f"警告: 跳过异常点云文件 {bins[i].name}, float数={cloud.size}")
            continue
        cloud = cloud.reshape((-1, 4))
        cloud_msg = _build_pointcloud2(cloud, ts, frame_lidar)
        bag.write(lidar_topic, cloud_msg, t=rospy.Time.from_sec(ts))

        img_msg = CompressedImage()
        img_msg.header.stamp = rospy.Time.from_sec(ts)
        img_msg.header.frame_id = frame_cam
        img_msg.format = "png"
        with imgs[i].open("rb") as f:
            img_msg.data = f.read()
        bag.write(image_topic, img_msg, t=rospy.Time.from_sec(ts))

        if (i + 1) % 200 == 0:
            print(f"{vehicle.name}: 已写入雷达+图像 {i + 1}/{n}")


def _write_vehicle_imu(bag: Any, vehicle: VehicleData) -> None:
    topic = f"/{vehicle.name}/imu"
    frame = f"{vehicle.name}_imu"

    for i, row in enumerate(vehicle.imu_rows):
        ts, ax, ay, az, gx, gy, gz, _q1, _q2, _q3, _q4 = row
        msg = Imu()
        msg.header.stamp = rospy.Time.from_sec(ts)
        msg.header.frame_id = frame
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        bag.write(topic, msg, t=rospy.Time.from_sec(ts))

        if (i + 1) % 2000 == 0:
            print(f"{vehicle.name}: 已写入IMU {i + 1}/{len(vehicle.imu_rows)}")


def _write_vehicle_pose(bag: Any, vehicle: VehicleData) -> None:
    topic = f"/{vehicle.name}/pose"
    frame = "map"
    child = f"{vehicle.name}_base_link"

    for i, row in enumerate(vehicle.pose_rows):
        ts, x, y, z, qx, qy, qz, qw = row
        msg = Odometry()
        msg.header.stamp = rospy.Time.from_sec(ts)
        msg.header.frame_id = frame
        msg.child_frame_id = child
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        bag.write(topic, msg, t=rospy.Time.from_sec(ts))

        if (i + 1) % 2000 == 0:
            print(f"{vehicle.name}: 已写入Pose {i + 1}/{len(vehicle.pose_rows)}")


def main() -> None:
    if rosbag is None or rospy is None:
        print(f"错误：缺少ROS Python依赖: {_import_err}")
        sys.exit(1)

    if not BASE_DIR.is_dir():
        print(f"错误：目录不存在: {BASE_DIR}")
        sys.exit(1)

    if CLEAR_OUTPUT_BAG and OUTPUT_BAG.exists():
        OUTPUT_BAG.unlink()

    xianfeng = _load_vehicle(BASE_DIR, "xianfeng")
    gensui = _load_vehicle(BASE_DIR, "gensui")

    print("=== 开始合成 long2.bag ===")
    print(f"输入目录: {BASE_DIR}")
    print(f"输出bag: {OUTPUT_BAG}")

    with rosbag.Bag(str(OUTPUT_BAG), "w") as bag:
        _write_vehicle_lidar_and_image(bag, xianfeng)
        _write_vehicle_lidar_and_image(bag, gensui)

        _write_vehicle_imu(bag, xianfeng)
        _write_vehicle_imu(bag, gensui)

        _write_vehicle_pose(bag, xianfeng)
        _write_vehicle_pose(bag, gensui)

    print("完成。")
    print(f"已生成: {OUTPUT_BAG}")


if __name__ == "__main__":
    main()
