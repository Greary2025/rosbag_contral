<div align="center">
  <h1>rosbag_contral</h1>
  <p>ROS package for rosbag processing, pose conversion, and trajectory evaluation utilities.</p>
  <p>
    <img src="https://img.shields.io/badge/ROS-Catkin-22314E?style=flat-square" alt="ROS Catkin" />
    <img src="https://img.shields.io/badge/Language-C++%20%7C%20Python-5D6D7E?style=flat-square" alt="Languages" />
    <img src="https://img.shields.io/badge/Status-Research%20Toolkit-2C3E50?style=flat-square" alt="Status" />
  </p>
</div>

---

## Abstract

This repository contains a ROS package focused on rosbag data handling, pose/trajectory conversion, and evaluation workflows. The core C++ node is built via [CMakeLists.txt](CMakeLists.txt) and Python utilities live under the scripts/ directory with conversion, visualization, and bag processing helpers.

---

## Code Artifacts

| Category | Content | Notes |
| --- | --- | --- |
| C++ node | [src/filter_teaching2_dual_lidar_imu.cpp](src/filter_teaching2_dual_lidar_imu.cpp) | Built as the `filter_teaching2_dual_lidar_imu` executable via [CMakeLists.txt](CMakeLists.txt). |
| ROS package metadata | [package.xml](package.xml) | Dependencies and package metadata. |
| Evaluation notes | [evo.md](evo.md) | Example `evo_ape`, `evo_rpe`, `evo_traj` commands. |
| Pose transform doc | [scripts/convert/README_transform_poses.md](scripts/convert/README_transform_poses.md) | CSV pose transform rule and usage. |

---

## Directory Layout

```
.
├── CMakeLists.txt
├── package.xml
├── evo.md
├── config
├── launch
├── scripts
└── src
```

---

## Scripts Catalog

### rosbag utilities

- [scripts/rosbag/add_ring.py](scripts/rosbag/add_ring.py)
- [scripts/rosbag/bag_to_ereasor.py](scripts/rosbag/bag_to_ereasor.py)
- [scripts/rosbag/bag_to_pcd.py](scripts/rosbag/bag_to_pcd.py)
- [scripts/rosbag/build_long2_bag_2026_02_19.py](scripts/rosbag/build_long2_bag_2026_02_19.py)
- [scripts/rosbag/check_timestamps.py](scripts/rosbag/check_timestamps.py)
- [scripts/rosbag/extract_gt_asynchronous_0115_D3_2026_02_19.py](scripts/rosbag/extract_gt_asynchronous_0115_D3_2026_02_19.py)
- [scripts/rosbag/filter_teaching2_dual_lidar_imu.py](scripts/rosbag/filter_teaching2_dual_lidar_imu.py)
- [scripts/rosbag/imu_to_txt.py](scripts/rosbag/imu_to_txt.py)
- [scripts/rosbag/pcd_utils.py](scripts/rosbag/pcd_utils.py)
- [scripts/rosbag/pose_to_tum.py](scripts/rosbag/pose_to_tum.py)
- [scripts/rosbag/split_teaching2_to_dual_2026_02_19.py](scripts/rosbag/split_teaching2_to_dual_2026_02_19.py)

### conversion

- [scripts/convert/convert_csv_to_json.py](scripts/convert/convert_csv_to_json.py)
- [scripts/convert/convert_pose.py](scripts/convert/convert_pose.py)
- [scripts/convert/convert_quaternion.py](scripts/convert/convert_quaternion.py)
- [scripts/convert/test_transform.py](scripts/convert/test_transform.py)
- [scripts/convert/transform_poses.py](scripts/convert/transform_poses.py)

### visualization

- [scripts/visualize/visualize_poses.py](scripts/visualize/visualize_poses.py)
- [scripts/visualize/visualize_two_tum.py](scripts/visualize/visualize_two_tum.py)

### misc

- [scripts/utils/rename_pcd.py](scripts/utils/rename_pcd.py)

---

## Config and Data Artifacts

- [config/params.yaml](config/params.yaml)
- [config/point_raw.txt](config/point_raw.txt)
- [config/pose.json](config/pose.json)
- [config/poses_lidar2body.csv](config/poses_lidar2body.csv)
- [config/test_poses_original.csv](config/test_poses_original.csv)
- [config/trajectory_tum_30fps.txt](config/trajectory_tum_30fps.txt)
- [config/trajectory_tum.txt](config/trajectory_tum.txt)

---

## Launch

- [launch/run.launch](launch/run.launch)

---

## Dependencies (from package.xml)

- `catkin`
- `roscpp`
- `rospy`
- `std_msgs`
- `rosbag`
- `sensor_msgs`
- `tf2_msgs`

---

## Notes

- The license field is still TODO in [package.xml](package.xml).
- The pose transform rule and CSV format expectations are documented in [scripts/convert/README_transform_poses.md](scripts/convert/README_transform_poses.md).
