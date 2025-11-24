## 评估evo指令

### 绝对误差

evo_ape tum fr2_desk_groundtruth.txt orb_slam.txt -p

evo_ape tum fr2_desk_groundtruth.txt orb_slam.txt -r trans_part -a -p --plot_mode xy

#### 方便绘图

evo_ape tum traj.txt --save_results results.zip

### 相对误差

evo_rpe tum fr2_desk_groundtruth.txt orb_slam.txt -r full -d 5 -a -p --plot_mode xyz

### 轨迹

evo_traj tum traj.txt -p

#### 对比
evo_traj tum est_traj.txt --ref real_traj.txt -a -p

evo_traj tum traj.txt --plot_mode xyz --plot_linewidth 2 --plot_reference_color red

### 11.11更新

evo_traj tum /mnt/d/rosbag/Examdata/double_evo/ours/gensui/gensui_trajectory.txt   --ref /mnt/d/rosbag/big_exam/synchronous_1014_gensui/gensui_gt_tum.txt   -a --correct_scale --t_offset 0.0 --t_max_diff 0.05 --plot   --save_plot /mnt/d/rosbag/Examdata/double_evo/ours/gensui_traj_results.png

evo_rpe tum /mnt/d/rosbag/big_exam/synchronous_1014_xianfeng/xianfeng_gt_tum.txt   /mnt/d/rosbag/Examdata/double_evo/ours/xianfeng_trajectory.txt   -a --correct_scale   -r trans_part   -d 1 -u d   --t_offset 0.0 --t_max_diff 0.05   --plot   --save_results /mnt/d/rosbag/Examdata/double_evo/ours/synchronous_xianfeng_rpe_results.zip

evo_ape tum /mnt/d/rosbag/big_exam/synchronous_1014_gensui/gensui_gt_tum.txt /mnt/d/rosbag/Examdata/double_evo/ours/trajectory.txt -a --correct_scale --t_offset 0.0 --t_max_diff 0.05 --plot --save_results /mnt/d/rosbag/Examdata/double_evo/ours/synchronous_gensui_aperesults.zip

