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

