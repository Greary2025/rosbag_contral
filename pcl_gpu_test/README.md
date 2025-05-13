## 安装pcl-1.15
### 首先安装pcl各种依赖
```bash
sudo apt-get update
sudo apt-get install git build-essential linux-libc-dev
sudo apt-get install cmake cmake-gui
sudo apt-get install libusb-1.0-0-dev libusb-dev libudev-dev
sudo apt-get install mpi-default-dev openmpi-bin openmpi-common
sudo apt-get install libflann1.9 libflann-dev # ubuntu20.4对应1.9
sudo apt-get install libeigen3-dev
sudo apt-get install libboost-all-dev
sudo apt-get install libqhull* libgtest-dev
sudo apt-get install freeglut3-dev pkg-config
sudo apt-get install libxmu-dev libxi-dev
sudo apt-get install mono-complete
sudo apt-get install libopenni-dev
sudo apt-get install libopenni2-dev
```
### 安装VTK
```bash
sudo apt-get install libx11-dev libxext-dev libxtst-dev libxrender-dev libxmu-dev libxmuu-dev
```
```bash
sudo apt-get install build-essential libgl1-mesa-dev libglu1-mesa-dev
```
```bash
sudo apt-get install cmake cmake-gui
```

### 下载pcl-1.15
```bash
git clone https://github.com/PointCloudLibrary/pcl.git 
```
### 编译pcl-1.15(需具备CUDA和CMake，仔细看cmake的过程)
```bash
cd pcl
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_GPU=ON \
      -DBUILD_apps=ON \
      -DBUILD_examples=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      ..
make -j8
sudo make install
```
#### 检查过程：
https://zhuanlan.zhihu.com/p/622952543

## 安装cuda-11.8
### 首先访问NVIDIA官方CUDA Toolkit存档页面下载11.8版本：
```bash
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
```
### 如果没有安装CUDA,安装CUDA 11.8：
```bash
sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-8-local/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda-11-8
```
### 如果安装过CUDA
#### 卸载旧版CUDA
```bash
sudo apt-get --purge remove "*cublas*" "*cufft*" "*curand*" "*cusolver*" "*cusparse*" "*npp*" "*nvjpeg*" "cuda*" "nsight*" 
sudo apt-get autoremove
```

#### 安装CUDA 11.8(与PCL兼容)
```bash
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo apt-key add /var/cuda-repo-ubuntu2004-11-8-local/7fa2af80.pub
sudo apt-get update
sudo apt-get install cuda-11-8
```
#### 密钥报错
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 91CD13F9368EAC11
sudo apt-get update
```
#### 仍然报错
##### 下载并安装CUDA官方GPG密钥
```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
sudo apt-key add 3bf863cc.pub
rm 3bf863cc.pub
```
##### 再次更新
```bash
sudo apt-get update
```
##### 如果问题仍然存在，可能需要重新安装CUDA仓库：
```bash
# 先删除现有仓库
sudo rm /etc/apt/sources.list.d/cuda-ubuntu2004-11-8-local.list

# 重新添加仓库
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /"
sudo apt-get update
```
##### 如果仍然失败，可以手动下载并添加密钥：
```bash
wget -qO - https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub | sudo apt-key add -
```
##### 或者直接跳过密钥验证（不推荐，仅作为最后手段）：
```bash
sudo apt-get update --allow-unauthenticated
```

### 设置环境变量(添加到~/.bashrc)：
```bash
echo 'export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc
```
### 验证CUDA安装：
```bash
nvcc --version
```
### 重新编译PCL时，确保指定正确的CUDA路径：
```bash
cd /home/john/pcl/release
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DBUILD_GPU=ON \
      -DBUILD_CUDA=ON \
      -DBUILD_apps=ON \
      -DBUILD_examples=ON \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 ..
```
## 安装cmake-3.30.8
### 下载cmake安装脚本
```bash
wget https://cmake.org/files/v3.30/cmake-3.30.8-linux-x86_64.sh
chmod +x cmake-3.30.8-linux-x86_64.sh
sudo ./cmake-3.30.8-linux-x86_64.sh --skip-licence --prefix=/usr
```
#### 在弹出来的提示中，输入y/n
```bash
# 安装过程中遇到：
# 第一个选择时，输入y!!!
Do you accept the license? [yn]: 
# 输入 y

# 第二个选择时，输入n!!!
By default the CMake will be installed in:
  "/usr/cmake-3.23.0-linux-x86_64"
Do you want to include the subdirectory cmake-3.23.0-linux-x86_64?
Saying no will install in: "/usr" [Yn]:
# 输入 n
```
### 查看cmake版本
```bash
cmake --version
```

### 重新编译PCL时，确保指定正确的CUDA路径：
```bash
cd /home/john/pcl/release
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr \
      -DBUILD_GPU=ON \
      -DBUILD_CUDA=ON \
      -DBUILD_apps=ON \
      -DBUILD_examples=ON \
      -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8 ..
```