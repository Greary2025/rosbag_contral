cmake_minimum_required(VERSION 3.10)

project(PCL_GPU_TEST CUDA CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找CUDA包
find_package(CUDA REQUIRED)

# 查找PCL包
find_package(PCL 1.15 REQUIRED)

# 查找Eigen包
find_package(Eigen3 REQUIRED)

# 设置包含目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(pcl_gpu pcl_gpu.cpp)
add_executable(matrix_eigen matrix_eigen.cpp)
add_executable(matrix_eigen_gpu matrix_eigen_gpu.cu)
add_executable(matrix_eigen_large matrix_eigen_large.cpp)
add_executable(matrix_eigen_gpu_large matrix_eigen_gpu_large.cu)

# 链接库
target_link_libraries(pcl_gpu ${PCL_LIBRARIES})
target_link_libraries(matrix_eigen Eigen3::Eigen)
target_link_libraries(matrix_eigen_gpu Eigen3::Eigen)
target_link_libraries(matrix_eigen_large Eigen3::Eigen)
target_link_libraries(matrix_eigen_gpu_large Eigen3::Eigen)

# 设置CUDA架构
set_target_properties(matrix_eigen_gpu PROPERTIES
    CUDA_ARCHITECTURES native)
set_target_properties(matrix_eigen_gpu_large PROPERTIES
    CUDA_ARCHITECTURES native)

# sample.pcd用时
# cpu use :276.346[msec] 
# gpu use :943.197[msec]
# sample.pcd用时
# cpu use :7870.46[msec] 
# gpu use :1180.71[msec] 
# 这个结果和点云的数量有关
# 使用GPU计算法向量时用时最多的是点云邻域搜索这一块，
# 而这一块是用CPU计算的，邻域搜索点设置的越多，计算时间越久，
# 也就是说GPU计算法向量本身的速度是非常快的，用时主要在是用CPU搜索领域这一块。
# 不过在test文件里有利用GPU和八叉树搜索邻域的方法。