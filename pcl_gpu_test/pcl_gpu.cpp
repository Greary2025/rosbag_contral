/**
 * @file pcl_gpu.cpp
 * @brief 比较CPU和GPU版本的法线估计性能
 * 该程序演示了使用PCL库在CPU和GPU上计算点云法线，并比较两者的耗时
 */

#include <pcl/point_types.h>       // PCL点云数据类型
#include <pcl/io/pcd_io.h>         // PCD文件IO
#include <pcl/search/impl/kdtree.hpp>  // KD树实现
#include <pcl/search/kdtree.h>     // KD树接口
#include <pcl/features/normal_3d.h> // 法线估计
#include <pcl/gpu/features/features.hpp> // GPU特征计算
#include <pcl/common/io.h>  // 添加PCL版本信息头文件

#include <chrono>  // 计时功能
#include <vector>  // STL向量容器

int main()
{
    // 输出PCL版本信息
    std::cout << "PCL version: " << PCL_VERSION_PRETTY << std::endl;

    // 1. 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("your_pointcloud_file.pcd", *cloud);  // 从PCD文件加载点云
    pcl::io::loadPCDFile("../sample.pcd", *cloud);  // 从PCD文件加载点云
    
    // -------------------- CPU版本法线估计 --------------------------------------
    auto t1 = std::chrono::high_resolution_clock::now();  // 开始计时
    
    // 创建KD树用于最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
   
    // 创建法线估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);      // 设置输入点云
    ne.setSearchMethod(tree);     // 设置搜索方法
    ne.setRadiusSearch(0.03);     // 设置搜索半径(3cm)

    // 计算法线
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);   // 执行法线计算

    // 结束计时并输出耗时
    auto t2 = std::chrono::high_resolution_clock::now();
    double time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
    std::cout << std::endl << "cpu use :" << time << "[msec] " << std::endl;

    // ----------------------- GPU版本法线估计 ------------------------------------
    t1 = std::chrono::high_resolution_clock::now();  // 重新开始计时
    
    // 上传点云数据到GPU
    pcl::gpu::NormalEstimation::PointCloud gpuCloud;
    gpuCloud.upload(cloud->points);

    // 创建KD树用于最近邻搜索(CPU端)
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    // 准备邻居索引数据
    size_t cloud_size = cloud->points.size();
    std::vector<float> dists;
    std::vector<std::vector<int>> neighbors_all;
    std::vector<int> sizes;
    neighbors_all.resize(cloud_size);
    sizes.resize(cloud_size);
    
    // 使用OpenMP并行搜索最近邻
    #pragma omp parallel for
    for (int i = 0; i < cloud_size; ++i) {
        kdtree->nearestKSearch(cloud->points[i], 20, neighbors_all[i], dists);
        sizes[i] = (int)neighbors_all[i].size();
    }

    // 处理邻居索引数据
    int max_nn_size = *max_element(sizes.begin(), sizes.end());
    std::vector<int> temp_neighbors_all(max_nn_size * cloud->size());
    pcl::gpu::PtrStep<int> ps(&temp_neighbors_all[0], max_nn_size * pcl::gpu::PtrStep<int>::elem_size);
    
    // 复制邻居索引数据
    for (size_t i = 0; i < cloud->size(); ++i)
        std::copy(neighbors_all[i].begin(), neighbors_all[i].end(), ps.ptr(i));

    // 上传邻居索引到GPU
    pcl::gpu::NeighborIndices indices;
    indices.upload(temp_neighbors_all, sizes, max_nn_size);

    // GPU计算法线
    pcl::gpu::NormalEstimation::Normals normals;
    pcl::gpu::NormalEstimation::computeNormals(gpuCloud, indices, normals);
    
    // 结束计时并输出耗时
    t2 = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
    std::cout << std::endl << "gpu use :" << time << "[msec] " << std::endl;

    return 0;  // 程序正常退出
}