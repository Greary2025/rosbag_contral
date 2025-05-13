// gtsam-examples的MATLAB封装声明

// gtsam前置声明
class gtsam::Point2;  // 2D点类
class gtsam::Pose2;   // 2D位姿类

class gtsam::GaussianFactorGraph;  // 高斯因子图
class gtsam::Values;               // 变量值容器
virtual class gtsam::noiseModel::Base;  // 噪声模型基类
virtual class gtsam::NonlinearFactor;   // 非线性因子基类
virtual class gtsam::NonlinearFactorGraph; // 非线性因子图
virtual class gtsam::NoiseModelFactor;     // 带噪声模型的因子基类

// 自定义命名空间
namespace gtsamexamples {

// 包含GPSPose2Factor的头文件
#include <cpp/GPSPose2Factor.h>

// 为Pose2设计的GPS因子类
virtual class GPSPose2Factor : gtsam::NoiseModelFactor {
  // 构造函数
  // 参数: poseKey - 位姿变量键值
  //       m - 测量值(Point2类型)
  //       model - 噪声模型指针
  GPSPose2Factor(size_t poseKey, const gtsam::Point2& m, gtsam::noiseModel::Base* model);
};

} // namespace gtsamexamples

