/**
 * @file GPSPose2Factor.h
 * @brief 2D 'GPS'类似因子，用于Pose2位姿
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * 一个简单的2D 'GPS'类似因子
 * 该因子包含对Pose2的X-Y位置测量(mx, my)，但不包含旋转信息
 * 误差向量为[x-mx, y-my]'
 */

// 防止头文件重复包含
#pragma once

// 包含GTSAM库必要头文件
#include <gtsam/nonlinear/NonlinearFactor.h>  // 非线性因子基类
#include <gtsam/base/Matrix.h>  // 矩阵类型
#include <gtsam/base/Vector.h>  // 向量类型
#include <gtsam/geometry/Pose2.h>  // 二维位姿类型

// 自定义命名空间(根据项目需要)
namespace gtsamexamples {

// 继承自NoiseModelFactor1<Pose2>的GPS因子类
class GPSPose2Factor: public gtsam::NoiseModelFactor1<gtsam::Pose2> {

private:
  // 测量值存储
  double mx_;  // X坐标测量值
  double my_;  // Y坐标测量值

public:
  /**
   * 构造函数
   * @param poseKey  关联的位姿变量键值
   * @param m        Point2类型的测量值
   * @param model    GPS传感器的噪声模型(X-Y方向)
   */
  GPSPose2Factor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<gtsam::Pose2>(model, poseKey), mx_(m.x()), my_(m.y()) {}

  /**
   * 误差计算函数
   * @param p    输入的Pose2位姿
   * @param H    可选的雅可比矩阵输出参数(使用boost optional，默认为空指针)
   * @return     误差向量[x-mx, y-my]'
   */
  gtsam::Vector evaluateError(const gtsam::Pose2& p, boost::optional<gtsam::Matrix&> H = boost::none) const {
  
    // 当H非空时计算雅可比矩阵
    // 雅可比矩阵结构: 2x3矩阵，前两列为单位矩阵，最后一列为0
    if (H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0, 
                                      0.0, 1.0, 0.0).finished();
    
    // 返回误差向量
    return (gtsam::Vector2() << p.x() - mx_, p.y() - my_).finished();
  }
};

} // namespace gtsamexamples

