/**
 * @file functions.cpp
 * @brief 表达式相关函数实现
 * @date Nov 8, 2016
 * @author Jing Dong
 */

// 包含自定义函数声明头文件
#include "functions.h"

// 包含GTSAM库相关头文件
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

// 定义gtsamexamples命名空间
namespace gtsamexamples {

/* ************************************************************************** */
// 将Pose2投影到Point2的函数实现
// 参数: pose - 输入的Pose2位姿
//       H - 可选的雅可比矩阵输出参数
// 返回值: 投影后的Point2点(只取Pose2的x,y坐标)
gtsam::Point2 projectPose2(const gtsam::Pose2& pose, gtsam::OptionalJacobian<2,3> H) {
  
  // 如果请求雅可比矩阵，计算并赋值
  // 雅可比矩阵结构: 左侧2x2单位矩阵 + 右侧2x1零矩阵
  if (H) *H = (gtsam::Matrix23() << 1.0, 0.0, 0.0, 
                                    0.0, 1.0, 0.0).finished();
  
  // 返回Pose2的平移部分(x,y坐标)
  return gtsam::Point2(pose.x(), pose.y());
}

} // namespace gtsamexamples
