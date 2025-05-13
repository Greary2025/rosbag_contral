/**
 * @file functions.h  
 * @brief 表达式相关函数声明
 * @date Nov 8, 2016
 * @author Jing Dong
 */

// 防止头文件重复包含
#pragma once

// 包含GTSAM库基础类型头文件
#include <gtsam/base/Matrix.h>  // 矩阵类型
#include <gtsam/base/Vector.h>  // 向量类型
#include <gtsam/geometry/Point2.h>  // 二维点类型
#include <gtsam/geometry/Pose2.h>  // 二维位姿类型

// 自定义命名空间(根据项目需要)
namespace gtsamexamples {

/**
 * @brief 将Pose2位姿投影到Point2点的函数声明
 * @param pose 输入的二维位姿
 * @param H 可选的输出参数，返回2x3雅可比矩阵(默认为boost::none)
 * @return 返回位姿的平移部分(x,y坐标)构成的Point2
 */
gtsam::Point2 projectPose2(const gtsam::Pose2& pose,
    gtsam::OptionalJacobian<2,3> H = boost::none);

} // namespace gtsamexamples
