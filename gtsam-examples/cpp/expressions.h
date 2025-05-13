/**
 * @file expressions.h
 * @brief 自定义表达式函数
 * @date Nov 8, 2016
 * @author Jing Dong
 */

// 防止头文件重复包含
#pragma once

// 包含自定义函数定义的头文件
#include "functions.h"

// 包含GTSAM库中的表达式定义
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>


// 定义gtsamexamples命名空间
namespace gtsamexamples {

// 定义将Pose2表达式投影到Point2表达式的函数
// 参数: pose - 输入的Pose2表达式
// 返回: Point2表达式
inline gtsam::Point2_ projectPose2_(const gtsam::Pose2_& pose) {
  return gtsam::Point2_(&projectPose2, pose);
}

} // namespace gtsamexamples

