/**
 * @file Pose2GPSExpressionExample.cpp
 * @brief 使用自定义'GPS'表达式的2D示例
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * 一个带有'GPS'测量的简单2D位姿图SLAM
 * 机器人从x1移动到x3，每步之间有里程计信息
 * 每一步都有相关联的'GPS'测量
 * 图结构如下：
 * 
 *  g1   g2   g3
 *  |    |    |
 *  x1 - x2 - x3
 */

// 自定义GPS表达式
#include "../expressions.h"

// between表达式
#include <gtsam/slam/expressions.h>

// GTSAM头文件
#include <gtsam/geometry/Pose2.h>          // 2D位姿
#include <gtsam/nonlinear/ExpressionFactorGraph.h> // 表达式因子图
#include <gtsam/nonlinear/Values.h>        // 初始值
#include <gtsam/inference/Symbol.h>        // 符号变量
#include <gtsam/nonlinear/GaussNewtonOptimizer.h> // 高斯牛顿优化器
#include <gtsam/nonlinear/Marginals.h>     // 边缘化

using namespace std;
using namespace gtsam;
using namespace gtsamexamples;

int main(int argc, char** argv) {

  // 创建表达式因子图容器
  ExpressionFactorGraph graph;
  
  // 定义位姿变量的表达式
  // GTSAM命名约定：表达式和类型以_结尾
  Pose2_ x1_(Symbol('x', 1)), x2_(Symbol('x', 2)), x3_(Symbol('x', 3));

  // 里程计测量噪声模型(协方差矩阵)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // 添加里程计因子
  // 在连续位姿之间创建里程计(Between)因子
  graph.addExpressionFactor(between(x1_, x2_), Pose2(5, 0, 0), odomModel);
  graph.addExpressionFactor(between(x2_, x3_), Pose2(5, 0, 0), odomModel);

  // 2D 'GPS'测量噪声模型，2维
  noiseModel::Diagonal::shared_ptr gpsModel = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));

  // 添加GPS因子，通过组合表达式
  // 注意第一个位姿不需要先验因子，因为GPS提供了全局位置
  // (如果有多个GPS测量还可以提供旋转信息)
  graph.addExpressionFactor(projectPose2_(x1_), Point2(0, 0), gpsModel);
  graph.addExpressionFactor(projectPose2_(x2_), Point2(5, 0), gpsModel);
  graph.addExpressionFactor(projectPose2_(x3_), Point2(10, 0), gpsModel);
  
  // 打印因子图
  graph.print("\n因子图:\n"); 

  // 优化的初始变量值
  // 从真实值添加随机噪声
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -0.2));
  
  // 打印初始值
  initials.print("\n初始值:\n"); 

  // 使用高斯牛顿法优化初始值
  GaussNewtonParams parameters;
  
  // 设置每次迭代打印错误
  parameters.setVerbosity("ERROR");
  
  // 开始优化
  GaussNewtonOptimizer optimizer(graph, initials, parameters);
  Values results = optimizer.optimize();
  
  // 打印最终结果
  results.print("最终结果:\n");

  // 计算所有位姿的边缘协方差
  Marginals marginals(graph, results);
  
  // 打印边缘协方差
  cout << "x1 协方差:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
  cout << "x2 协方差:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
  cout << "x3 协方差:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;

  return 0;
}
