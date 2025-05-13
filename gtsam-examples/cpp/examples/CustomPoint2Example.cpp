/**
 * @file CustomPoint2Example.cpp
 * @brief 使用自定义Point2c类型的2D示例
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * 一个简单的2D位姿图开环示例
 * 机器人从x1移动到x5，步长为2，每步之间有里程计信息
 * 第一个状态有先验因子，其余为开环
 * 图结构如下：
 * 
 *  p-x1 - x2 - x3 - x4 - x5
 */

// 自定义Point2类型头文件
#include "../Point2c.h"

// GTSAM头文件
#include <gtsam/nonlinear/NonlinearFactorGraph.h> // 非线性因子图
#include <gtsam/nonlinear/Values.h>              // 变量值容器
#include <gtsam/inference/Symbol.h>              // 符号变量
#include <gtsam/slam/PriorFactor.h>              // 先验因子
#include <gtsam/slam/BetweenFactor.h>            // 里程计因子
#include <gtsam/nonlinear/GaussNewtonOptimizer.h> // 高斯牛顿优化器
#include <gtsam/nonlinear/Marginals.h>           // 边缘化计算

using namespace std;
using namespace gtsam;
using namespace gtsamexamples;

int main(int argc, char** argv) {

  // 创建因子图容器
  NonlinearFactorGraph graph;

  // 第一个状态的先验噪声模型(协方差矩阵)
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector2(0.2, 0.2));
  
  // 在第一个状态(原点)添加先验因子
  graph.add(PriorFactor<Point2c>(Symbol('x', 1), Point2c(0, 0), priorModel));

  // 里程计测量噪声模型(协方差矩阵)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));

  // 添加里程计因子
  // 在连续Point2c之间创建Between因子
  graph.add(BetweenFactor<Point2c>(Symbol('x', 1), Symbol('x', 2), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 2), Symbol('x', 3), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 3), Symbol('x', 4), Point2c(2, 0), odomModel));
  graph.add(BetweenFactor<Point2c>(Symbol('x', 4), Symbol('x', 5), Point2c(2, 0), odomModel));
  
  // 打印因子图
  graph.print("\n因子图:\n"); 

  // 优化的初始变量值
  // 从真实值添加随机噪声
  Values initials;
  initials.insert(Symbol('x', 1), Point2c(0.2, -0.3));
  initials.insert(Symbol('x', 2), Point2c(2.1, 0.3));
  initials.insert(Symbol('x', 3), Point2c(3.9, -0.1));
  initials.insert(Symbol('x', 4), Point2c(5.9, -0.3));
  initials.insert(Symbol('x', 5), Point2c(8.2, 0.1));
  
  // 打印初始值
  initials.print("\n初始值:\n"); 

  // 使用高斯牛顿法优化初始值
  GaussNewtonParams parameters;
  
  // 设置每次迭代打印错误信息
  parameters.setVerbosity("ERROR");
  
  // 执行优化
  GaussNewtonOptimizer optimizer(graph, initials, parameters);
  Values results = optimizer.optimize();
  
  // 打印优化结果
  results.print("最终结果:\n");

  // 计算所有位姿的边缘协方差
  Marginals marginals(graph, results);
  
  // 打印边缘协方差
  cout << "x1 协方差:\n" << marginals.marginalCovariance(Symbol('x', 1)) << endl;
  cout << "x2 协方差:\n" << marginals.marginalCovariance(Symbol('x', 2)) << endl;
  cout << "x3 协方差:\n" << marginals.marginalCovariance(Symbol('x', 3)) << endl;
  cout << "x4 协方差:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
  cout << "x5 协方差:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

  return 0;
}
