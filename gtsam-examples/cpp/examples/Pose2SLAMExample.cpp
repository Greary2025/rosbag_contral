/**
 * @file Pose2SLAMExample.cpp
 * @brief 2D SLAM示例
 * @date Nov 7, 2016
 * @author Jing Dong
 */

/**
 * 一个简单的2D位姿图SLAM示例
 * 机器人从x1移动到x5，每步之间有里程计信息
 * 机器人每步移动5个单位，在x3-x5处右转90度
 * 在x5处有一个到x2的回环闭合
 * 图结构如下：
 * 
 *  p-x1 - x2 - x3
 *         |    |
 *         x5 - x4 
 */

// 在平面情况下，我们使用Pose2变量(x,y,theta)表示机器人在SE(2)中的位姿
#include <gtsam/geometry/Pose2.h>

// 因子图类，各种因子的容器
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// 图节点值类，各种几何类型的容器
// 这里Values用作SE(2)的容器
#include <gtsam/nonlinear/Values.h>

// 符号类用于索引值中的变量
// 例如：位姿变量通常索引为'x'+数字，地标为'l'+数字
#include <gtsam/inference/Symbol.h>

// 本示例中使用的因子类型
// PriorFactor给出变量的先验分布
// BetweenFactor给出里程计约束
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// 优化器类，这里使用高斯牛顿法
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// 计算优化后变量的(近似/线性化)边缘协方差
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // 创建因子图容器
  NonlinearFactorGraph graph;

  // 添加第一个位姿的先验，设置为原点
  // 先验用于将整个轨迹固定/对齐到世界坐标系
  // 先验因子包含均值(原点)和噪声模型(协方差矩阵)
  noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Vector3(1.0, 1.0, 0.1));
  graph.add(PriorFactor<Pose2>(Symbol('x', 1), Pose2(0, 0, 0), priorModel));

  // 里程计测量噪声模型(协方差矩阵)
  noiseModel::Diagonal::shared_ptr odomModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // 添加里程计因子
  // 在连续位姿之间创建里程计(Between)因子
  // 机器人在x3-x5处右转90度
  graph.add(BetweenFactor<Pose2>(Symbol('x', 1), Symbol('x', 2), Pose2(5, 0, 0), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 2), Symbol('x', 3), Pose2(5, 0, -M_PI_2), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 3), Symbol('x', 4), Pose2(5, 0, -M_PI_2), odomModel));
  graph.add(BetweenFactor<Pose2>(Symbol('x', 4), Symbol('x', 5), Pose2(5, 0, -M_PI_2), odomModel));

  // 回环闭合测量噪声模型
  noiseModel::Diagonal::shared_ptr loopModel = noiseModel::Diagonal::Sigmas(Vector3(0.5, 0.5, 0.1));

  // 添加回环闭合约束
  graph.add(BetweenFactor<Pose2>(Symbol('x', 5), Symbol('x', 2), Pose2(5, 0, -M_PI_2), loopModel));
  
  // 打印因子图
  graph.print("\n因子图:\n"); 

  // 优化的初始变量值
  // 从真实值添加随机噪声
  Values initials;
  initials.insert(Symbol('x', 1), Pose2(0.2, -0.3, 0.2));
  initials.insert(Symbol('x', 2), Pose2(5.1, 0.3, -0.1));
  initials.insert(Symbol('x', 3), Pose2(9.9, -0.1, -M_PI_2 - 0.2));
  initials.insert(Symbol('x', 4), Pose2(10.2, -5.0, -M_PI + 0.1));
  initials.insert(Symbol('x', 5), Pose2(5.1, -5.1, M_PI_2 - 0.1));
  
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
  cout << "x4 协方差:\n" << marginals.marginalCovariance(Symbol('x', 4)) << endl;
  cout << "x5 协方差:\n" << marginals.marginalCovariance(Symbol('x', 5)) << endl;

  return 0;
}
