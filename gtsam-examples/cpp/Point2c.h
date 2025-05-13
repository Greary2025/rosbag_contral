/**
 * @file Point2c.h
 * @brief 自定义2D点类，通过traits使其可在GTSAM中优化
 * @date Nov 8, 2016
 * @author Jing Dong
 */

// 防止头文件重复包含
#pragma once

// 包含GTSAM必要头文件
#include <gtsam/base/Matrix.h>  // 矩阵运算
#include <gtsam/base/Vector.h>  // 向量运算
#include <gtsam/base/Lie.h>     // Lie群相关定义

// 标准库头文件
#include <cmath>     // 数学函数
#include <iostream>  // 输入输出

// 自定义命名空间
namespace gtsamexamples {

// 最小化的2D点类，'c'表示custom(自定义)
struct Point2c {
  double x;  // x坐标
  double y;  // y坐标
  
  // 便捷构造函数
  Point2c(double xi, double yi) : x(xi), y(yi) {}
};

} // namespace gtsamexamples


/** 
 * Point2c的traits定义
 *
 * 任何与GTSAM兼容的类型必须在其traits中包含：
 *   - 带可选前缀字符串的Print函数
 *   - 带可选容差的Equal函数
 *
 * 流形类型必须包含：
 *   - 维度定义
 *   - TangentVector类型定义
 *   - Local坐标函数
 *   - Retract回流形函数
 *
 * Lie群类型必须包含：
 *   - Identity函数
 *   - Logmap函数(可选Jacobian)
 *   - Expmap函数(可选Jacobian) 
 *   - Compose函数(可选Jacobian)
 *   - Between函数(可选Jacobian)
 *   - Inverse函数(可选Jacobian)
 */

// traits必须定义在gtsam命名空间
namespace gtsam {

// Point2c的特化traits
template<>
struct traits<gtsamexamples::Point2c> {

  // 结构类别：这是一个Lie群
  typedef lie_group_tag structure_category;

  /* 基本功能(Testable) */
  
  // 打印函数
  static void Print(const gtsamexamples::Point2c& m, const std::string& str = "") {
    std::cout << str << "(" << m.x << ", " << m.y << ")" << std::endl;
  }
  
  // 带容差的相等判断
  static bool Equals(const gtsamexamples::Point2c& m1, const gtsamexamples::Point2c& m2, 
      double tol = 1e-8) {
    return fabs(m1.x - m2.x) < tol && fabs(m1.y - m2.y) < tol;
  }

  /* 流形功能 */

  // 固定维度为2
  enum { dimension = 2 };
  static int GetDimension(const gtsamexamples::Point2c&) { return dimension; }
  
  // 类型定义
  typedef gtsamexamples::Point2c ManifoldType;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  
  // Local坐标函数(向量空间简单实现)
  static TangentVector Local(const gtsamexamples::Point2c& origin, 
      const gtsamexamples::Point2c& other) {
    return Vector2(other.x - origin.x, other.y - origin.y);
  }
  
  // 回流形函数(向量空间简单实现)
  static gtsamexamples::Point2c Retract(const gtsamexamples::Point2c& origin, 
      const TangentVector& v) {
    return gtsamexamples::Point2c(origin.x + v(0), origin.y + v(1));
  }

  /* Lie群功能 */
  
  // 使用乘法运算符*的组合方式
  typedef multiplicative_group_tag group_flavor;
   
  // 类型定义
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;
  
  // 单位元素
  static gtsamexamples::Point2c Identity() { 
    return gtsamexamples::Point2c(0, 0);
  }
  
  // 对数映射
  static TangentVector Logmap(const gtsamexamples::Point2c& m, 
      ChartJacobian Hm = boost::none) {
    if (Hm) *Hm = Matrix2::Identity(); 
    return Vector2(m.x, m.y);
  }

  // 指数映射
  static gtsamexamples::Point2c Expmap(const TangentVector& v, 
      ChartJacobian Hv = boost::none) {
    if (Hv) *Hv = Matrix2::Identity(); 
    return gtsamexamples::Point2c(v(0), v(1));
  }

  // 组合运算
  static gtsamexamples::Point2c Compose(const gtsamexamples::Point2c& m1, 
      const gtsamexamples::Point2c& m2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = Matrix2::Identity();
    if (H2) *H2 = Matrix2::Identity(); 
    return gtsamexamples::Point2c(m1.x + m2.x, m1.y + m2.y);
  }

  // 相对运算
  static gtsamexamples::Point2c Between(const gtsamexamples::Point2c& m1, 
      const gtsamexamples::Point2c& m2,
      ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    if (H1) *H1 = -Matrix2::Identity();
    if (H2) *H2 = Matrix2::Identity(); 
    return gtsamexamples::Point2c(m2.x - m1.x, m2.y - m1.y);
  }

  // 逆运算
  static gtsamexamples::Point2c Inverse(const gtsamexamples::Point2c& m,
      ChartJacobian H = boost::none) {
    if (H) *H = -Matrix2::Identity(); 
    return gtsamexamples::Point2c(-m.x, -m.y);
  }
};

} // namespace gtsam

// 自定义命名空间中定义运算符*
namespace gtsamexamples {

  // 定义*运算符实现组合运算
  Point2c operator*(const Point2c& m1, const Point2c& m2) {
    return Point2c(m1.x + m2.x, m1.y + m2.y);
  }
}





