#include <iostream>
#include <chrono>
#include "Eigen/Dense"
#include <functional>

using namespace std;
using namespace Eigen;

MatrixXd Generate2DMatrixByEigen()
{
    // 直接使用内置的Random，产生均匀分布随机矩阵
    MatrixXd m = MatrixXd::Random(3,3);
    
    // 也可以调用自定义的随机数生成函数填充数据
    // MatrixXd m = MatrixXd::Zero(3,3).unaryExpr(std::bind(GenerateRandomRealValue));
    return m;
}

void ModeEigen(const MatrixXd& a, const MatrixXd& b)
{
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto c = a * b;
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    std::cout << "Eigen takes " << time_span.count() << " ms\n";
    // cout << "matrix c:\n" << c << endl;
}

int main()
{
    // 创建两个3x3矩阵并填充随机数
    MatrixXd matrix_a = Generate2DMatrixByEigen();
    MatrixXd matrix_b = Generate2DMatrixByEigen();

    // 计算矩阵乘法
    ModeEigen(matrix_a, matrix_b);

    return 0;
}

