#include <iostream>
#include <chrono>
#include "Eigen/Dense"
#include <functional>

using namespace std;
using namespace Eigen;

MatrixXd Generate2DMatrixByEigen()
{
    // 直接使用内置的Random，产生均匀分布随机矩阵
    MatrixXd m = MatrixXd::Random(1000, 1000);
    return m;
}

void ModeEigen(const MatrixXd& a, const MatrixXd& b)
{
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto c = a * b;
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    std::cout << "Eigen takes " << time_span.count() << " ms\n";
}

int main()
{
    std::cout << "Generating 1000x1000 random matrices...\n";
    
    // 创建两个1000x1000矩阵并填充随机数
    MatrixXd matrix_a = Generate2DMatrixByEigen();
    MatrixXd matrix_b = Generate2DMatrixByEigen();

    std::cout << "Starting CPU matrix multiplication...\n";
    // 计算矩阵乘法
    ModeEigen(matrix_a, matrix_b);

    return 0;
}