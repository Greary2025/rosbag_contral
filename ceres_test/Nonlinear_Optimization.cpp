#include <iostream>
#include <ceres/ceres.h>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace ceres;
using namespace chrono;

// 第一部分：构建代价函数，重载 () 符号
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
       residual[0] = T(10.0) - x[0];
       return true;
   }
};

// 主函数
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 寻优参数x的初始值，为5
    double initial_x = 5.0;
    double x = initial_x;

    auto total_start = high_resolution_clock::now();
    
    // 第二部分：构建寻优问题
    auto problem_start = high_resolution_clock::now();
    Problem problem;
    // 使用自动求导，将之前的代价函数结构体传入，第一个 1 是输出维度，即残差的维度，第二个 1 是输入维度，即待寻优参数 x 的维度
    CostFunction* cost_function = 
        new AutoDiffCostFunction<CostFunctor, 1, 1>(
        	new CostFunctor
    ); 
    // 向问题中添加误差项
    problem.AddResidualBlock(cost_function, NULL, &x); 
    auto problem_end = high_resolution_clock::now();
    auto problem_duration = duration_cast<milliseconds>(problem_end - problem_start);
    cout << "问题构建用时: " << problem_duration.count() << " 毫秒" << endl;

    // 第三部分： 配置并运行求解器
    auto solve_start = high_resolution_clock::now();
    Solver::Options options;
    // 配置增量方程的解法
    options.linear_solver_type = ceres::DENSE_QR; 
    // 输出到 cout
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;             // 优化信息
    Solve(options, &problem, &summary);  // 求解

    // 输出优化的简要信息
    auto solve_end = high_resolution_clock::now();
    auto solve_duration = duration_cast<milliseconds>(solve_end - solve_start);
    cout << "优化求解用时: " << solve_duration.count() << " 毫秒" << endl;

    auto total_end = high_resolution_clock::now();
    auto total_duration = duration_cast<milliseconds>(total_end - total_start);
    cout << "总执行用时: " << total_duration.count() << " 毫秒" << endl;

    std::cout << summary.BriefReport() << "\n";
	// 最终结果
    std::cout << "x : " << initial_x << " -> " << x << "\n";
    return 0;
}