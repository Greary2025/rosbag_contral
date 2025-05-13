#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <iomanip>

using namespace std;
using namespace cv;
using namespace chrono;

// 构建代价函数结构体，abc 为待优化参数，residual 为残差。
struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}
    template <typename T>
    bool operator()(const T* const abc,T* residual)const {
        residual[0]=_y-ceres::exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
        return true;
    }
    const double _x, _y;
};

int main() {
    // 参数初始化设置，abc 初始化为0，白噪声方差为 1
    double a = 3, b = 2, c = 1;
    double w = 1;
    cv::RNG rng;
    double abc[3] = {0, 0, 0};

    auto total_start = high_resolution_clock::now();

    // 生成待拟合曲线的数据散点，储存在 Vector 里
    auto data_gen_start = high_resolution_clock::now();
    std::vector<double> x_data, y_data;
    for(int i = 0; i < 1000; i++) {
    	double x = i / 1000.0;
    	x_data.push_back(x);
    	y_data.push_back(exp(a * x * x + b * x + c ) + rng.gaussian(w));
  	}
    auto data_gen_end = high_resolution_clock::now();
    auto data_gen_duration = duration_cast<milliseconds>(data_gen_end - data_gen_start);
    cout << "数据生成用时: " << data_gen_duration.count() << " 毫秒" << endl;

	// 反复使用 AddResidualBlock 方法
    auto problem_start = high_resolution_clock::now();
	// 将每个点的残差累计求和构建最小二乘优化式
	// 不使用核函数，待优化参数是 abc
  	ceres::Problem problem;
  	for(int i = 0; i < 1000; i++) {
    	problem.AddResidualBlock(
      		new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
      		nullptr,
      		abc
    	);
  	}
    auto problem_end = high_resolution_clock::now();
    auto problem_duration = duration_cast<milliseconds>(problem_end - problem_start);
    cout << "问题构建用时: " << problem_duration.count() << " 毫秒" << endl;

	// 配置求解器并求解，输出结果
    auto solve_start = high_resolution_clock::now();
  	ceres::Solver::Options options;
  	options.linear_solver_type = ceres::DENSE_QR;
  	options.minimizer_progress_to_stdout = true;
  	ceres::Solver::Summary summary;
  	ceres::Solve(options, &problem, &summary);
    auto solve_end = high_resolution_clock::now();
    auto solve_duration = duration_cast<milliseconds>(solve_end - solve_start);
    cout << "优化求解用时: " << solve_duration.count() << " 毫秒" << endl;

    auto total_end = high_resolution_clock::now();
    auto total_duration = duration_cast<milliseconds>(total_end - total_start);
    cout << "总执行用时: " << total_duration.count() << " 毫秒" << endl;

  	cout<<"a= "<<abc[0]<<endl;
  	cout<<"b= "<<abc[1]<<endl;
  	cout<<"c= "<<abc[2]<<endl;
	return 0;
}