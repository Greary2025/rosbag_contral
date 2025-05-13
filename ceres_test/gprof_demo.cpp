#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

// 计算密集型函数1：计算斐波那契数列
long long fibonacci(int n) {
    if (n <= 1) return n;
    return fibonacci(n - 1) + fibonacci(n - 2);
}

// 计算密集型函数2：矩阵乘法
void matrix_multiply(const std::vector<std::vector<int>>& a, 
                    const std::vector<std::vector<int>>& b, 
                    std::vector<std::vector<int>>& result) {
    int n = a.size();
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            result[i][j] = 0;
            for (int k = 0; k < n; ++k) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

// 计算密集型函数3：素数检测
bool is_prime(int n) {
    if (n <= 1) return false;
    for (int i = 2; i <= sqrt(n); ++i) {
        if (n % i == 0) return false;
    }
    return true;
}

// 模拟延迟的函数
void simulate_delay() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main() {
    // 测试斐波那契数列计算
    std::cout << "计算斐波那契数列第40项..." << std::endl;
    fibonacci(40);

    // 测试矩阵乘法
    int matrix_size = 100;
    std::vector<std::vector<int>> matrix_a(matrix_size, std::vector<int>(matrix_size, 1));
    std::vector<std::vector<int>> matrix_b(matrix_size, std::vector<int>(matrix_size, 1));
    std::vector<std::vector<int>> matrix_result(matrix_size, std::vector<int>(matrix_size));
    
    std::cout << "执行矩阵乘法..." << std::endl;
    matrix_multiply(matrix_a, matrix_b, matrix_result);

    // 测试素数检测
    std::cout << "执行素数检测..." << std::endl;
    for (int i = 2; i < 10000; ++i) {
        is_prime(i);
    }

    // 模拟I/O或网络延迟
    std::cout << "模拟系统调用延迟..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        simulate_delay();
    }

    return 0;
}