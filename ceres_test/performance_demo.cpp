#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>

// 矩阵乘法函数
void matrix_multiply(const std::vector<std::vector<double>>& a,
                    const std::vector<std::vector<double>>& b,
                    std::vector<std::vector<double>>& result) {
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

// 快速排序函数
template<typename T>
void quicksort(std::vector<T>& arr, int left, int right) {
    if (left >= right) return;
    
    int pivot = left + (right - left) / 2;
    T pivot_value = arr[pivot];
    
    std::swap(arr[pivot], arr[right]);
    int store_index = left;
    
    for (int i = left; i < right; i++) {
        if (arr[i] < pivot_value) {
            std::swap(arr[store_index], arr[i]);
            store_index++;
        }
    }
    
    std::swap(arr[right], arr[store_index]);
    
    quicksort(arr, left, store_index - 1);
    quicksort(arr, store_index + 1, right);
}

// 矩阵转置函数
std::vector<std::vector<double>> matrix_transpose(
    const std::vector<std::vector<double>>& matrix) {
    int rows = matrix.size();
    int cols = matrix[0].size();
    std::vector<std::vector<double>> result(cols, std::vector<double>(rows));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j][i] = matrix[i][j];
        }
    }
    return result;
}

/**
 * @brief 通用计时器包装函数，用于测量任意函数或lambda表达式的执行时间
 * 
 * @tparam Func 模板参数，可以是任意可调用对象类型（函数、lambda表达式、函数对象等）
 * 
 * @param op_name 操作名称，用于在输出执行时间时标识当前操作
 * @param func 被测量的函数对象，使用完美转发确保不会发生不必要的拷贝
 * 
 * @return double 返回操作执行时间，单位为毫秒
 * 
 * @details 该函数使用C++11的高精度时钟（high_resolution_clock）进行时间测量
 *          通过RAII方式自动管理计时器的开始和结束
 *          支持任意可调用对象的执行时间测量
 *          使用chrono库进行精确的时间计算和转换
 */
template<typename Func>
double time_operation(const std::string& op_name, Func&& func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << op_name << "用时: " << std::fixed << std::setprecision(2)
              << duration.count() << " 毫秒" << std::endl;
    return duration.count();
}

int main() {
    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 100);

    // 1. 测试快速排序
    time_operation("快速排序", [&]() {
        std::vector<int> arr(100000);
        for (int i = 0; i < 100000; ++i) {
            arr[i] = dis(gen);
        }
        quicksort(arr, 0, arr.size() - 1);
    });

    // 2. 测试矩阵乘法
    time_operation("矩阵乘法", [&]() {
        const int size = 500;
        std::vector<std::vector<double>> matrix_a(size, std::vector<double>(size));
        std::vector<std::vector<double>> matrix_b(size, std::vector<double>(size));
        std::vector<std::vector<double>> result(size, std::vector<double>(size));

        // 初始化矩阵
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                matrix_a[i][j] = dis(gen);
                matrix_b[i][j] = dis(gen);
            }
        }

        matrix_multiply(matrix_a, matrix_b, result);
    });

    // 3. 测试矩阵转置
    time_operation("矩阵转置", [&]() {
        const int size = 1000;
        std::vector<std::vector<double>> matrix(size, std::vector<double>(size));

        // 初始化矩阵
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                matrix[i][j] = dis(gen);
            }
        }

        auto transposed = matrix_transpose(matrix);
    });

    return 0;
}