#include <iostream>
#include <vector>
#include <random>
#include <algorithm>

// 递归快速排序函数
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

// 内存密集型操作：大矩阵操作
void matrix_operations() {
    const int size = 1000;
    std::vector<std::vector<double>> matrix(size, std::vector<double>(size));
    
    // 填充矩阵
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 100);
    
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            matrix[i][j] = dis(gen);
        }
    }
    
    // 执行转置操作
    auto transposed = matrix_transpose(matrix);
    
    // 计算对角线元素之和
    double sum = 0;
    for (int i = 0; i < size; ++i) {
        sum += matrix[i][i];
    }
}

int main() {
    // 测试快速排序
    std::cout << "执行快速排序..." << std::endl;
    std::vector<int> arr(10000);
    for (int i = 0; i < 10000; ++i) {
        arr[i] = rand() % 10000;
    }
    quicksort(arr, 0, arr.size() - 1);
    
    // 测试矩阵操作
    std::cout << "执行矩阵操作..." << std::endl;
    matrix_operations();
    
    return 0;
}