#include <iostream>
#include <random>
#include <chrono>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>  // 添加这个头文件

using namespace std;

// CUDA kernel for matrix multiplication
__global__ void matrixMul(const double* a, const double* b, double* c, int n) {
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (row < n && col < n) {
        double sum = 0.0;
        for (int k = 0; k < n; k++) {
            sum += a[row * n + k] * b[k * n + col];
        }
        c[row * n + col] = sum;
    }
}

// 生成随机数
double GenerateRandomRealValue() {
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(1, 10);
    return distr(eng);
}

int main() {
    const int n = 3;  // 矩阵大小
    const int size = n * n * sizeof(double);

    // 主机内存分配
    double *a = new double[n * n];
    double *b = new double[n * n];
    double *c = new double[n * n];

    // 初始化矩阵
    for (int i = 0; i < n * n; i++) {
        a[i] = GenerateRandomRealValue();
        b[i] = GenerateRandomRealValue();
    }

    // GPU设备内存
    double *da, *db, *dc;
    
    auto t_begin = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point t1, t2;
    std::chrono::duration<double, std::milli> time_span;

    t1 = std::chrono::high_resolution_clock::now();
    cudaMalloc((void**)&da, size);
    cudaMalloc((void**)&db, size);
    cudaMalloc((void**)&dc, size);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "GPU malloc takes " << time_span.count() << " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    cudaMemcpy(da, a, size, cudaMemcpyHostToDevice);
    cudaMemcpy(db, b, size, cudaMemcpyHostToDevice);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "cudaMemcpy takes " << time_span.count() << " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    dim3 threadsPerBlock(16, 16);
    dim3 numBlocks((n + threadsPerBlock.x - 1) / threadsPerBlock.x,
                   (n + threadsPerBlock.y - 1) / threadsPerBlock.y);
    matrixMul<<<numBlocks, threadsPerBlock>>>(da, db, dc, n);
    cudaDeviceSynchronize();
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "gpu takes " << time_span.count() << " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    cudaMemcpy(c, dc, size, cudaMemcpyDeviceToHost);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "cudaMemcpy back takes " << time_span.count() << " ms\n";

    // 清理内存
    cudaFree(da);
    cudaFree(db);
    cudaFree(dc);
    delete[] a;
    delete[] b;
    delete[] c;

    auto t_end = std::chrono::high_resolution_clock::now();
    time_span = t_end - t_begin;
    std::cout << "GPU total takes " << time_span.count() << " ms\n";

    return 0;
}
