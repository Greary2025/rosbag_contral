#include <iostream>
#include <chrono>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

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

MatrixXd Generate2DMatrixByEigen() {
    MatrixXd m = MatrixXd::Random(1000, 1000);
    return m;
}

void ModeEigen(const MatrixXd& a, const MatrixXd& b) {
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto c = a * b;
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    std::cout << "Eigen CPU takes " << time_span.count() << " ms\n";
}

void ModeGPU(const MatrixXd& a, const MatrixXd& b) {
    const int n = a.rows();
    const int size = n * n * sizeof(double);

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
    cudaMemcpy(da, a.data(), size, cudaMemcpyHostToDevice);
    cudaMemcpy(db, b.data(), size, cudaMemcpyHostToDevice);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "cudaMemcpy takes " << time_span.count() << " ms\n";

    t1 = std::chrono::high_resolution_clock::now();
    dim3 threadsPerBlock(32, 32);  // 增加线程块大小以适应更大的矩阵
    dim3 numBlocks((n + threadsPerBlock.x - 1) / threadsPerBlock.x,
                   (n + threadsPerBlock.y - 1) / threadsPerBlock.y);
    matrixMul<<<numBlocks, threadsPerBlock>>>(da, db, dc, n);
    cudaDeviceSynchronize();
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "GPU computation takes " << time_span.count() << " ms\n";

    // 分配主机内存接收结果
    double* c = new double[n * n];
    t1 = std::chrono::high_resolution_clock::now();
    cudaMemcpy(c, dc, size, cudaMemcpyDeviceToHost);
    t2 = std::chrono::high_resolution_clock::now();
    time_span = t2 - t1;
    std::cout << "cudaMemcpy back takes " << time_span.count() << " ms\n";

    // 清理内存
    cudaFree(da);
    cudaFree(db);
    cudaFree(dc);
    delete[] c;

    auto t_end = std::chrono::high_resolution_clock::now();
    time_span = t_end - t_begin;
    std::cout << "GPU total takes " << time_span.count() << " ms\n";
}

int main() {
    std::cout << "Generating 1000x1000 random matrices...\n";
    
    // 创建两个1000x1000矩阵并填充随机数
    MatrixXd matrix_a = Generate2DMatrixByEigen();
    MatrixXd matrix_b = Generate2DMatrixByEigen();

    // CPU版本计算
    std::cout << "Starting CPU matrix multiplication...\n";
    ModeEigen(matrix_a, matrix_b);
    
    // GPU版本计算
    std::cout << "Starting GPU matrix multiplication...\n";
    ModeGPU(matrix_a, matrix_b);

    return 0;
}