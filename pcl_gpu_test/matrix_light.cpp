#include <iostream>
#include <random>

using namespace std;

// 生成随机数
double GenerateRandomRealValue()
{
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(1, 10);
    return distr(eng);
}


void CalcMatrixDotForLoop(const vector<vector<double>>& a, const vector<vector<double>>& b)
{
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    if (a[0].size() != b.size()) {
        cout << "error:" << a.size() << "," << b[0].size() << endl;
        return;
    }

    vector<vector<double>> c;
    vector<double> c_row(b[0].size());
    for (int i = 0; i < a.size(); ++i) {
        for (int j = 0; j < b[0].size(); ++j) {
            for (int k = 0; k < b.size(); ++k) {
                c_row[j] += a[i][k] * b[k][j];
            }
        }
        c.emplace_back(c_row);
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span = t2 - t1;
    std::cout << "Loop takes " << time_span.count() << " ms\n";

    // cout << "matrix c:\n";
    // for (int i = 0; i < c.size(); ++i) {
    //     for (int j = 0; j < c[0].size(); ++j) {
    //         cout << c[i][j] << ",";
    //     }
    //     cout << endl;
    // }
}


int main()
{
		// 3d矩阵
    double a[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0;  j < 3; ++j) {
            a[i][j] = GenerateRandomRealValue();
        }
    }

    return 0;
}
