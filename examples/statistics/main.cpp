#include <iostream>
#include <Eigen/Core>
#include <Euclid/Math/Statistics.h>

int main(int argc, char* argv[])
{
    int rows = 5;
    int cols = 3;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--rows" && i + 1 < argc) {
            rows = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--cols" && i + 1 < argc) {
            cols = std::atoi(argv[++i]);
        }
    }

    Eigen::MatrixXf data(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            data(i, j) = static_cast<float>(i + j + 1);
        }
    }

    Eigen::Matrix3f cov;
    Euclid::covariance_matrix(data, cov);

    std::cout << "Covariance matrix:" << std::endl;
    std::cout << cov << std::endl;
}