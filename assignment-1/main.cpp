#include "iostream"

#include <Eigen/Dense>

Eigen::Matrix3d rotate_x(double degrees)
{
    Eigen::Matrix3d matrix;
    matrix <<
        1.0, 0.0, 0.0,
    0.0, std::cos(degrees), -std::sin(degrees),
    0.0, std::sin(degrees), std::cos(degrees);
    return matrix;
}

void example(double constant)
{
    Eigen::Matrix3d identity;
    identity <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    std::cout << "I: " << std::endl << identity << std::endl << std::endl;
    std::cout << constant <<"*I: " << std::endl << constant * identity << std::endl << std::endl;
}

int main()
{
    example(2.0);
    return 0;
}
