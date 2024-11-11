//
// Created by EIer on 05.09.2024.
//

#ifndef MATH_H
#define MATH_H

#endif //MATH_H
namespace math
{
    double deg_to_rad(double degrees);
    double rad_to_deg(double radians);

    double const_deg_to_rad();

    double const_rad_to_deg();

    bool floatEquals(double a, double b);

    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v);

    Eigen::Matrix3d rotate_x(double radians);

    Eigen::Matrix3d rotate_y(double radians);

    Eigen::Matrix3d rotate_z(double radians);

    Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e);

    Eigen::Matrix3d rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e);

    Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r);

    Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p);

    Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h);

    Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta); // Uses Rad

    Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta);

    Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf);

    void print_pose(const std::string &label, const Eigen::Matrix4d &tf);
    
}