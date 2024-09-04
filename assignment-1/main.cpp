#include "iostream"

#include <Eigen/Dense>

double deg_to_rad(double degrees)
{
    return degrees * M_PI / 180;
}


// Eigen::Matrix3d rotate_x(double degrees)
// {
//  Eigen::Matrix3d matrix;
//  matrix <<
//       1.0, 0.0, 0.0,
//   0.0, std::cos(degrees), -std::sin(degrees),
//   0.0, std::sin(degrees), std::cos(degrees);
// matrix;
//}

// void example(double constant)
//{
//   Eigen::Matrix3d identity;
//   identity <<
//       1.0, 0.0, 0.0,
//       0.0, 1.0, 0.0,
//       0.0, 0.0, 1.0;
//   std::cout << "I: " << std::endl << identity << std::endl << std::endl;
//   std::cout << constant <<"*I: " << std::endl << constant * identity << std::endl << std::endl;
//}


    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d skew_symmetric;
        skew_symmetric <<
            0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
        return skew_symmetric;
    }

    void skew_symmetric_test()
    {
        Eigen::Matrix3d skew_matrix = skew_symmetric(Eigen::Vector3d{0.5, 0.5, 0.707107});
        std::cout << "Skew-symmetric matrix: " << std::endl;
        std::cout << skew_matrix << std::endl;
        std::cout << "Skew-symmetric matrix transposition: " << std::endl;
        std::cout <<-skew_matrix.transpose() << std::endl;
    }


Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
    {
        Eigen::Matrix3d matrix;
        matrix.col(0) = x;
        matrix.col(1) = y;
        matrix.col(2) = z;
        return matrix;
    }

Eigen::Matrix3d rotate_x(double degrees)
    {
        Eigen::Matrix3d matrix;
        matrix <<
            1, 0, 0,
        0, std::cos(degrees), -std::sin(degrees),
        0, std::sin(degrees), std::cos(degrees);
        return matrix;
    }

Eigen::Matrix3d rotate_y(double degrees)
    {
        Eigen::Matrix3d matrix;
        matrix <<
            std::cos(degrees), 0, std::sin(degrees),
        0, 1, 0,
        -std::sin(degrees), 0, std::cos(degrees);
        return matrix;
    }

Eigen::Matrix3d rotate_z(double degrees)
    {
        Eigen::Matrix3d matrix;
        matrix <<
            std::cos(degrees), -std::sin(degrees), 0,
        std::sin(degrees), std::cos(degrees), 0,
        0, 0, 1;
        return matrix;
    }

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double degrees)
    {
        Eigen::Matrix3d matrix;
        double rad = deg_to_rad(degrees);
        //std::cout << std::sin(rad) * axis(0) + (1 - std::cos(rad)) * (axis*axis) << std::endl;
        matrix <<
        //*matrix.setIdentity()
        Eigen::Matrix3d::Identity()+ std::sin(rad) * skew_symmetric(axis) + (1 - std::cos(rad)) * skew_symmetric(axis)*skew_symmetric(axis);
            return matrix;
    }
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
    {
    double rad1 = deg_to_rad(e(0));
    double rad2 = deg_to_rad(e(1));
    double rad3 = deg_to_rad(e(2));

    Eigen::Matrix3d matrix;
        matrix <<
            rotate_z(rad1)*rotate_y(rad2)*rotate_x(rad3);
        return matrix;
        }


void rotation_matrix_test()
    {
        Eigen::Matrix3d rot = rotation_matrix_from_euler_zyx(Eigen::Vector3d{45.0,-45.0, 90.0});
        Eigen::Matrix3d rot_aa = rotation_matrix_from_axis_angle(Eigen::Vector3d{0.8164966, 0.0, 0.5773503}, 120.0);
        Eigen::Matrix3d rot_fa = rotation_matrix_from_frame_axes(Eigen::Vector3d{0.5, 0.5, 0.707107},
        Eigen::Vector3d{-0.5,-0.5, 0.707107},
        Eigen::Vector3d{0.707107,-0.707107, 0.0});
        std::cout << "Rotation matrix from Euler: " << std::endl;
        std::cout << rot << std::endl << std::endl;
        std::cout << "Rotation matrix from axis-angle pair: " << std::endl;
        std::cout << rot_aa << std::endl << std::endl;
        std::cout << "Rotation matrix from frame axes: " << std::endl;
        std::cout << rot_fa << std::endl << std::endl;
    }

Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::Matrix4d matrix;
    matrix <<
        r(0,0),r(0,1), r(0,2), p(0),
    r(1,0), r(1,1), r(1,2), p(1),
    r(2,0), r(2,1), r(2,2), p(2),
    0,0,0,1;
        return matrix;
}

void transformation_matrix_test()
{
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(Eigen::Vector3d{45,-45.0, 90.0});
    Eigen::Vector3d v{1.0,-2.0, 3.0};
    std::cout << "transformation_matrix: " << std::endl;
    std::cout << transformation_matrix(r, v) << std::endl;
}

void transform_vector()
{
    Eigen::Vector3d v = Eigen::Vector3d{2.5, 3.0, -10};
    Eigen::Vector3d euler = Eigen::Vector3d{60, 45, 0};
    Eigen::Vector3d p = Eigen::Vector3d{0,0,10};
    Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(euler);
    Eigen::Matrix4d t = transformation_matrix(r, p);
    Eigen::Vector4d v2 = Eigen::Vector4d{v[0], v[1], v[2], 1};
    Eigen::Vector4d vw1 = t * v2;
    Eigen::Vector3d vw2 = Eigen::Vector3d{vw1[0], vw1[1], vw1[2]};
    std::cout << "Transformed vector in frame {w}: \n" << vw2 << std::endl;
}

int main()
{
    skew_symmetric_test();
    rotation_matrix_test();
    transformation_matrix_test();
    transform_vector();
    return 0;
}
