//
// Created by EIer on 05.09.2024.
//
#include <iostream>

#include <Eigen/Dense>

#include "math.h"
#include "../../../Program Files/JetBrains/CLion 2024.2.0.1/bin/mingw/x86_64-w64-mingw32/include/math.h"

namespace math
{
    double deg_to_rad(double degrees)
    {
        return degrees * M_PI / 180;
    }

    double rad_to_deg(double radians)
    {
        return radians * 180 / M_PI;
    }

    double const_deg_to_rad()
    {
        return 0.0174533;
    }

    double const_rad_to_deg()
    {
       return 57.29578;
    }

    bool floatEquals(double a, double b)
    {
        return std::abs(a - b) < 1e-6;
    }

    Eigen::Matrix3d rotate_x(double radians)
    {
        //Eigen::Matrix3d matrix<3,1>(0,0) = {1,0,0};
        Eigen::Matrix3d matrix;
        //matrix<1,2>
        matrix
            <<
            1.0, 0.0, 0.0,
            0.0, std::cos(radians), -std::sin(radians),
            0.0, std::sin(radians), std::cos(radians);
        return matrix;
    }

    Eigen::Matrix3d rotate_y(double radians)
    {
        Eigen::Matrix3d matrix;
        matrix
            <<
            std::cos(radians), 0.0, std::sin(radians),
            0.0, 1.0, 0.0,
            -std::sin(radians), 0.0, std::cos(radians);
        return matrix;
    }

    Eigen::Matrix3d rotate_z(double radians)
    {
        Eigen::Matrix3d matrix;
        matrix
            <<
            std::cos(radians), -std::sin(radians), 0.0,
            std::sin(radians), std::cos(radians), 0.0,
            0.0, 0.0, 1.0;
        return matrix;
    }

    Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
    {
        return rotate_z(e.x()) * rotate_y(e.y()) * rotate_x(e.z());
    }

    Eigen::Matrix3d rotation_matrix_from_euler_yzx(const Eigen::Vector3d &e)
    {
        return rotate_y(e.x()) * rotate_z(e.y()) * rotate_x(e.z());
    }


    Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r)
    {
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;

        if (floatEquals(r(2, 0), -1.0))
        {
            b = EIGEN_PI / 2.0;
            a = 0.0;
            c = std::atan2(r(0, 1), r(1, 1));
        }
        else if (floatEquals(r(2, 0), 1.0))
        {
            b = -(EIGEN_PI / 2.0);
            a = 0.0;
            c = -std::atan2(r(0, 1), r(1, 1));
        }
        else
        {
            b = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
            a = std::atan2(r(1, 0), r(0, 0));
            c = std::atan2(r(2, 1), r(2, 2));
        }
        return Eigen::Vector3d{a, b, c};


    }

    Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d skew_symmetric;
        skew_symmetric <<
            0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
        return skew_symmetric;
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

    Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
    {
        Eigen::VectorXd screw_axis(6);
        screw_axis <<
            s, - s.cross(q) + h * s;
        return screw_axis;
    }

    Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta) // Uses Rad
{
        const Eigen::Matrix3d skew_w = math::skew_symmetric(w);
        Eigen::Matrix3d matrix;
        matrix <<
            Eigen::Matrix3d::Identity() + std::sin(theta) * skew_w + (1.0 - std::cos(theta)) * skew_w * skew_w;
        return matrix; // Use variable for temporary storage, and use double literal
}

    Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
    {
        Eigen::Matrix4d matrix;
        matrix = Eigen::Matrix4d::Zero();
        // double rad = math::deg_to_rad(theta);

        if (math::floatEquals(w.norm(),1 )) //floatEquals
        {
            Eigen::Matrix3d ewt = matrix_exponential(w, theta);
            Eigen::Matrix3d G = Eigen::Matrix3d::Identity() * theta + (1.0 - std::cos(theta)) * math::skew_symmetric(w) + (theta - std::sin(theta)) * math::skew_symmetric(w) * math::skew_symmetric(w);
            matrix.block<3,3>(0,0) = ewt;
            matrix.block(0,3, 3,1) = G * v;
            matrix(3,3) = 1;
        }
        else if (math::floatEquals(v.norm(),1 ) && math::floatEquals(w.norm(), 0))
        {
            matrix.block(0,0,3,3) = Eigen::Matrix3d::Identity();
            matrix.block(0,3,3,1) = v * theta;
            matrix(3,3) = 1;
        }
        else
        {
            std::cout << "Invalid input" << std::endl;
            throw std::invalid_argument("Invalid input");
            return Eigen::Matrix4d::Zero();
        }
        return matrix;
    }


    Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
    {

        Eigen::Matrix3d r = tf.block<3,3>(0,0);
        Eigen::Vector3d p = tf.block<3,1>(0,3);
        Eigen::MatrixXd adjoint_matrix(6,6);
        adjoint_matrix.block<3,3>(0,0) = r;
        adjoint_matrix.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
        adjoint_matrix.block<3,3>(3,0) = math::skew_symmetric(p)*r;
        adjoint_matrix.block<3,3>(3,3) = r;
        return adjoint_matrix;
    }


    void print_pose(const std::string &label, const Eigen::Matrix4d &tf)
    {
        Eigen::Vector3d p = tf.block(0,3,3,1);
        const Eigen::Matrix3d r = tf.block(0,0,3,3);
        Eigen::Vector3d e = math::euler_zyx_from_rotation(r);
        std::cout << label << ":" << std::endl;
        std::cout << "Euler ZYX(rad): " << e.transpose() << std::endl;
        std::cout << "Euler ZYX(deg): " << e.transpose() * math::const_rad_to_deg() << std::endl;
        std::cout << "Position: " << p.transpose() << std::endl;
    }

}

// GO to math.h paste the function from assignment
// and then annotate the function in here with math::

// Eigen::VectorXd math::twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)