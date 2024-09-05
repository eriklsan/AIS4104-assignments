# include <iostream>
# include <Eigen/Dense>



constexpr double rad_to_deg = 57.29578;
constexpr double deg_to_rad = 0.01745;

bool floatEquals(double a, double b)
    {
  return std::abs(a-b) < 1e-6;
  }
// USE RADIANS ISTEAD OF DEGREES

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

Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    Eigen::Matrix3d matrix;
    matrix <<
        rotate_z(e(0))*rotate_y(e(1))*rotate_x(e(2));
    return matrix;
}


Eigen::Vector3d euler_zyx_from_rotation(const Eigen::Matrix3d &r)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if(!floatEquals(r(2,0),-1))
    {
        b = EIGEN_PI/2;
        a = 0.0;
        c = std::atan2(r(0, 1) ,r(1,1));
    }
        else if (!floatEquals(r(2), r(1) ))
    {
      b = -(EIGEN_PI / 2.0);
      a = 0.0;
      c = -std::atan2(r(0, 1), r(1,1));
     }
    else
    {
    b = std::atan2(-r(2, 0), std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0)));
      a = std::atan2(r(1, 0), r(0, 0));
      c = std::atan2(r(2, 1), r(2, 2));
    }
  return Eigen::Vector3d{a, b, c};
}

int main()
      {
          Eigen::Vector3d e = Eigen::Vector3d{60.0, 45.0, 30.0} * deg_to_rad;
          Eigen::Matrix3d r = rotation_matrix_from_euler_zyx(e);
          Eigen::Vector3d ea = euler_zyx_from_rotation(r);
          std::cout << " E: " << e.transpose() * rad_to_deg << std::endl;
          std::cout << "Ea: " << ea.transpose() * rad_to_deg << std::endl;
          return 0;
      }