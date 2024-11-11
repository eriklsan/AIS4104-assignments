#include <ranges>

#include "iostream"

#include <Eigen/Dense>

#include <math/math.h>

#include <utility>

#include "../../../Program Files/JetBrains/CLion 2024.2.0.1/bin/mingw/x86_64-w64-mingw32/include/math.h"


// Task 1 a)
Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
{
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;

    if (math::floatEquals(r( 2, 0), -1))
    {
        b = EIGEN_PI / 2.0;
        a = 0.0;
        c = std::atan2(r(0, 1), r(1, 1));
    }
    else if (math::floatEquals(r( 2, 0), 1))
    {
        b = -EIGEN_PI / 2.0;
        a = 0.0;
        c = -std::atan2(r(0, 1), r(1, 1));
    }
    else
    {
        b = std::atan2(-r(2,0), std::sqrt(r(0,0)*r(0,0) + r(1,0) * r(1,0)));
        a = std::atan2(r(1,0), r(0,0));
        c = std::atan2(r(2,1), r(2, 2));
    }
    return Eigen::Vector3d(a, b, c);
}
// Task 1 b)
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd resV(6);
    resV(0) = w(0);
    resV(1) = w(1);
    resV(2) = w(2);
    resV(3) = v(0);
    resV(4) = v(1);
    resV(5) = v(2);
    return resV;
}

// Task 1 c
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    Eigen::VectorXd screw_axis(6);
    screw_axis <<
        s, - s.cross(q) + h * s;
    return screw_axis;
}

// Task 1 d)
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

// Task 1 e)
double cot(double x)
{
    return std::cos(x)/std::sin(x);
}

// Task 2 a)
void changeWrench()
{
    Eigen::Vector3d fw;
        fw <<
            -30, 0, 0;
    Eigen::Vector3d ms;
        ms <<
            0, 0, 2;
    Eigen::Vector3d ews = Eigen::Vector3d(math::deg_to_rad(60), math::deg_to_rad(-60), math::deg_to_rad(0));

    Eigen::Matrix3d Rws = math::rotation_matrix_from_euler_yzx(ews);
    // std::cout << "Rws:\n" << Rws << std::endl;

    Eigen::Matrix3d Rsw = Rws.transpose();
    // std::cout << "Rsw (transpose):\n" << Rsw << std::endl;

    Eigen::Vector3d fs = Rsw * fw;

    Eigen::Vector3d mw = Rws * ms;

    std::cout << "fw: " << fw.transpose() << std::endl;
    std::cout << "mw: " << mw.transpose() << std::endl;
    std::cout << "fs: " << fs.transpose() << std::endl;
    std::cout << "ms: " << ms.transpose() << std::endl;
}

// Task 2 b)
void calcForce()
{
    // Creating a wrench
    Eigen::VectorXd apple = twist({0,0,0}, {0,0,1});
    Eigen::VectorXd rHand = twist({0,0,0},{0,-5,0});

    // Initializing for the first transformation matrix
    Eigen::Matrix3d mhf;
    mhf << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;

    Eigen::Matrix4d hf = math::transformation_matrix(mhf, {-0.1,0,0});

    // Initializing for the second transformation matrix
    Eigen::Matrix3d maf;
    maf << 1, 0, 0,
    0,0,1,
    0,-1,0;

    Eigen::Matrix4d af = math::transformation_matrix(maf, {-0.25,0,0});

    Eigen::VectorXd forceRes = adjoint_matrix(hf).transpose() * rHand + adjoint_matrix(af).transpose() * apple;

    std::cout << "forceRes: " << forceRes.transpose() << std::endl;
}

// Task 3 a)
Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta) // Uses Rad
{
    const Eigen::Matrix3d skew_w = math::skew_symmetric(w);
    Eigen::Matrix3d matrix;
    matrix <<
        Eigen::Matrix3d::Identity() + std::sin(theta) * skew_w + (1.0 - std::cos(theta)) * skew_w * skew_w;
    return matrix; // Use variable for temporary storage, and use double literal
}


// Task 3 b)
bool isIdentity(Eigen::MatrixXd r, const int N) // isApprox could be used
{
    if (r.rows() != N || r.cols() != N)
        return false; // Ensure matrix is N x N

    for (int row = 0; row < N; row++)
    {
        for (int col = 0; col < N; col++)
        {
            if (row == col && not math::floatEquals(r(row, col),1))
                return false;
            else if (row != col && not math::floatEquals(r(row, col),0))
                return false;
        }
    }
    return true;
}


std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r) // Initial theta and w outside the if function
{
    double theta;
    Eigen::Vector3d w;
    theta = 0.0;
    w = Eigen::Vector3d(0, 0, 0);
    if (isIdentity(r, 3))
    {
        std::cout << "w is not defined " << std::endl;
    }
    else if (r.trace() == -1) // Check if any values makes the denominator 0
    {
        theta = 180 * math::const_deg_to_rad();
        if (not (math::floatEquals(r(0,0), -1)))
        {
            w = Eigen::Vector3d( r(0, 0)+1, r(1, 0), r(2, 0));
            w /= std::sqrt(2.0 * (1.0 + r(0, 0)));
        }

        else if (not (math::floatEquals(r(1,1), -1)))
        {
            w = Eigen::Vector3d(r(0, 1), r(1, 1) + 1.0, r(2, 1));
            w /= std::sqrt(2.0 * (1.0 + r(1, 1)));
        }

        else
        {
            w = Eigen::Vector3d(r(0, 2), r(1, 2), r(2, 2)+1.0);
            w /= std::sqrt(2 * (1 + r(2, 2)));
        }
    }
    else
    {
        theta = std::acos(0.5*(r.trace() - 1));
        Eigen::Matrix3d skew_w = (r - r.transpose()) / (2.0 * std::sin(theta));
        w = Eigen::Vector3d(skew_w(2, 1), skew_w(0, 2), skew_w(1, 0));
    }

    return std::make_pair(w, theta);
}

// Task 3 c)

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

// Task 3 d)

std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &t)
{
    Eigen::Matrix3d R = t.block<3,3>(0,0);
    Eigen::Vector3d p = {t(3,0), t(3,1), t(3,2)};


    if (isIdentity(t,4))
    {
        double theta = p.norm();
        Eigen::Vector3d vf = p/p.norm();

        return std::make_pair(twist({0,0,0}, vf),theta );
    }
    else
    {
        //Eigen::Vector3d w = {0,0,-t[0,1]};
        // Eigen::Vector3d v = {t[0,3], t[1,3], 0};
        std::pair<Eigen::Vector3d, double> m = matrix_logarithm(R);
        Eigen::Vector3d w = m.first;
        double theta = m.second;
        Eigen::Matrix3d w_sk = math::skew_symmetric(w);
        Eigen::Matrix3d G_1 = Eigen::Matrix3d::Identity()/theta - w_sk / 2.0 + (1.0 / theta - cot(theta / 2.0) / 2.0) * w_sk * w_sk;
        Eigen::Vector3d v;
        v = G_1 * p;
        return std::make_pair(twist(w, v), theta);
    }
}

// Task 4 a)

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

// Task 4 b)
Eigen::Matrix4d planar_3r_fk_transform(const std::vector<double> &joint_positions)
{
    double L1 = 10.0, L2 = 10.0, L3 = 10.0;
    Eigen::Matrix4d t01;
    t01.block(0,0,3,3) = math::rotate_z(joint_positions[0] * math::const_deg_to_rad());
    t01.block(0,3,3,1) = Eigen::Vector3d(0.0,0.0,0.0);
    t01.block(3,0,1,3) = Eigen::Vector3d::Zero().transpose();
    t01(3,3) = 1.0;
    Eigen::Matrix4d t12;
    t12.block(0,0,3,3) = math::rotate_z(joint_positions[1] * math::const_deg_to_rad());
    t12.block(0,3,3,1) = Eigen::Vector3d(L1, 0.0, 0.0);
    t12.block(3,0,1,3) = Eigen::Vector3d::Zero().transpose();
    t12(3,3) = 1.0;
    Eigen::Matrix4d t23;
    t23.block(0,0,3,3) = math::rotate_z(joint_positions[2] * math::const_deg_to_rad());
    t23.block(0,3,3,1) = Eigen::Vector3d(L2, 0.0, 0.0);
    t23.block(3,0,1,3) = Eigen::Vector3d::Zero().transpose();
    t23(3,3) = 1.0;
    Eigen::Matrix4d t34 = Eigen::Matrix4d::Identity();
    t34.block(0,3,3,1) = Eigen::Vector3d(L3,0.0,0.0);
    return t01 * t12 * t23 * t34;
}

//Task 4 c)

Eigen::Matrix4d planar_3r_fk_screw(const std::vector<double> &joint_positions)
{
    double L1 = 10.0, L2 = 10.0, L3 = 10.0;
    Eigen::Vector3d w1 = {0.0, 0.0, 1.0};
    Eigen::Vector3d v1 = {0.0, 0.0, 0.0};
    Eigen::Matrix4d e1 = matrix_exponential(w1, v1, joint_positions[0] * math::const_deg_to_rad());
    Eigen::Vector3d w2 = {0.0, 0.0, 1.0};
    Eigen::Vector3d v2 = {0.0, -L1, 0.0};
    Eigen::Matrix4d e2 = matrix_exponential(w2, v2, joint_positions[1]* math::const_deg_to_rad());
    Eigen::Vector3d w3 = {0.0, 0.0, 1.0};
    Eigen::Vector3d v3 = {0.0, -(L1+L2), 0.0};
    Eigen::Matrix4d e3 = matrix_exponential(w3, v3, joint_positions[2] * math::const_deg_to_rad());
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m.block(0, 3, 3, 1) = Eigen::Vector3d(L1+L2+L3, 0.0, 0.0);
    return e1 * e2 * e3 * m;
}


//Task 5 a)

Eigen::Matrix4d make_matrix_for_function()
{
    double L1 = 0.2436, L2 = 0.2132, W1 = 0.1311, W2 = 0.0921, H1 = 0.1519, H2 = 0.0854;
    Eigen::Matrix3d mR1 = math::rotate_z(M_PI);
    Eigen::Matrix3d mR2 = math::rotate_x(M_PI/2);

    //std::cout << mR1 *mR2 << std::endl;
    return math::transformation_matrix(mR1 * mR2, { L1 + L2,W1 +W2, H1 - H2});
}

Eigen::Matrix4d ur3e_fk_screw(const std::vector<double> &joint_positions)
{
    double L1 = 0.2436, L2 = 0.2132, W1 = 0.1311, W2 = 0.0921, H1 = 0.1519, H2 = 0.0854;

    Eigen::Vector3d w1 = {0,0,1};
    Eigen::Vector3d q1 = {0.0, 0.0, 0.0}; // Can choose a suitable value for p, how does this effect further calculations?
    Eigen::VectorXd screw = screw_axis(q1, w1, 0);

    Eigen::Matrix4d e1 = matrix_exponential(w1, screw.tail(3), joint_positions[0] * math::const_deg_to_rad());

    Eigen::Vector3d w2 = {0.0, 1.0, 0.0};
    Eigen::Vector3d q2 = {0.0, 0.0, H1};
    screw = screw_axis(q2, w2, 0);

    Eigen::Matrix4d e2 = matrix_exponential( w2, screw.tail(3),joint_positions[1] * math::const_deg_to_rad());

    Eigen::Vector3d w3 = {0, 1, 0};
    Eigen::Vector3d q3 = {L1, 0.0, H1};
    screw = screw_axis(q3, w3, 0);

    Eigen::Matrix4d e3 = matrix_exponential(w3, screw.tail(3), joint_positions[2] * math::const_deg_to_rad());

    Eigen::Vector3d w4 = {0.0, 1.0, 0.0};
    Eigen::Vector3d q4 = {L1 + L2, 0.0,H1};
    screw = screw_axis(q4, w4, 0);

    Eigen::Matrix4d e4 = matrix_exponential(w4, screw.tail(3), joint_positions[3] * math::const_deg_to_rad());

    Eigen::Vector3d w5 = {0.0, 0, -1.0};
    Eigen::Vector3d q5 = {L1 + L2, W1, 0.0};
    screw = screw_axis(q5, w5, 0);

    Eigen::Matrix4d e5 = matrix_exponential(w5, screw.tail(3), joint_positions[4] * math::const_deg_to_rad());

    Eigen::Vector3d w6 = {0.0, 1.0, 0.0};
    Eigen::Vector3d q6 = {L1 + L2, 0.0,  H1 - H2};
    screw = screw_axis(q6, w6, 0);

    Eigen::Matrix4d e6 = matrix_exponential(w6, screw.tail(3), joint_positions[5] * math::const_deg_to_rad());

    print_pose("J", e1 * e2 * e3 * e4 * e5 * e6 * make_matrix_for_function());

    return e1 * e2 * e3 * e4 * e5 * e6 * make_matrix_for_function();
}

//Task 5 b)
Eigen::Matrix4d ur3e_fk_transform(const std::vector<double> &joint_positions)
{
    double L1 = 0.2436, L2 = 0.2132, W1 = 0.1311, W2 = 0.0921, H1 = 0.1519, H2 = 0.0854;

    // Transformation from base to joint 1 (Z-axis rotation)
    Eigen::Matrix4d t01 = Eigen::Matrix4d::Identity();
    t01.block<3,3>(0,0) = math::rotate_z(joint_positions[0] * math::const_deg_to_rad());
    t01.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0,0.0);

    // Transformation from joint 1 to joint 2 (Negative Y-axis rotation to correct orientation)
    Eigen::Matrix4d t12 = Eigen::Matrix4d::Identity();
    t12.block<3,3>(0,0) = math::rotate_y(joint_positions[1] * math::const_deg_to_rad());
    t12.block<3,1>(0,3) = Eigen::Vector3d(0.0, W1, H1);

    // Transformation from joint 2 to joint 3 (Negative Y-axis rotation)
    Eigen::Matrix4d t23 = Eigen::Matrix4d::Identity();
    t23.block<3,3>(0,0) = math::rotate_y(joint_positions[2] * math::const_deg_to_rad());
    t23.block<3,1>(0,3) = Eigen::Vector3d(L1, 0.0, 0.0);

    // Transformation from joint 3 to joint 4 (Positive Y-axis rotation, correction for joint 4)
    Eigen::Matrix4d t34 = Eigen::Matrix4d::Identity();
    t34.block<3,3>(0,0) = math::rotate_y(joint_positions[3] * math::const_deg_to_rad());
    t34.block<3,1>(0,3) = Eigen::Vector3d(L2, 0.0, 0.0);

    // Transformation from joint 4 to joint 5 (Z-axis rotation for joint 5)
    Eigen::Matrix4d t45 = Eigen::Matrix4d::Identity();
    t45.block<3,3>(0,0) = math::rotate_z(-joint_positions[4] * math::const_deg_to_rad());
    t45.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, -H2);

    // Transformation from joint 5 to joint 6 (Negative Y-axis rotation for joint 6)
    Eigen::Matrix4d t56 = Eigen::Matrix4d::Identity();
    t56.block<3,3>(0,0) = math::rotate_y(joint_positions[5] * math::const_deg_to_rad());
    t56.block<3,1>(0,3) = Eigen::Vector3d(0.0, W2, 0.0);

    // Transformation from joint 6 to the end-effector (fixed X-axis rotation for the tool)
    Eigen::Matrix4d t67 = Eigen::Matrix4d::Identity();
    t67.block<3,3>(0,0) = math::rotate_x(-90*math::const_deg_to_rad()) * math::rotate_z(180*math::const_deg_to_rad());
    t67.block<3,1>(0,3) = Eigen::Vector3d(0.0, 0.0, 0.0);

    // Compute final transformation matrix
    Eigen::Matrix4d final_transform = t01 * t12 * t23 * t34 * t45 * t56 * t67;

    // Print the pose
    print_pose("J", final_transform);

    // Return the final transformation matrix
    return final_transform;
}


// -------------------------------------------------------------------------------------------------------------------
// Test functions
void matrix_exponential_test()
{
    Eigen::Matrix3d test = matrix_exponential({0, 0.866, 0.5}, 30);
    std::cout << "This is the output: \n" << test << std::endl;
}


// Test function
void isIdentity_test()
{
    Eigen::Matrix3d test;
    test <<
        1, 0, 0,
        0,1,0,
        0,0,1;

    const bool testRes = isIdentity(test, 3);
    std::cout << "This is the output: \n" << testRes << std::endl;
}


// Test function
void test_matrix_logarithm() {
    // Test 1: Identity matrix (should return w = [0, 0, 0], theta = 0)
    Eigen::Matrix3d identity_matrix = Eigen::Matrix3d::Identity();
    auto result1 = matrix_logarithm(identity_matrix);
    std::cout << "Test 1 - Identity Matrix:\n";
    std::cout << "w: " << result1.first.transpose() << ", theta: " << result1.second << "\n\n";

    // Test 2: 90-degree rotation around z-axis (should return w = [0, 0, 1], theta = M_PI/2)
    Eigen::Matrix3d rot_z_90;
    rot_z_90 = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
    auto result2 = matrix_logarithm(rot_z_90);
    std::cout << "Test 2 - 90-degree Rotation around Z-axis:\n";
    std::cout << "w: " << result2.first.transpose() << ", theta: " << result2.second << "\n\n";

    // Test 3: 180-degree rotation around y-axis (should return w = [0, 1, 0], theta = M_PI)
    Eigen::Matrix3d rot_y_180;
    rot_y_180 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    auto result3 = matrix_logarithm(rot_y_180);
    std::cout << "Test 3 - 180-degree Rotation around Y-axis:\n";
    std::cout << "w: " << result3.first.transpose() << ", theta: " << result3.second << "\n\n";

    // Test 4: 45-degree rotation around x-axis (should return w = [1, 0, 0], theta = M_PI/4)
    Eigen::Matrix3d rot_x_45;
    rot_x_45 = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX());
    auto result4 = matrix_logarithm(rot_x_45);
    std::cout << "Test 4 - 45-degree Rotation around X-axis:\n";
    std::cout << "w: " << result4.first.transpose() << ", theta: " << result4.second << "\n\n";
}



int main()
{

    // changeWrench();
    // calcForce();
    // isIdentity_test();
    // matrix_exponential_test();
    // test_matrix_logarithm();
    // planar_3r_fk_transform({10,-15, 2.75});
    // planar_3r_fk_screw({10,-15, 2.75});
    //std::cout << planar_3r_fk_transform({0, 0, 90}) << std::endl<< std::endl;
    //std::cout << planar_3r_fk_screw({0, 0, 90}) << std::endl;

    ur3e_fk_screw({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // std::cout << ur3e_fk_screw({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) << std::endl;
    ur3e_fk_transform({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    ur3e_fk_screw({0.0, 0.0, 0.0, -90.0, 0.0, 0.0});
    ur3e_fk_transform({0.0, 0.0, 0.0, -90.0, 0.0, 0.0});

    ur3e_fk_screw({0.0, -90.0, 90.0, 0.0, 90.0, 0.0});
    ur3e_fk_transform({0.0, -90.0, 90.0, 0.0, 90.0, 0.0});

    ur3e_fk_screw({0.0, 0.0, -90.0, 0.0, 0.0, 0.0});
    ur3e_fk_transform({0.0, 0.0, -90.0, 0.0, 0.0, 0.0});


    /*
    Eigen::Matrix3d matrix = math::rotate_x(90*math::const_deg_to_rad());
    std::cout << "Matrix Rotation X: " << matrix << std::endl;
    Eigen::Matrix3d matrixy = math::rotate_y(90*math::const_deg_to_rad());
    std::cout << "Matrix Rotation Y: " << matrixy << std::endl;
    Eigen::Matrix3d matrixz = math::rotate_z(90*math::const_deg_to_rad());
    std::cout << "Matrix Rotation Z: " << matrixz << std::endl;
    */

    //make_matrix_for_function();
    //std::cout << make_matrix_for_function() << std::endl;
}
