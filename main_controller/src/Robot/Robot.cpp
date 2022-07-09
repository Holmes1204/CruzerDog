#include <Robot/Robot.h>
Robot my_robot(23.0, 0.2, 0.2, 0.1805, 0.1308, Eigen::Matrix3d::Zero());
Robot::Robot(double mass_, double len_1, double len_2, double x_offset, double y_offset, Eigen::Matrix3<double> inerial)
{
    mass = mass_;
    L2 = len_1;
    L2 = len_2;
    // each hip frame origin in body frame
    for (int leg_ = 0; leg_ < 4; leg_++)
    {
        if (leg_ == 0 || leg_ == 1)
        {
            hip_location[leg_].x() = x_offset;
        }
        else
        {
            hip_location[leg_].x() = -x_offset;
        }

        if (leg_ == 0 || leg_ == 2)
        {
            hip_location[leg_].y() = y_offset;
        }
        else
        {
            hip_location[leg_].y() = -y_offset;
        }
        hip_location[leg_].z() = 0.;
    }
    inertial_tensor = inerial;
}
Robot::~Robot()
{
}

// forward_kinematic from  joint space to body frame
Eigen::Vector3<double> Robot::forward_kinematic(Eigen::Vector3<double> q_, int leg_)
{
    Eigen::Vector3<double> p_;
    double s1 = std::sin(q_(1));
    double c1 = std::cos(q_(1));
    double s12 = std::sin(q_(1) + q_(2));
    double c12 = std::cos(q_(1) + q_(2));
    p_.x() = this->hip_location[leg_].x() - this->L1 * s1 - this->L2 * s12;
    p_.y() = this->hip_location[leg_].y();
    p_.z() = this->hip_location[leg_].z() - this->L1 * c1 - this->L2 * c12;
    return p_;
}

// inverse_kinematic from body frame to joint space
Eigen::Vector3<double> Robot::inverse_kinematic(Eigen::Vector3<double> p_, int leg_)
{
    Eigen::Vector3<double> q_,foot_hip_frame;
    q_.setZero();
    foot_hip_frame = p_-hip_location[leg_];
    
    double c2 = (pow(foot_hip_frame.x(), 2) + pow(foot_hip_frame.z(), 2) - pow(this->L2, 2) - pow(this->L2, 2)) / (2 * this->L2 * this->L2);
    q_.z() = acos(c2);
    if (q_.z() > 0)
    {
        q_.z() = -q_.z();
    }
    double s2 = sin(q_.z());

    double c1 = (-(this->L2 + L2 * c2) * foot_hip_frame.z() - this->L2 * s2 * foot_hip_frame.x()) / (pow(foot_hip_frame.x(), 2) + pow(foot_hip_frame.z(), 2));
    q_.y() = acos(c1);

    double x_output, z_output;
    x_output = -this->L2 * sin(q_.y()) - this->L2 * sin(q_.y() + q_.z());
    z_output = -this->L2 * cos(q_.y()) - this->L2 * cos(q_.y() + q_.z());
    if (abs(x_output - foot_hip_frame.x()) > 0.02 || abs(z_output - foot_hip_frame.z()) > 0.02)
    {
        q_.y() = -q_.y();
        x_output = -this->L2 * sin(q_.y()) - this->L2 * sin(q_.y() + q_.z());
        z_output = -this->L2 * cos(q_.y()) - this->L2 * cos(q_.y() + q_.z());
    }
    // std::cout << "foot" << leg_ << std::endl;
    // std::cout << "theta1 = " << q_.y() << " theta2 = " << q_.z() << std::endl;
    // std::cout << "x = " << foot_hip_frame.x() << " z = " << foot_hip_frame.z() << std::endl;
    // std::cout << "x_output = " << x_output << " z_output = " << z_output << std::endl;
    return q_;
}

// calcualte jacobian matrix for each leg_
Eigen::Matrix3<double> Robot::cal_jacobian(Eigen::Vector3<double> q_, int leg_)
{
    Eigen::Matrix3<double> J;
    double s1 = std::sin(q_(1));
    double s2 = std::sin(q_(2));
    double c1 = std::cos(q_(1));
    double c2 = std::cos(q_(2));
    double s12 = s1 * c2 + c1 * s2;
    double c12 = c1 * c2 - s1 * s2;
    J(0, 0) = 0;
    J(0, 1) = -(L1 * c1 + L2 * c12);
    J(0, 2) = -(L2 * c12);
    J(1, 0) = L2 * c12 + L1 * c1;
    J(1, 1) = 0;
    J(1, 2) = 0;
    J(2, 0) = 0;
    J(2, 1) = L1 * s1 + L2 * s12;
    J(2, 2) = L2 * s12;
    return J;
}
