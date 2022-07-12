#include <Robot/Robot.h>
Robot::Robot(double mass_, double len_1, double len_2, double x_offset, double y_offset, Mat3<double> inerial)
{
    mass = mass_;
    L1 = len_1;
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

// forward_kinematic from  joint space to hip frame
Vec3<double> Robot::forward_kinematic(Vec3<double> q_, int leg_)
{
    Vec3<double> p_;
    double s1 = std::sin(q_(1));
    double c1 = std::cos(q_(1));
    double s12 = std::sin(q_(1) + q_(2));
    double c12 = std::cos(q_(1) + q_(2));
    // p_.x() = this->hip_location[leg_].x() - this->L1 * s1 - this->L2 * s12;
    // p_.y() = this->hip_location[leg_].y();
    // p_.z() = this->hip_location[leg_].z() - this->L1 * c1 - this->L2 * c12;
    p_.x() = -this->L1 * s1 - this->L2 * s12;
    p_.y() = 0.0f;
    p_.z() = -this->L1 * c1 - this->L2 * c12;
    return p_;
}

// inverse_kinematic from hip frame to joint space
Vec3<double> Robot::inverse_kinematic(Vec3<double> p_, int leg_)
{
    Vec3<double> q_;
    q_.setZero();
    double L = p_.norm();
    //assert(L < L1 + L2&&L>0.001);
    if (L>L1+L2)
    {
        L=L1+L2;
    }
    if (L<0.001)
    {
        L=0.001;
    }
    
    
    double t0 = atan2(p_(0),p_(2));
    if (t0>0)
    {
        t0-=2*M_PI;
    }
    double t1 = std::acos((L1*L1+L*L-L2*L2)/(2*L1*L));
    double t2 = std::acos((L1*L1+L2*L2-L*L)/(2*L1*L2));
    q_(1)=t0 + t1 +M_PI;
    q_(2)=t2 - M_PI;
    // std::cout << "foot" << leg_ << std::endl;
    // std::cout << "theta1 = " << q_.y() << " theta2 = " << q_.z() << std::endl;
    // std::cout << "x = " << p_.x() << " z = " << p_.z() << std::endl;
    // std::cout << "x_output = " << x_output << " z_output = " << z_output << std::endl;
    return q_;
}

// calcualte jacobian matrix for each leg_
Mat3<double> Robot::cal_jacobian(Vec3<double> q_, int leg_)
{
    Mat3<double> J;
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
