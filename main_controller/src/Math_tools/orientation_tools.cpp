#include <Math_tools/orientation_tools.h>

Vec3<double> ori::quatToRPY(const Eigen::Quaterniond &q_)
{
    Vec3<double> rpy;
    double q[4];
    q[0] = q_.w();
    q[1] = q_.x();
    q[2] = q_.y();
    q[3] = q_.z();
    double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) =
        std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                   square(q[0]) + square(q[1]) - square(q[2]) - square(q[3]));
    rpy(1) = std::asin(as);
    rpy(0) =
        std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                   square(q[0]) - square(q[1]) - square(q[2]) + square(q[3]));
    return rpy;
}