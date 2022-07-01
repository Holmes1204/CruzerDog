#ifndef VILEOM_A1KINEMATICS_H
#define VILEOM_A1KINEMATICS_H

#include <eigen3/Eigen/Dense>

class LegKinematics
{

public:
    LegKinematics() = default;
    ~LegKinematics() = default;

    const int RHO_OPT_SIZE = 3;
    const int RHO_FIX_SIZE = 5;
    // rho opt are contact offset cx cy cz
    // rho fix are body offset x& y, thigh offset, upper leg length, lower leg length
    // functions with eigen interface
    // forward kinematics 3x1
    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix)
    {
        Eigen::Vector3d out;
        autoFunc_fk_derive(q.data(), rho_opt.data(), rho_fix.data(), out.data());
        return out;
    }
    // jacobian   3x3
    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix)
    {
        Eigen::Matrix3d mtx;
        autoFunc_d_fk_dq(q.data(), rho_opt.data(), rho_fix.data(), mtx.data());
        return mtx;
    }
    // the partial derivative of fk wrt rho opt   3x3
    Eigen::Matrix3d dfk_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix)
    {
        Eigen::Matrix3d mtx;
        autoFunc_d_fk_dc(q.data(), rho_opt.data(), rho_fix.data(), mtx.data());
        return mtx;
    }
    // the partial derivative of jacobian wrt q    9x3
    Eigen::Matrix<double, 9, 3> dJ_dq(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix)
    {
        Eigen::Matrix<double, 9, 3> mtx;
        autoFunc_dJ_dq(q.data(), rho_opt.data(), rho_fix.data(), mtx.data());
        return mtx;
    }
    // the partial derivative of jacobian wrt rho opt   9x3
    Eigen::Matrix<double, 9, 3> dJ_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix)
    {
        Eigen::Matrix<double, 9, 3> mtx;
        autoFunc_dJ_dpho(q.data(), rho_opt.data(), rho_fix.data(), mtx.data());
        return mtx;
    }

private:
    // functions with basic C++ interface, generated by Matlab
    void autoFunc_fk_derive(const double in1[3], const double in2[3], const double in3[5], double p_bf[3]);
    void autoFunc_d_fk_dq(const double in1[3], const double in2[3], const double in3[5], double jacobian[9]);
    void autoFunc_d_fk_dc(const double in1[3], const double in2[3], const double in3[5], double d_fk_dc[9]);
    void autoFunc_dJ_dq(const double in1[3], const double in2[3], const double in3[5], double dJ_dq[27]);
    void autoFunc_dJ_dpho(const double in1[3], const double[3], const double[5], double dJ_dpho[27]);
};

#endif // VILEOM_A1KINEMATICS_H
