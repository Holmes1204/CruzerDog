#ifndef _ROBOT_
#define _ROBOT_

#include <eigen3/Eigen/Dense>
#include <defination.h>
#include "LegKinematics_unitree.h"
#include "KinematicModel.h"
#include <vector>
#include <FSM/FSM_data.h>
using namespace quad;
class FSM_data;
/*!
 * Data returned from the legs to the control code.
 */
struct LegControllerData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p, v;
    Eigen::Matrix2d J_temp;
    Eigen::Matrix2d J_temp_inv;
    Eigen::Matrix3d J;
    Eigen::Matrix3d J_inv;
    /*! angle of three joints*/
    Eigen::Vector3d q;
    Eigen::Vector3d qd;

    LegControllerData() { zero(); }

    void zero()
    {
        p = Eigen::Vector3d::Zero();
        v = Eigen::Vector3d::Zero();
        J_temp = Eigen::Matrix2d::Zero();
        J_temp_inv = Eigen::Matrix2d::Zero();
        J = Eigen::Matrix3d::Zero();
        J_inv = Eigen::Matrix3d::Zero();
    }
};

/*!
 * Data sent from the control algorithm to the legs.
 */
struct LegControllerCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d qDes, qdDes, pDes, vDes,forceFeedForward;
    Eigen::Matrix3<double> kpCartesian,kdCartesian,kdJoint,kpJoint;

    LegControllerCommand() { zero(); }

    void zero()
    {
        qDes = Eigen::Vector3d::Zero();
        qdDes = Eigen::Vector3d::Zero();
        pDes = Eigen::Vector3d::Zero();
        vDes = Eigen::Vector3d::Zero();
        forceFeedForward = Eigen::Vector3d::Zero();
        kpCartesian = Eigen::Matrix3<double>::Zero();
        kdCartesian = Eigen::Matrix3<double>::Zero();
        kpJoint = Eigen::Matrix3<double>::Zero();
        kdJoint = Eigen::Matrix3<double>::Zero();
    }
};

class LegController
{
public:
    double dt_;
    LegControllerCommand command[4];
    LegControllerData data[4];

    KinematicModel *model_kinematic;
    LegKinematics *model_LegKinematic;

    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    LegController();
    ~LegController();

    void FirstUpdateData(FSM_data &data);

    void UpdateData();
    void UpdateCommand();
    void ComputeJacobian(Eigen::Vector3d &q, Eigen::Matrix2d *J, Eigen::Matrix3d *J_3DoF, int leg);
};

class Robot
{
public:
    Robot(){};
    ~Robot(){};
    Eigen::Vector3<double> getHipLocation(int leg){
        (void)leg;
        return Eigen::Vector3<double>::Zero();
    }
};

#endif