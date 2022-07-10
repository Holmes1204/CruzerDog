#include "Leg_Control/Leg_Control.h"
// leg frame is coinciden with body frame but the origin in hip
void LegControllerCommand::zero()
{
    // joint space
    q_Des = Eigen::Vector3d::Zero();
    qd_Des = Eigen::Vector3d::Zero();
    tau_FF = Eigen::Vector3d::Zero();
    kpJoint = Eigen::Matrix3d::Zero();
    kdJoint = Eigen::Matrix3d::Zero();
    //hip frame
    p_Des = Eigen::Vector3d::Zero();
    v_Des = Eigen::Vector3d::Zero();
    force_FF = Eigen::Vector3d::Zero();
    kpCartesian = Eigen::Matrix3d::Zero();
    kdCartesian = Eigen::Matrix3d::Zero();
}

void LegControllerData::zero()
{
    // joint space
    q = Eigen::Vector3d::Zero();
    qd = Eigen::Vector3d::Zero();
    tauEstimate = Eigen::Vector3d::Zero();
    //hip frame
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    // world frame jacobian
    J = Eigen::Matrix3d::Zero();
}

LegController::LegController(){

}

LegController::~LegController()
{

}

