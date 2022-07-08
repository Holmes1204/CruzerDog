#ifndef _LEG_CONTROL_
#define _LEG_CONTROL_
#include <eigen3/Eigen/Dense>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }
  void zero();

  Eigen::Vector3d q, qd;//joint space
  Eigen::Vector3d p, v;//world frame
  Eigen::Vector3d p_b,v_b;//body frame
  Eigen::Matrix3d  J;//world frame jacobian
  Eigen::Vector3d tauEstimate;

};


struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }
  void zero();
  Eigen::Vector3d q_Des, qd_Des,tau_FF;//joint space
  Eigen::Vector3d p_Des, v_Des,force_FF;//world frame
  Eigen::Vector3d p_b_Des, v_b_Des,force_b_FF;//body frame
  Eigen::Matrix3d kpCartesian, kdCartesian, kpJoint, kdJoint;
};

class LegController
{
public:
    LegControllerCommand command[4];
    LegControllerData data[4];
    
    LegController();
    ~LegController();
};


#endif 