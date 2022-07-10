#ifndef _LEG_CONTROL_
#define _LEG_CONTROL_
#include <eigen3/Eigen/Dense>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }
  void zero();
  Eigen::Vector3d q, qd;//joint space
  Eigen::Vector3d p, v;//hip frame
  Eigen::Matrix3d  J;//body frame jacobian
  Eigen::Vector3d tauEstimate;

};


struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }
  void zero();
  Eigen::Vector3d q_Des, qd_Des,tau_FF;//joint space
  Eigen::Vector3d p_Des, v_Des,force_FF;//hip frame
  Eigen::Matrix3d kpCartesian, kdCartesian, kpJoint, kdJoint;
};


//leg Controller 的定位是标准模型和驱动电机(仿真或者实际)的唯一接口
//FSM_Data -> leg controller -> motor
class LegController
{
public:
    LegControllerCommand command[4];
    LegControllerData data[4];
    
    LegController();
    ~LegController();
};


#endif 