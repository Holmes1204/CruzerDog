#ifndef _LEG_CONTROL_
#define _LEG_CONTROL_
#include <eigen_types.h>
struct LegControllerData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }
  void zero();
  Vec3<double> q, qd;//joint space
  Vec3<double> p, v;//hip frame
  Mat3<double>  J;//body frame jacobian
  Vec3<double> tauEstimate;

};


struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }
  void zero();
  Vec3<double> q_Des, qd_Des,tau_FF;//joint space
  Vec3<double> p_Des, v_Des,force_FF;//hip frame
  Mat3<double> kpCartesian, kdCartesian, kpJoint, kdJoint;
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