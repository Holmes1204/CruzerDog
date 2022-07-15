#ifndef _LEG_CONTROL_
#define _LEG_CONTROL_
#include <eigen_types.h>
struct LegControllerData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerData() { zero(); }
  void zero();
  Vec3<double> q, qd; // joint space
  Vec3<double> p, v;  // hip frame
  Mat3<double> J;     // body frame jacobian
  Vec3<double> tauEstimate;
};

struct LegControllerCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }
  void zero();
  Vec3<double> q_Des, qd_Des, tau_FF;  // joint space
  Vec3<double> p_Des, v_Des, force_FF; // hip frame
  Mat3<double> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

struct MotorDataRaw
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotorDataRaw() { zero(); }
  void zero()
  {
    q = Vec3<double>::Zero();
    qd = Vec3<double>::Zero();
    tau = Vec3<double>::Zero();
  }
  Vec3<double> q, qd, tau;
};

struct MotorCommandRaw
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotorCommandRaw() { zero(); }
  void zero()
  {
    q_Des = Vec3<double>::Zero();
    qd_Des = Vec3<double>::Zero();
    tau_FF = Vec3<double>::Zero();
    kp = Vec3<double>::Zero();
    kd = Vec3<double>::Zero();
  }
  Vec3<double> q_Des,
      qd_Des, tau_FF, kp, kd;
};

// leg Controller 的定位是标准模型和驱动电机(仿真或者实际)的唯一接口
// FSM_Data -> leg controller -> motor
class LegController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // leg LF 0 RF 1 LB 2 RB 3
  LegControllerCommand command[4];
  LegControllerData data[4];

  // real world parameters
  MotorCommandRaw motor_cmd[4];
  MotorDataRaw motor_data[4];
  uint32_t motor_enable = 0;                  // 1 if enable all teh motor
  Vec3<double> offset[4];                     // motor offset;
  const double direction[4] = {-1, 1, -1, 1}; // motor axis direction leg LF -1 RF 1 ,LB -1 ,RB 1;
  void set_offset(Vec3<double> &pos_offset, int leg)
  {
    offset[leg] = pos_offset;
    return;
  };

    //convert model cmd to motor cmd
  void convert_motor_cmd(int leg)
  {
    motor_cmd[leg].q_Des = direction[leg] * command[leg].q_Des + offset[leg];
    motor_cmd[leg].qd_Des = direction[leg] * command[leg].qd_Des;
    motor_cmd[leg].tau_FF = direction[leg] * command[leg].tau_FF;
    motor_cmd[leg].kp = command[leg].kpJoint.diagonal();
    motor_cmd[leg].kd = command[leg].kdJoint.diagonal();
  }

  //convert motor data to model data
  void convert_motor_data(int leg)
  {
    data[leg].q = direction[leg] *(motor_data[leg].q-offset[leg]);
    data[leg].qd = direction[leg] *motor_data[leg].qd;
    data[leg].tauEstimate = direction[leg] *motor_data[leg].tau;
  }

  LegController();
  ~LegController();
};

#endif