#ifndef _ROBOT_MODEL_
#define _ROBOT_MODEL_

#include <defination.h>
#include <vector>
#include <FSM/FSM_data.h>
#include <eigen_types.h>
using namespace quad;

//Model for 8DOF ROBOT
class Robot
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double mass;//机器人总的质量，而非单独躯干的质量,Kg
    double L1,L2;//L1 thigh(mid) 和 calf(down)电机转轴的距离，L2是calf电机转轴到足端中心点的距离,metre
    Mat3<double> inertial_tensor;//kg*m^2
    Vec3<double> hip_location[4];//metre, in body frame
    Vec3<double> CoM;//机器人的质量中心,metre, in body frame
    Robot(double mass_,double len_1,double len_2,double x_offset,double y_offset,Mat3<double>interial);
    ~Robot();
    Vec3<double> getHipLocation(int leg_){
        return hip_location[leg_];
    }
    //forward_kinematic from  joint space to body frame 
    Vec3<double> forward_kinematic(Vec3<double> q_,int leg_);
    //inverse_kinematic from body frame to joint space
    Vec3<double> inverse_kinematic(Vec3<double> p_,int leg_);
    //calcualte jacobian matrix for each leg
    Mat3<double> cal_jacobian(Vec3<double> q_,int leg_);
};
// Robot my_robot(23.0,0.2,0.2,0.1805,0.1308,Mat3::Zero());

#endif