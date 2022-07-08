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