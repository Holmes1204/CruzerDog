#ifndef _FSM_DATA_
#define _FSM_DATA_
#include <ros/ros.h>
#include <eigen_types.h>
#include <defination.h>
#include <Robot/Robot.h>
#include <Convex_MPC/MConvexMPC.h>
#include <Leg_Control/Leg_Control.h>
#include <State_Estimator/A1BasicEKF.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>

using namespace quad;

class FSM_data;
class A1BasicEKF;
class Solver;
class Robot;


class FSM_data
{
private:
    /* data */
public:
    //GaitScheduler* _gaitScheduler;
    //RobotControlParameters* controlParameters;
    //DesiredStateCommand<T>* _desiredStateCommand;
    //LegController<T>* _legController;
    // state_data* state;
    LegController* _legController;
    A1BasicEKF* model_StateEstimate;
    //Solver* mpc_solver;
    Robot* _quadruped;
    volatile int global_state_switch =0;
    volatile int global_gait_switch=0;
    FSM_data();
    ~FSM_data();
};


#endif//