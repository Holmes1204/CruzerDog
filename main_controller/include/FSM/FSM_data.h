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
    // state_data* state;
    LegController *_legController;
    A1BasicEKF *model_StateEstimate;
    Robot *_quadruped;
    // GaitScheduler* _gaitScheduler;
    // DesiredStateCommand<T>* _desiredStateCommand;
    volatile int global_state_switch = 0;
    volatile int global_gait_switch = 0;
    FSM_data();
    ~FSM_data();
};

#endif //