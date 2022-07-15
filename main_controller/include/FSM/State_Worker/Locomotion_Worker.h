#ifndef _LOCOMOTION_WORKER_
#define _LOCOMOTION_WORKER_
#include <FSM/FSM_state.h>
#include "FSM/FSM_data.h"
#include "FSM/FSM_tpcl.h"
#include "Leg_Control/FootSwingTrajectory.h"
#include "Leg_Control/Gait.h"
#include <fstream>
#include <iostream>
#include <eigen_types.h>

class Locomotion : public FSM_State
{
private:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // MPC_SLOVER *mpc_solver;
    int iterationsBetweenMPC;
    int horizonLength;
    double dt;
    double dtMPC;
    int iterationCounter = 0;
    int iter = 0;
    int transition_time;
    // debug print path
    std::ofstream file;
    bool firstRun = true;
    FootSwingTrajectory footSwingTrajectories[4];
    //

    Vec4<double> contact_state;
    Vec4<double> swingTimes;
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstSwing[4];
    double swingTimeRemaining[4];
    int current_gait;
    int gaitNumber;
    Vec3<double> pFoot[4];
    OffsetDurationGait trotting;

    //
    virtual void run();
    virtual void onEnter();
    virtual void onExit();
    virtual FSM_StateName checkTransition();
    virtual TransitionData transition();

    Locomotion(FSM_data *data);
    ~Locomotion();
};

#endif