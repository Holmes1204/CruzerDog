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

    // ConvexMPCLocomotion* cMPCOld;
    // WBC_Ctrl<T> * _wbc_ctrl;
    // LocomotionCtrlData<T> * _wbc_data;
    MPC_SLOVER *mpc_solver;
    int iterationsBetweenMPC;
    int horizonLength;
    int default_iterations_between_mpc;
    double dt;
    double dtMPC;
    int iterationCounter = 0;
    int iter = 0;
    // debug print path
    std::ofstream file;

    bool firstRun = true;
    FootSwingTrajectory footSwingTrajectories[4];
    //
    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;

    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;

    Vec3<double> pFoot_des[4];
    Vec3<double> vFoot_des[4];
    Vec3<double> aFoot_des[4];
    Vec3<double> p_Body[4];
    Vec12<double> Fr_des;

    Vec4<double> contact_state;
    double _yaw_turn_rate;
    double _yaw_des;

    double _roll_des;
    double _pitch_des;

    double _x_vel_des = 0.;
    double _y_vel_des = 0.;

    // High speed running
    // double _body_height = 0.34;
    double _body_height = 0.29;
    double _body_height_running = 0.29;
    double _body_height_jumping = 0.36;

    Vec3<double> f_ff[4];
    Vec4<double> swingTimes;
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstSwing[4];
    double swingTimeRemaining[4];
    double stand_traj[6];
    int current_gait;
    int gaitNumber;
    Mat34<double> r_foot_world;

    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    double x_comp_integral = 0;
    Vec3<double> pFoot[4];
    OffsetDurationGait trotting;

    //
    virtual void run();
    virtual void onEnter();
    virtual void onExit();
    virtual FSM_StateName checkTransition();
    virtual TransitionData transition();

    void b_run();
    void updateMPCIfNeeded(int *mpcTable);
    Locomotion(FSM_data *data);
    ~Locomotion();
};

#endif