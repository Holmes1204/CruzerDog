#ifndef _LOCOMOTION_WORKER_
#define _LOCOMOTION_WORKER_
#include "State_Worker.h"
#include "FSM/FSM_data.h"
#include "FSM/FSM_tpcl.h"
#include "Leg_Control/FootSwingTrajectory.h"
#include "Leg_Control/Gait.h"
#include <fstream>
#include <iostream>
class Locomotion : public StateWorker
{
private:
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int iter = 0;
    // ConvexMPCLocomotion* cMPCOld;
    // WBC_Ctrl<T> * _wbc_ctrl;
    // LocomotionCtrlData<T> * _wbc_data;
    FSM_data &data_;
    FSM_topic_control &tpcl_;
    //debug print path
    std::ofstream file;
    
    bool firstRun=true;
    FootSwingTrajectory footSwingTrajectories[4];
    //
    Eigen::Vector3<double> pBody_des;
    Eigen::Vector3<double> vBody_des;
    Eigen::Vector3<double> aBody_des;

    Eigen::Vector3<double> pBody_RPY_des;
    Eigen::Vector3<double> vBody_Ori_des;

    Eigen::Vector3<double> pFoot_des[4];
    Eigen::Vector3<double> vFoot_des[4];
    Eigen::Vector3<double> aFoot_des[4];

    Eigen::Vector3<double> Fr_des[4];

    Eigen::Vector4<double> contact_state;


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

    int iterationsBetweenMPC;
    int horizonLength;
    int default_iterations_between_mpc;
    double dt;
    double dtMPC;
    int iterationCounter = 0;

    Eigen::Vector3<double> f_ff[4];
    Eigen::Vector4<double> swingTimes;
    OffsetDurationGait trotting;
    Eigen::Matrix3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstSwing[4];
    double swingTimeRemaining[4];
    double stand_traj[6];
    int current_gait;
    int gaitNumber;

    Eigen::Vector3<double> world_position_desired;
    Eigen::Vector3<double> rpy_int;
    Eigen::Vector3<double> rpy_comp;
    double x_comp_integral = 0;
    Eigen::Vector3<double> pFoot[4];
    double trajAll[12 * 36];
    //

    virtual void send();
    virtual void run();
    virtual bool is_finished();
    void b_run();
    Locomotion(FSM_data &data, FSM_topic_control &tpcl);
    ~Locomotion();
};

#endif