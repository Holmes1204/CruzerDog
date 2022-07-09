#ifndef _STAND_WORKER_
#define _STAND_WORKER_
#include "State_Worker.h"
#include "eigen3/Eigen/Dense"
#include <defination.h>
using namespace quad;
using namespace Eigen;

class StandWorker: public StateWorker{
private:

public:
    int flag_ = 0;
    int count_;
    double command_hip_ = 0;
    double command_knee_ = 0;
    double init_time_, cur_time_, end_time_;
    double init_hip_ = 90.0, init_knee_ = -175.0;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;
    double targetPos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    double lastPos[12];
    uint32_t iter_run;
    float iter_time_ms;

    FSM_data &data_;
    
    FSM_topic_control &tpcl_;
    Vector3f f_t[4];
    Vector3f f_0[4];
    virtual void send();
    virtual void run();
    virtual bool is_finished();

    StandWorker(FSM_data &data,FSM_topic_control &tpcl);
    ~StandWorker();
};
#endif