#ifndef _STAND_WORKER_
#define _STAND_WORKER_
#include "State_Worker.h"
#include "eigen3/Eigen/Dense"
using namespace Eigen;
class StandWorker: public StateWorker{
private:

public:
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