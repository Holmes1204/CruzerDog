#ifndef _STAND_WORKER_
#define _STAND_WORKER_
#include "State_Worker.h"
#include <eigen_types.h>
#include <defination.h>
using namespace quad;
using namespace Eigen;

class StandWorker: public StateWorker{
private:

public:
    FSM_data &data_;
    FSM_topic_control &tpcl_;
    int flag_ = 0;
    int count_;
    double targetPos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};

    uint32_t iter_run;
    float iter_time_ms;

    Vec3<double> f_t[4];
    Vec3<double> f_0[4];
    virtual void send();
    virtual void run();
    virtual bool is_finished();

    StandWorker(FSM_data &data,FSM_topic_control &tpcl);
    ~StandWorker();
};
#endif