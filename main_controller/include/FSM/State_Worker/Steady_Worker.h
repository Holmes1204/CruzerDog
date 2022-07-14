#ifndef _STEADY_WORKER_
#define _STEADY_WORKER_

#include "State_Worker.h"

class SteadyWorker: public StateWorker{
private:

public:
    uint32_t iter_run;
    double iter_time_ms;
    double init_motor_angle[4][2]={{-1.44121 ,2.87804},{ 1.43246,-2.88679},{-1.44121 ,2.87804},{ 1.43246,-2.88679}};
    bool switch_conditon_check=false;
    FSM_data &data_;
    FSM_topic_control &tpcl_;
    virtual void send();
    virtual void run();
    virtual bool is_finished();
    SteadyWorker(FSM_data &data,FSM_topic_control &tpcl);
    ~SteadyWorker();
};


#endif
