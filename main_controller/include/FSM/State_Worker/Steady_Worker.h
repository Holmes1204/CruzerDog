#ifndef _STEADY_WORKER_
#define _STEADY_WORKER_

#include <FSM/FSM_state.h>

class SteadyWorker : public FSM_State
{
public:
    uint32_t iter_run;
    double iter_time_ms;
    bool switch_conditon_check = false;
    bool init_angle =false;

    virtual void run();
    virtual void onEnter();
    virtual void onExit();
    virtual FSM_StateName checkTransition();
    virtual TransitionData transition();

    SteadyWorker(FSM_data *data);
    ~SteadyWorker();
private:
};

#endif
