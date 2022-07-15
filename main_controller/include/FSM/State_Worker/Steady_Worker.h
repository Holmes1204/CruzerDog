#ifndef _STEADY_WORKER_
#define _STEADY_WORKER_

#include <FSM/FSM_state.h>

class SteadyWorker : public FSM_State
{
public:
    uint32_t iter_run;
    double iter_time_ms;
    double init_motor_angle[4][2] = {{-1.44121, 2.87804}, {1.43246, -2.88679}, {-1.44121, 2.87804}, {1.43246, -2.88679}};
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
