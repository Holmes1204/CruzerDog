#ifndef _STAND_WORKER_
#define _STAND_WORKER_
#include <FSM/FSM_state.h>
#include <eigen_types.h>
#include <defination.h>
using namespace quad;
using namespace Eigen;

class StandWorker : public FSM_State
{
private:
public:
    bool switch_conditon_check = false;
    int count_;
    double targetPos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};

    uint32_t iter_run;
    float iter_time_ms;

    Vec3<double> f_t[4];
    Vec3<double> f_0[4];

    virtual void run();
    virtual void onEnter();
    virtual void onExit();
    
    virtual FSM_StateName checkTransition();
    virtual TransitionData transition();

    StandWorker(FSM_data *data);
    ~StandWorker();
};
#endif