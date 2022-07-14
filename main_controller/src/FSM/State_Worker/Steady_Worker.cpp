#include <FSM/State_Worker/Steady_Worker.h>

SteadyWorker::SteadyWorker(FSM_data *data)
    : FSM_State(data, FSM_StateName::STEADY, "STEADY")
{
    this->iter_run = 0;
    this->iter_time_ms = 0.0f;
    for (int i = 0; i < 4; i++)
    {
    }
}

SteadyWorker::~SteadyWorker()
{
}

void SteadyWorker::onExit()
{
    std::cout<<"steady finished!"<<std::endl;
}

void SteadyWorker::onEnter()
{
    std::cout<<"steady start!"<<std::endl;
}
//不会一直卡在一个run中运行
void SteadyWorker::run()
{
    // check all state
    this->iter_run++;
    return;
}

//各个状态的切换，
FSM_StateName SteadyWorker::checkTransition()
{
    if (1)
    {
        return FSM_StateName::STAND_UP;
    }else{
        return FSM_StateName::STEADY;
    }
}

// Runs the transition behaviors and returns true when done transitioning
TransitionData SteadyWorker::transition()
{
    transitionData.done = true;
    return transitionData;
}