#include <FSM/State_Worker/Steady_Worker.h>

SteadyWorker::SteadyWorker(FSM_data *data)
    : FSM_State(data, FSM_StateName::STEADY, "STEADY")
{
    this->iter_run = 0;
    this->iter_time_ms = 0.0f;
}

SteadyWorker::~SteadyWorker()
{
}

void SteadyWorker::onExit()
{
    std::cout << "steady finished!" << std::endl;
}

void SteadyWorker::onEnter()
{
    std::cout << "steady start!" << std::endl;
}
//不会一直卡在一个run中运行
void SteadyWorker::run()
{

    data_->_legController->motor_enable = 0;
    // // debug
    {
        static uint32_t dbg_count = 0;

        if (dbg_count % 20 == 0)
        {
            system("clear");
            for (int fsd = 0; fsd < 4; fsd++)
            {
                std::cout << "pos" << fsd << std::endl
                          << data_->_legController->motor_data[fsd].q.transpose() << std::endl
                          << std::endl;
            }
            if (init_angle)
            {
                for (int i = 0; i < 4; i++)
                {
                    std::cout << "leg " << i << " offset :" << data_->_legController->offset[i].transpose() << std::endl;
                }
            }
            dbg_count = 0;
        }
        dbg_count++;
    }

    if (iter_run > int(quad::df))
    {
        if (!init_angle)
        {
            init_angle = true;
            for (int i = 0; i < 4; i++)
            {
                data_->_legController->set_offset(data_->_legController->motor_data[i].q, i);
            }
        }
    }
    this->iter_run++;
    return;
}

//各个状态的切换，
FSM_StateName SteadyWorker::checkTransition()
{
    if (data_->global_state_switch)
    {
        data_->global_state_switch = 0;
        std::cout << stateString << " to "
                  << "Stand" << std::endl;
        return FSM_StateName::STAND_UP;
    }
    else
    {
        return FSM_StateName::STEADY;
    }
}

// Runs the transition behaviors and returns true when done transitioning
TransitionData SteadyWorker::transition()
{
    transitionData.done = true;
    return transitionData;
}