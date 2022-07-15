#include "FSM/State_Worker/Stand_Worker.h"

StandWorker::StandWorker(FSM_data *data)
    : FSM_State(data, FSM_StateName::STAND_UP, "STAND_UP")
{
    this->iter_run = 0;
    this->iter_time_ms = 0.0f;
    turnOnAllSafetyChecks();
}

StandWorker::~StandWorker()
{
}

void StandWorker::onExit()
{

    std::cout << "steady finished!" << std::endl;
}

void StandWorker::onEnter()
{    
    iter_run = 0;
    std::cout << "stand start!" << std::endl;
}

FSM_StateName StandWorker::checkTransition()
{

    if (data_->global_state_switch)
    {
        data_->global_state_switch = 0;
        std::cout << stateString << " to "
                  << "Locomotion" << std::endl;
        return FSM_StateName::LOCOMOTION;
    }
    else
    {
        return FSM_StateName::STAND_UP;
    }
}

// Runs the transition behaviors and returns true when done transitioning
TransitionData StandWorker::transition()
{
    transitionData.done = true;
    return transitionData;
}
//不会一直卡在一个run中运行
void StandWorker::run()
{
    if (iter_run == 0)
    {
        data_->_legController->motor_enable = 1;
        for (int i = 0; i < 4; i++)
        {
            f_0[i] = data_->_legController->data[i].p; // hip frame
            // f_0[i] = Vec3<double>(0, 0, -0.1);
            f_t[i] = Vec3<double>(0, 0, -0.3);
        }
    }

    // debug
    {
        static uint32_t dbg_count = 0;
        if (dbg_count % 40 == 0)
        {
            system("clear");
            for (int i = 0; i < 4; i++)
            {
                std::cout << "leg" << i << std::endl;
                std::cout << "state" << data_->_legController->motor_enable << std::endl;
                std::cout << "J " << std::endl
                          << data_->_legController->data[i].J << std::endl;
                std::cout << "q " << std::endl
                          << data_->_legController->data[i].q.transpose() << "\n"
                          << "d " << std::endl
                          << data_->_legController->data[i].qd.transpose() << std::endl;
                std::cout << "p " << std::endl
                          << data_->_legController->data[i].p.transpose() << "\n"
                          << "v " << std::endl
                          << data_->_legController->data[i].v.transpose() << std::endl;
                dbg_count = 0;
            }
        }
        dbg_count++;
    }
    double percent;
#define period 500u // 1s
    if (iter_run < period)
    {
        percent = iter_run / double(period);
    }
    else
    {
        percent = 1;
    }
    for (int leg = 0; leg < 4; leg++)
    {
        data_->_legController->command[leg].zero();
        data_->_legController->command[leg].q_Des = this->data_->_quadruped->inverse_kinematic(f_0[leg] * (1 - percent) + f_t[leg] * percent, 0);
        data_->_legController->command[leg].kpJoint(1, 1) = 50;
        data_->_legController->command[leg].kdJoint(1, 1) = .5;
        data_->_legController->command[leg].kpJoint(2, 2) = 50;
        data_->_legController->command[leg].kdJoint(2, 2) = .5;
    }
    // static int diter = -1;
    // if (iter_run > period)
    // {
    //     if (diter > 0)
    //     {
    //         diter = -1;
    //     }
    // }
    // else if (iter_run < 1)
    // {
    //     if (diter < 0)
    //     {
    //         diter = 1;
    //     }
    // }
    // this->iter_run += diter;
    this->iter_run++;
    return;
}
