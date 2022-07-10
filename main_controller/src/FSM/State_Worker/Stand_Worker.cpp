#include "FSM/State_Worker/Stand_Worker.h"
#include <Convex_MPC/ConvexMPC.h>

StandWorker::StandWorker(FSM_data &data, FSM_topic_control &tpcl) : data_(data), tpcl_(tpcl)
{
    this->iter_run = 0;
    this->iter_time_ms = 0.0f;
}

StandWorker::~StandWorker()
{
}

void StandWorker::send()
{
}

//不会一直卡在一个run中运行
void StandWorker::run()
{

    if (iter_run == 0)
    {
        for (int i = 0; i < 4; i++)
        {
        }
    }

    for (int i = 0; i < 4; i++)
    {
    }

    if (iter_run % 50 == 0)
    {

        // std::cout<<"------------"<<std::endl;
        // std::cout<<0<<data_.leg[0].p_ref.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].p.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].force.transpose()<<std::endl;
    }
    double percent;

#define period 1000u
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
        data_._legController->command[leg].zero();
        data_._legController->command[leg].q_Des(1) = 0.0f * (1 - percent) + targetPos[1] * percent;
        data_._legController->command[leg].qd_Des(1) = 0;
        data_._legController->command[leg].kpJoint(1, 1) = 300;
        data_._legController->command[leg].kdJoint(1, 1) = 15;
        data_._legController->command[leg].q_Des(2) = 0.0f * (1 - percent) + targetPos[2] * percent;
        data_._legController->command[leg].qd_Des(2) = 0;
        data_._legController->command[leg].kpJoint(2, 2) = 300;
        data_._legController->command[leg].kdJoint(2, 2) = 15;
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
    tpcl_.unitree_sim_send_cmd();
    this->iter_run ++;
    return;
}

bool StandWorker::is_finished()
{
    if (iter_run > 1+period)
    {
        std::cout << "Stand finished!" << std::endl;
        // for (int i = 0; i < 4; i++)
        // {
        //     std::cout << i << " -------------- " << std::endl;
        // }
        return true;
    }
    else
    {
        return false;
    }
}
