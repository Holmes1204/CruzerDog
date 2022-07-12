#include <FSM/State_Worker/Steady_Worker.h>

SteadyWorker::SteadyWorker(FSM_data &data, FSM_topic_control &tpcl) : data_(data), tpcl_(tpcl)
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

//不会一直卡在一个run中运行
void SteadyWorker::run()
{
    this->iter_run++;
    return;
}

void SteadyWorker::send()
{
}

bool SteadyWorker::is_finished()
{
    if (iter_run > 100)
    {
        for (int i = 0; i < 4; i++)
        {
            std::cout <<"leg"<< i << " -------------- " << std::endl;
            std::cout << "q" << std::endl
                      << data_._legController->data[i].q.transpose() << std::endl
                      << "p" << std::endl
                      << data_._legController->data[i].p.transpose()
                      << std::endl;
        }
        std::cout << "Steady State Over!\n"
                  << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}
