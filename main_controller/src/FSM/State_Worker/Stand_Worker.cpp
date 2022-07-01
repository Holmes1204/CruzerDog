#include "FSM/State_Worker/Stand_Worker.h"
#include <mutex>
StandWorker::StandWorker(FSM_data &data,FSM_topic_control &tpcl):data_(data),tpcl_(tpcl){
    this->iter_run=0;
    this->iter_time_ms=0.0f;
    for (int i = 0; i < 4; i++)
    {
        f_t[i] = Vector3f::Zero();
        f_0[i] = Vector3f::Zero();
    }
    
}

StandWorker::~StandWorker() {

}


void StandWorker::send(){

}

//不会一直卡在一个run中运行
void StandWorker::run() {
    if (iter_run==0)
    {    for (int i = 0; i < 4; i++)
        {

        }
    }
    
    for (int i = 0; i < 4; i++)
    {
      
    }

    if (iter_run%50==0){


        // std::cout<<"------------"<<std::endl;
        // std::cout<<0<<data_.leg[0].p_ref.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].p.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].force.transpose()<<std::endl;
    }

    //leg_angel 增减
    this-> iter_run++;
    //std::cout<<"iter_run"<<iter_run<<std::endl;
    return;
}


bool StandWorker::is_finished() {
    if(0)
        {
            std::cout<<"Stand finished!"<<std::endl;
            for (int i = 0; i < 4; i++)
            {   std::cout<<i<<" -------------- "<<std::endl;

            }
            return true;
        }
    else
        {return false;}
}

