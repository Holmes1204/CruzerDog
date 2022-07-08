#include "FSM/FiniteStateMachine.h"
#include "FSM/State_Worker/Stand_Worker.h"
#include "FSM/State_Worker/Steady_Worker.h"
#include "FSM/State_Worker/Locomotion_Worker.h"
#include "defination.h"

FSM::FSM(ros::NodeHandle &nh) :global_data(),topic_contrl(nh,global_data)
{

}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::build_ScheduleTable(int Schedule, ...) {
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != quad::END){
        switch (Schedule) {
            case quad::STEADY:{
                SteadyWorker* tmp_Worker = new SteadyWorker(this->global_data,this->topic_contrl);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::STAND:{
                StandWorker* tmp_Worker = new StandWorker(this->global_data,this->topic_contrl);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::LOCOMOTION:{
                Locomotion* tmp_Worker = new Locomotion(this->global_data,this->topic_contrl);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            /*
            case quad::TROT:{
                TrotWorker* tmp_Worker = new TrotWorker(this->global_data,this->topic_contrl);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::STOP:{
                StopWorker* tmp_Worker = new StopWorker(this->global_data,this->topic_contrl);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            */
            default:
                ROS_ERROR("Wrong type of Schedule Table");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}



void FSM::loop() {
    //this->global_data.state->plan_dt = 0.01;
    /*! running the schedule table */
    if(this->Workers[this->flow]->is_finished()){
        this->flow++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable");
            exit(0);
        }
    }
    else{
        /*! Update Leg Control Data */
        //Update_LegController();
        /*
        if(!this->global_data.model_StateEstimate->is_inited()){
            this->global_data.model_StateEstimate->init_state(this->global_data);
        }else{
            Update_StateEstimate();
        }*/
        this->Workers[this->flow]->run();    
    }
}


void FSM::Update_LegController(){
    /*! Update feedback */

}


//MPC
inline void FSM::Update_MPC() {
    this->global_data.state->foot_contact_force = this->global_data.mpc_solver->Calculate_contact_force(this->global_data, this->global_data.state->plan_dt);
}



//State_Estimator
inline void FSM::Update_StateEstimate() {
    this->global_data.model_StateEstimate->update_estimation(this->global_data,this->global_data.state->plan_dt);
    this->global_data.state->cur_vel_b = this->global_data.state->rotate_matrix_z.transpose() * this->global_data.state->cur_vel;
    this->global_data.state->command_vel_b = this->global_data.state->rotate_matrix_z.transpose() * this->global_data.state->command_vel;

}

