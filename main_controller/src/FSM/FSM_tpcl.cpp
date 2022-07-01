#include <FSM/FSM_tpcl.h>
#include <FSM/FSM_data.h>


FSM_topic_control::FSM_topic_control(ros::NodeHandle &nh,FSM_data &data):nh_(nh),data_(data)
{
  unitree_sim_set_up();

}

FSM_topic_control::~FSM_topic_control()
{
    


//real_world
}
void FSM_topic_control::em_cmd_send(){
    return;
}



//unitree_sim
void FSM_topic_control::unitree_sim_set_up(){
    //joint publisher
    this->joint_sim_unitree_pub[0] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    this->joint_sim_unitree_pub[1] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    this->joint_sim_unitree_pub[2] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    this->joint_sim_unitree_pub[3] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    this->joint_sim_unitree_pub[4] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    this->joint_sim_unitree_pub[5] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    this->joint_sim_unitree_pub[6] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    this->joint_sim_unitree_pub[7] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    this->joint_sim_unitree_pub[8] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);
    this->joint_sim_unitree_pub[9] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    this->joint_sim_unitree_pub[10] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    this->joint_sim_unitree_pub[11] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    //joint subscriber

    this->joint_sim_unitree_sub[0] = this->nh_.subscribe("/a1_gazebo/FL_hip_controller/state", 2,    &FSM_topic_control::FL_hip_state_callback,  this);
    this->joint_sim_unitree_sub[1] = this->nh_.subscribe("/a1_gazebo/FL_thigh_controller/state", 2,  &FSM_topic_control::FL_thigh_state_callback,this);
    this->joint_sim_unitree_sub[2] = this->nh_.subscribe("/a1_gazebo/FL_calf_controller/state", 2,   &FSM_topic_control::FL_calf_state_callback, this);
    this->joint_sim_unitree_sub[3] = this->nh_.subscribe("/a1_gazebo/FR_hip_controller/state", 2,    &FSM_topic_control::FR_hip_state_callback,  this);
    this->joint_sim_unitree_sub[4] = this->nh_.subscribe("/a1_gazebo/FR_thigh_controller/state", 2,  &FSM_topic_control::FR_thigh_state_callback,this);
    this->joint_sim_unitree_sub[5] = this->nh_.subscribe("/a1_gazebo/FR_calf_controller/state", 2,   &FSM_topic_control::FR_calf_state_callback, this);
    this->joint_sim_unitree_sub[6] = this->nh_.subscribe("/a1_gazebo/RL_hip_controller/state", 2,    &FSM_topic_control::RL_hip_state_callback,  this);
    this->joint_sim_unitree_sub[7] = this->nh_.subscribe("/a1_gazebo/RL_thigh_controller/state", 2,  &FSM_topic_control::RL_thigh_state_callback,this);
    this->joint_sim_unitree_sub[8] = this->nh_.subscribe("/a1_gazebo/RL_calf_controller/state", 2,   &FSM_topic_control::RL_calf_state_callback, this);
    this->joint_sim_unitree_sub[9] = this->nh_.subscribe("/a1_gazebo/RR_hip_controller/state", 2,    &FSM_topic_control::RR_hip_state_callback,  this);
    this->joint_sim_unitree_sub[10]= this->nh_.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &FSM_topic_control::RR_thigh_state_callback,this);
    this->joint_sim_unitree_sub[11]= this->nh_.subscribe("/a1_gazebo/RR_calf_controller/state", 2,  &FSM_topic_control::RR_calf_state_callback, this);

}



void FSM_topic_control::unitree_sim_send_cmd(){

    // send control cmd to robot via ros topic
    for (int i = 0; i < 4; i++) {
        if(1){
            /*! Position Control Without MPC */
                low_cmd.motorCmd[3*i].mode = 0x0A;
                low_cmd.motorCmd[3*i].q = 0;
                low_cmd.motorCmd[3*i].dq = 0;
                low_cmd.motorCmd[3*i].Kp = 70;
                low_cmd.motorCmd[3*i].Kd = 3;
                low_cmd.motorCmd[3*i].tau = 0;

                low_cmd.motorCmd[3*i+1].mode = 0x0A;
                low_cmd.motorCmd[3*i+1].q = 0;
                low_cmd.motorCmd[3*i+1].dq = 0;
                low_cmd.motorCmd[3*i+1].Kp = 180;
                low_cmd.motorCmd[3*i+1].Kd = 8;
                low_cmd.motorCmd[3*i+1].tau = 0;

                low_cmd.motorCmd[3*i+2].mode = 0x0A;
                low_cmd.motorCmd[3*i+2].q = 0;
                low_cmd.motorCmd[3*i+2].dq = 0;
                low_cmd.motorCmd[3*i+2].Kp = 300;
                low_cmd.motorCmd[3*i+2].Kd = 15;
                low_cmd.motorCmd[3*i+2].tau = 0;
        }
        else {
            /*! MPC Torques Control */
            low_cmd.motorCmd[3 * i].mode = 0x0A;
            low_cmd.motorCmd[3 * i].q = 0;
            low_cmd.motorCmd[3 * i].dq = 0;
            low_cmd.motorCmd[3 * i].Kp = 0;
            low_cmd.motorCmd[3 * i].Kd = 0;
            low_cmd.motorCmd[3 * i].tau = 0;

            low_cmd.motorCmd[3 * i + 1].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 1].q = 0;
            low_cmd.motorCmd[3 * i + 1].dq = 0;
            low_cmd.motorCmd[3 * i + 1].Kp = 0;
            low_cmd.motorCmd[3 * i + 1].Kd = 0;
            low_cmd.motorCmd[3 * i + 1].tau = 0;

            low_cmd.motorCmd[3 * i + 2].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 2].q = 0;
            low_cmd.motorCmd[3 * i + 2].dq = 0;
            low_cmd.motorCmd[3 * i + 2].Kp = 0;
            low_cmd.motorCmd[3 * i + 2].Kd = 0;
            low_cmd.motorCmd[3 * i + 2].tau = 0;
        }
    }

    for (int i = 0; i < 12; i++) {
        joint_sim_unitree_pub[i].publish(low_cmd.motorCmd[i]);
    }
}


void FSM_topic_control::FL_hip_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].q = msg.q;
        lowState.motorState[0].dq = msg.dq;
        lowState.motorState[0].tauEst = msg.tauEst;
        std::cout<<0<<std::endl;
        std::cout<<msg<<std::endl;
}
void FSM_topic_control::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].q = msg.q;
        lowState.motorState[1].dq = msg.dq;
        lowState.motorState[1].tauEst = msg.tauEst;
        std::cout<<1<<std::endl;
}
void FSM_topic_control::FL_calf_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].q = msg.q;
        lowState.motorState[2].dq = msg.dq;
        lowState.motorState[2].tauEst = msg.tauEst;
        std::cout<<2<<std::endl;
}

// FR
void FSM_topic_control::FR_hip_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].q = msg.q;
        lowState.motorState[3].dq = msg.dq;
        lowState.motorState[3].tauEst = msg.tauEst;
        std::cout<<3<<std::endl;
}
void FSM_topic_control::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].q = msg.q;
        lowState.motorState[4].dq = msg.dq;
        lowState.motorState[4].tauEst = msg.tauEst;
        std::cout<<4<<std::endl;
}
void FSM_topic_control::FR_calf_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].q = msg.q;
        lowState.motorState[5].dq = msg.dq;
        lowState.motorState[5].tauEst = msg.tauEst;
        std::cout<<5<<std::endl;
}

// RL
void FSM_topic_control::RL_hip_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].q = msg.q;
        lowState.motorState[6].dq = msg.dq;
        lowState.motorState[6].tauEst = msg.tauEst;
        std::cout<<6<<std::endl;
}
void FSM_topic_control::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].q = msg.q;
        lowState.motorState[7].dq = msg.dq;
        lowState.motorState[7].tauEst = msg.tauEst;
        std::cout<<7<<std::endl;
}
void FSM_topic_control::RL_calf_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].q = msg.q;
        lowState.motorState[8].dq = msg.dq;
        lowState.motorState[8].tauEst = msg.tauEst;
        std::cout<<8<<std::endl;
}

// RR
void FSM_topic_control::RR_hip_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].q = msg.q;
        lowState.motorState[9].dq = msg.dq;
        lowState.motorState[9].tauEst = msg.tauEst;
        std::cout<<9<<std::endl;
}
void FSM_topic_control::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].q = msg.q;
        lowState.motorState[10].dq = msg.dq;
        lowState.motorState[10].tauEst = msg.tauEst;
        std::cout<<10<<std::endl;
}
void FSM_topic_control::RR_calf_state_callback(const unitree_legged_msgs::MotorState &msg) {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].q = msg.q;
        lowState.motorState[11].dq = msg.dq;
        lowState.motorState[11].tauEst = msg.tauEst;
        std::cout<<11<<std::endl;
}
















