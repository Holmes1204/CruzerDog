#ifndef _FSM_TPCL_
#define _FSM_TPCL_

#include "ros/ros.h"
#include "FSM_data.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"


class FSM_topic_control
{
private:
public:
    FSM_data& data_;
    ros::NodeHandle& nh_;
    FSM_topic_control(ros::NodeHandle &nh,FSM_data &data);
    ~FSM_topic_control();

    //unitree sim
    ros::Publisher joint_sim_unitree_pub[12];
    ros::Subscriber joint_sim_unitree_sub[12];
    unitree_legged_msgs::LowCmd low_cmd;
    unitree_legged_msgs::LowState lowState;
    void unitree_sim_set_up();
    void unitree_sim_send_cmd();
    //imu_callback
    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &msg);

    //real 
    ros::Publisher em_cmd_pub_;
    ros::Subscriber em_fdb_sub_;
    void em_cmd_send();

};


#endif//