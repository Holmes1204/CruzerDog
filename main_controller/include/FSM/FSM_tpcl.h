#ifndef _FSM_TPCL_
#define _FSM_TPCL_

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"

#include "FSM_data.h"

#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"

#include "wtr_serial_msg/em_ev.h"
#include "wtr_serial_msg/em_fb_raw.h"

#include "Math_tools/orientation_tools.h"

//控制所有的收发，以及将电机反馈解算反馈到leg Controller中
class FSM_topic_control
{
private:
public:
    FSM_data &data_;
    ros::NodeHandle nh_;
    FSM_topic_control(FSM_data &data);
    ~FSM_topic_control();
    // joy
    ros::Subscriber joy_sub_;
    // unitree sim
    ros::Publisher joint_sim_unitree_pub[8];
    ros::Subscriber joint_sim_unitree_sub[8];
    ros::Subscriber footForce_sim_unitree_sub[4];
    ros::Subscriber imu_sim_unitree_sub;
    ros::Subscriber cheater_estimator_sub;
    // msg_type for unitree sim
    unitree_legged_msgs::LowCmd low_cmd;
    unitree_legged_msgs::LowState low_state;

    void unitree_sim_set_up();
    void unitree_sim_send_cmd();
    void unitree_sim_data_decode();
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg);
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &msg);
    void cheater_estimator_callback(const gazebo_msgs::ModelStates &msg);
    void Imu_Callback(const sensor_msgs::Imu &msg);
    void FRfootCallback(const geometry_msgs::WrenchStamped &msg);
    void FLfootCallback(const geometry_msgs::WrenchStamped &msg);
    void RRfootCallback(const geometry_msgs::WrenchStamped &msg);
    void RLfootCallback(const geometry_msgs::WrenchStamped &msg);

    //
    void Joy_Callback(const sensor_msgs::Joy &msg);
    // real hardware
    ros::Publisher em_cmd_pub_;
    ros::Subscriber em_fdb_sub_;
    ros::Subscriber imu_real_sub_;
    // msg_type for hardware
    wtr_serial_msg::em_ev mt_cmd;
    wtr_serial_msg::em_fb_raw mt_fdb;
    void em_cmd_send();
    void em_fdb_callback(const wtr_serial_msg::em_fb_raw &msg);
};

#endif //