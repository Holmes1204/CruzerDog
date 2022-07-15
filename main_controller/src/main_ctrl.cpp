#include <ros/ros.h>
#include <FSM/FSM_ctrl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_ctrl");
    FSM_ctrl FSM;
    ros::Rate rate(quad::df); // Hz
    // spinner.start();
    //控制频率500hz
    while (ros::ok())
    {
        ros::spinOnce();
        FSM.runFSM();
        //考虑到unitree topic的特殊情况，不仿直接将要更新的部分放在主循环
        FSM.tpcl_.em_cmd_send();
        // FSM.tpcl_.unitree_sim_send_cmd();
        rate.sleep();
    }

    return 0;
}