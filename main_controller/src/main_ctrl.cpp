#include <ros/ros.h>
#include <FSM/FiniteStateMachine.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(100); // Hz
    FSM Finite_State_Machine(nh);
    // ros::AsyncSpinner spinner(12);
    Finite_State_Machine.build_ScheduleTable(
        quad::STAND,
        quad::LOCOMOTION,
        quad::END);
    // spinner.start();
    //控制频率400hz
    while (ros::ok())
    {
        ros::spinOnce();
        //考虑到unitree topic的特殊情况，不仿直接将要更新的部分放在主循环
        Finite_State_Machine.topic_contrl.unitree_sim_data_decode();
        Finite_State_Machine.loop();
        rate.sleep();
    }

    return 0;
}