#include <ros/ros.h>
#include <FSM/FiniteStateMachine.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(400);//Hz
    FSM Finite_State_Machine(nh);
    //ros::AsyncSpinner spinner(12);
    Finite_State_Machine.build_ScheduleTable(
            // quad::STAND,
            quad::LOCOMOTION,
//            quad::WALK,
//            quad::TROT,
//           quad::PACE,
//           quad::GALLOP,
            quad::END
            );
    //spinner.start();
    //控制频率400hz
    while (ros::ok()) {
        rate.sleep();
        Finite_State_Machine.loop();
        ros::spinOnce();
    }

    return 0;
}