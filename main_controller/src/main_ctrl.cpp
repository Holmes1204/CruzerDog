#include <ros/ros.h>
#include <FSM/FiniteStateMachine.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(400);//Hz
    FSM Finite_State_Machine(nh);
    //ros::AsyncSpinner spinner(12);
    Finite_State_Machine.build_ScheduleTable(
            quad::STEADY,
            quad::STAND,
//            quad::WALK,
//            quad::TROT,
//           quad::PACE,
//           quad::GALLOP,
            quad::END
            );
    //spinner.start();
    while (ros::ok()) {
        Finite_State_Machine.loop();
        ros::spinOnce();
        rate.sleep();
    }
    //控制频率400hz
    return 0;
}