#include <ros/ros.h>
#include <FSM/FiniteStateMachine.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(1000); // Hz
    FSM Finite_State_Machine(nh);
    // ros::AsyncSpinner spinner(12);
    Finite_State_Machine.build_ScheduleTable(
        quad::STAND,
        quad::LOCOMOTION,
        //            quad::WALK,
        //            quad::TROT,
        //           quad::PACE,
        //           quad::GALLOP,
        quad::END);
    // spinner.start();
    //控制频率400hz
    const uint32_t horizon = 5;
    int t[4 * horizon];
    double traj[12 * horizon];
    Eigen::Vector3d foot[4];
    while (ros::ok())
    {
        //Finite_State_Machine.global_data.mpc_solver->solve_mpc(t,traj);
        // rate.sleep();
        Finite_State_Machine.loop();
        // ros::spinOnce();
    }

    return 0;
}