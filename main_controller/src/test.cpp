#include <ros/ros.h>
#include <Convex_MPC/ConvexMPC.h>
//test
int main(int argc, char** argv){

    FSM_data data;
    data.mpc_solver->Calculate_contact_force(data,data.state->plan_dt);
    data.mpc_solver->Calculate_joint_torques(data);

    return 0;
}