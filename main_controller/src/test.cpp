#include <ros/ros.h>
#include <Convex_MPC/MConvexMPC.h>

// test
int main(int argc, char **argv)
{
    const uint32_t horizon = 5;
    int t[4 * horizon];
    double traj[12 * horizon];
    Eigen::Vector3d foot[4];
    MPC_SLOVER test(horizon);

    test.solve_mpc(foot, t, traj);
    return 0;
}