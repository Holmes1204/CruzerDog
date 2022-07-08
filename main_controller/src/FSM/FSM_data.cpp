#include <FSM/FSM_data.h>

FSM_data::FSM_data()
{
    state = new state_data();
    state->InitParams();
    state->Reset();
    model_StateEstimate = new A1BasicEKF();
    mpc_solver = new Solver();
    model_LegController = new LegController();
    _quadruped  = new Robot();
    _legController = new LegController();
}

FSM_data::~FSM_data()
{


}

state_data::state_data()
{
    Reset();
    InitParams();
}

state_data::~state_data()
{


}

void state_data::Reset()
{
    /*! For test parameters */
    counter_per_gait = 240;
    counter_per_swing = 120;
    counter = 0;
    gait_counter.setZero();
    foot_pos_start.setZero();
    foot_pos_rel_last_time.setZero();
    foot_pos_target_last_time.setZero();
    gait_counter_reset();
    // gait_counter_speed << 1.5, 1.5, 1.5, 1.5 ;
    gait_counter_speed << 2, 2, 2, 2;

    robot_mass = 12.0;
    inertia_tensor << 0.0158533, 0.0, 0.0,
        0.0, 0.0377999, 0.0,
        0.0, 0.0, 0.0456542;

    gait_type = STAND;
    leg_control_type = 1;

    plan_dt = 0.0025;

    command_position << 0, 0, 0.32;
    command_vel.setZero();
    command_vel_b.setZero();
    command_angle_vel.setZero();
    command_rpy.setZero();

    cur_position.setZero();
    cur_vel.setZero();
    cur_vel_b.setZero();
    b_angle_vel.setZero();
    w_angle_vel.setZero();
    cur_rpy.setZero();
    b_acc.setZero();

    quaternion.setIdentity();
    rotate_matrix.setZero();
    rotate_matrix_z.setZero();

    foot_force.setZero();
    foot_contact_force.setZero();
    foot_forces_swing.setZero();
    foot_p.setZero();
    foot_p_bias.setZero();
    foot_p_robot.setZero();
    foot_p_abs.setZero();
    foot_v.setZero();
    foot_v_robot.setZero();
    foot_v_abs.setZero();
    foot_q.setZero();
    foot_qd.setZero();
    foot_jacobian.setZero();
    foot_jacobian_inv.setZero();

    foot_pDes.setZero();
    foot_pDes_robot.setZero();
    foot_pDes_abs.setZero();
    foot_vDes.setZero();
    foot_qDes.setZero();
    foot_qdDes.setZero();

    for (int i = 0; i < NUM_LEG; i++)
    {
        contacts[i] = 0;
        plan_contacts[i] = false;
        early_contacts[i] = false;
        estimate_contacts[i] = false;
    }
    phase_variable.setZero();

    q_weights.resize(MPC_STATE_DIM);
    r_weights.resize(12);
    q_weights << 20.0, 10.0, 1.0,
        0.0, 0.0, 420.0,
        0.05, 0.05, 0.05,
        30.0, 30.0, 10.0,
        0.0;
    r_weights << 1e-7, 1e-7, 1e-7,
        1e-7, 1e-7, 1e-7,
        1e-7, 1e-7, 1e-7,
        1e-7, 1e-7, 1e-7;
    mpc_states.setZero();
    mpc_states_list.setZero();

    estimate_position.setZero();
    estimate_vel.setZero();

    joint_position.setZero();
    joint_velocity.setZero();
    joint_torques.setZero();

    /*! Init PD controller Parameters, Why ?*/
    double kp_foot_x = 200.0;
    double kp_foot_y = 200.0;
    double kp_foot_z = 150.0;
    double kd_foot_x = 10.0;
    double kd_foot_y = 10.0;
    double kd_foot_z = 5.0;
    kp_foot << kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
        kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
        kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
    kd_foot << kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
        kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
        kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;
    km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

    kp_linear = Eigen::Vector3d(120.0, 120.0, 500.0);
    kd_linear = Eigen::Vector3d(70.0, 70.0, 120.0);
    kp_angular = Eigen::Vector3d(250.0, 35.0, 1.0);
    kd_angular = Eigen::Vector3d(1.5, 1.5, 30.0);

    torques_gravity << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;
}

void state_data::InitParams()
{
    for (int foot = 0; foot < NUM_LEG; foot++)
    {
        if (foot == 0 || foot == 1)
        {
            foot_p_bias.block<3, 1>(0, foot).x() = X_OFFSET;
        }
        else
        {
            foot_p_bias.block<3, 1>(0, foot).x() = -X_OFFSET;
        }
        if (foot == 0 || foot == 2)
        {
            foot_p_bias.block<3, 1>(0, foot).y() = Y_OFFSET;
        }
        else
        {
            foot_p_bias.block<3, 1>(0, foot).y() = -Y_OFFSET;
        }
        foot_p_bias.block<3, 1>(0, foot).z() = -0.33;
    }
}

void state_data::gait_counter_reset()
{
    gait_counter << 0, 120, 120, 0;
}
