#include <Convex_MPC/ConvexMPC.h>

Eigen::Matrix3d skew(Eigen::Vector3d vec);

ConvexMPC::ConvexMPC(Eigen::VectorXd &_q_weights, Eigen::VectorXd &_r_weights)
{
    this->mu = 0.3;
    // reserve size for sparse matrix
    this->Q_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * PLAN_HORIZON,
                                                 MPC_STATE_DIM * PLAN_HORIZON);
    this->R_sparse = Eigen::SparseMatrix<double>(NUM_DOF * PLAN_HORIZON,
                                                 NUM_DOF * PLAN_HORIZON);
    this->q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    this->r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);

    // init Q
    for (int i = 0; i < PLAN_HORIZON; i++)
    {
        this->q_weights_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = _q_weights;
    }
    /*! why should multiply 2 ?*/
    this->Q.diagonal() = 2 * this->q_weights_mpc;
    for (int i = 0; i < MPC_STATE_DIM * PLAN_HORIZON; i++)
    {
        this->Q_sparse.insert(i, i) = 2 * this->q_weights_mpc(i);
    }

    // init R
    for (int i = 0; i < PLAN_HORIZON; i++)
    {
        this->r_weights_mpc.segment(i * NUM_DOF, NUM_DOF) = _r_weights;
    }
    this->R.diagonal() = 2 * this->r_weights_mpc;
    for (int i = 0; i < PLAN_HORIZON * NUM_DOF; i++)
    {
        this->R_sparse.insert(i, i) = 2 * this->r_weights_mpc(i);
    }

    this->linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    // init linear constraints
    for (int i = 0; i < NUM_LEG * PLAN_HORIZON; i++)
    {
        this->linear_constraints.insert(0 + 5 * i, 0 + 3 * i) = 1;
        this->linear_constraints.insert(1 + 5 * i, 0 + 3 * i) = 1;
        this->linear_constraints.insert(2 + 5 * i, 1 + 3 * i) = 1;
        this->linear_constraints.insert(3 + 5 * i, 1 + 3 * i) = 1;
        this->linear_constraints.insert(4 + 5 * i, 2 + 3 * i) = 1;
        this->linear_constraints.insert(0 + 5 * i, 2 + 3 * i) = this->mu;
        this->linear_constraints.insert(1 + 5 * i, 2 + 3 * i) = this->mu * (-1);
        this->linear_constraints.insert(2 + 5 * i, 2 + 3 * i) = this->mu;
        this->linear_constraints.insert(3 + 5 * i, 2 + 3 * i) = this->mu * (-1);
    }
}

ConvexMPC::~ConvexMPC()
{
    
}

/*! Update A_mat_c*/
void ConvexMPC::Calculate_A_mat_c(Eigen::Vector3d _rpy)
{
    double cos_yaw = cos(_rpy.z());
    double sin_yaw = sin(_rpy.z());

    Eigen::Matrix3d angle_vel_to_rpy_rate;
    angle_vel_to_rpy_rate << cos_yaw, sin_yaw, 0,
        -sin_yaw, cos_yaw, 0,
        0, 0, 1;

    this->A_mat_c.block<3, 3>(0, 6) = angle_vel_to_rpy_rate;
    this->A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    this->A_mat_c(11, NUM_DOF) = 1;
}

/*! Update B_mat_c*/
void ConvexMPC::Calculate_B_mat_c(double _robot_mass, const Eigen::Matrix3d &_inertia_matrix, Eigen::Matrix3d _rotate_matrix,
                                  Eigen::Matrix<double, 3, NUM_LEG> _foot_position)
{
    Eigen::Matrix3d inertia_matrix_world;
    inertia_matrix_world = _rotate_matrix * _inertia_matrix * _rotate_matrix.transpose();
    for (int i = 0; i < NUM_LEG; i++)
    {
        this->B_mat_c.block<3, 3>(6, 3 * i) =
            inertia_matrix_world.inverse() * skew(_foot_position.block<3, 1>(0, i));
        this->B_mat_c.block<3, 3>(9, 3 * i) =
            (1 / _robot_mass) * Eigen::Matrix3d::Identity();
    }
}

/*! State Space Discrete */
//backwrad derivative
void ConvexMPC::State_space_discrete(double _dt)
{
    this->A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() +
                    this->A_mat_c * _dt;
    this->B_mat_d = this->B_mat_c * _dt;
}

/*! Update qp matrix*/
void ConvexMPC::Calculate_qp(FSM_data &data)
{
    // calculate A_qp and B_qp
    for (int i = 0; i < PLAN_HORIZON; ++i)
    {
        if (i == 0)
        {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
        }
        else
        {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) =
                A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i - 1), 0) * A_mat_d;
        }
        for (int j = 0; j < i + 1; ++j)
        {
            if (i - j == 0)
            {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                    B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            }
            else
            {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                    A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i - j - 1), 0) * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            }
        }
    }

    // transform to QP Problems
    // Calculate Hessian
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> dense_hessian;
    dense_hessian = (this->B_qp.transpose() * this->Q * this->B_qp);
    dense_hessian += this->R;
    this->hessian = dense_hessian.sparseView();
    // calculate gradient
    Eigen::Matrix<double, 13 * PLAN_HORIZON, 1> tmp_vec = this->A_qp * data.state->mpc_states;
    tmp_vec -= data.state->mpc_states_list;
    this->gradient = this->B_qp.transpose() * this->Q * tmp_vec;
    this->fz_min = 0;
    this->fz_max = 180;

    // Calculate lower bound and upper bound
    Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
    for (int i = 0; i < NUM_LEG; i++)
    {
        lb_one_horizon.segment<5>(i * 5) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, fz_min * data.state->contacts[i];
        ub_one_horizon.segment<5>(i * 5) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, fz_max * data.state->contacts[i];
    }
    for (int i = 0; i < PLAN_HORIZON; i++)
    {
        lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
        ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
    }
}

void ConvexMPC::Reset()
{
    A_mat_c.setZero();
    B_mat_c.setZero();
    AB_mat_c.setZero();
    A_mat_d.setZero();
    B_mat_d.setZero();
    AB_mat_d.setZero();
    B_mat_d_list.setZero();

    A_qp.setZero();
    B_qp.setZero();

    gradient.setZero();
    lb.setZero();
    ub.setZero();
}

// solver

Solver::Solver()
{
    InitParams();
}

Solver::~Solver()
{
}

void Solver::InitParams()
{
    ROS_INFO("[MPC SOLVER] Init QP Solver, MPC Parameters");
    Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
    R = 1e-3;
    mu = 0.7;
    F_min = 0;
    F_max = 180;
    last_force.setZero();

    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // init mpc skip counter
    mpc_init_counter = 0;
    // init constraints matrix
    for (int i = 0; i < NUM_LEG; i++)
    {
        // extract F_zi
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
        // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        // 3. F_yi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        // 4. -F_yi > uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
    }
}

/*! For Testing */
void Solver::update_plan(FSM_data &data, double dt)
{
    data.state->counter += 1;
    if (1)
    {
        // movement_mode == 0, standstill with all feet in contact with ground
        for (bool &plan_contact : data.state->plan_contacts)
            plan_contact = true;
        for (int i = 0; i < NUM_LEG; ++i)
        {
            data.state->contacts[i] = 1;
        }

        data.state->gait_counter_reset();
    }
    else
    {
        // movement_mode == 1, walk
        for (int i = 0; i < NUM_LEG; ++i)
        {
            data.state->gait_counter(i) = data.state->gait_counter(i) + data.state->gait_counter_speed(i);
            data.state->gait_counter(i) = std::fmod(data.state->gait_counter(i), data.state->counter_per_gait);
            if (data.state->gait_counter(i) <= data.state->counter_per_swing)
            {
                data.state->contacts[i] = 1;
            }
            else
            {
                data.state->contacts[i] = 0;
            }
        }
    }

    // update foot plan: data.state->foot_pos_target_world
    Eigen::Vector3d lin_vel_world = data.state->cur_vel;                                 // world frame linear velocity
    Eigen::Vector3d lin_vel_rel = data.state->rotate_matrix.transpose() * lin_vel_world; // robot body frame linear velocity

    // Raibert Heuristic, calculate foothold position
    data.state->foot_pDes_robot = data.state->foot_p_bias;
    for (int i = 0; i < NUM_LEG; ++i)
    {
        double delta_x =
            std::sqrt(std::abs(data.state->foot_p_bias(2)) / 9.8) * (lin_vel_rel(0) - data.state->command_vel(0)) +
            ((data.state->counter_per_swing / data.state->gait_counter_speed(i)) * data.state->plan_dt) / 2.0 *
                data.state->command_vel(0);
        double delta_y =
            std::sqrt(std::abs(data.state->foot_p_bias(2)) / 9.8) * (lin_vel_rel(1) - data.state->command_vel(1)) +
            ((data.state->counter_per_swing / data.state->gait_counter_speed(i)) * data.state->plan_dt) / 2.0 *
                data.state->command_vel(1);

        if (delta_x < -FOOT_DELTA_X_LIMIT)
        {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT)
        {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT)
        {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT)
        {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        data.state->foot_pDes_robot(0, i) += delta_x;
        data.state->foot_pDes_robot(1, i) += delta_y;

        data.state->foot_pDes_abs.block<3, 1>(0, i) = data.state->rotate_matrix * data.state->foot_pDes_robot.block<3, 1>(0, i);
        data.state->foot_pDes.block<3, 1>(0, i) = data.state->foot_pDes_abs.block<3, 1>(0, i) + data.state->cur_position;
    }
}
void Solver::generate_swing_legs_ctrl(FSM_data &data, double dt)
{
    data.state->joint_torques.setZero();

    // get current foot pos and target foot pose
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<double, 1, NUM_LEG> spline_time;
    spline_time.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    foot_vel_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

    for (int i = 0; i < NUM_LEG; ++i)
    {
        foot_pos_cur.block<3, 1>(0, i) = data.state->foot_p_robot.block<3, 1>(0, i);

        // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils
        if (data.state->gait_counter(i) <= data.state->counter_per_swing)
        {
            // stance foot
            spline_time(i) = 0.0;
            // in this case the foot should be stance
            // keep refreshing foot_pos_start in stance mode
            data.state->foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        }
        else
        {
            // in this case the foot should be swing
            spline_time(i) = double(data.state->gait_counter(i) - data.state->counter_per_swing) / double(data.state->counter_per_swing);
        }

        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              data.state->foot_pos_start.block<3, 1>(0, i),
                                                                              data.state->foot_pDes_robot.block<3, 1>(0, i),
                                                                              0.0);

        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - data.state->foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        data.state->foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - data.state->foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        data.state->foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        data.state->foot_forces_swing.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(data.state->kp_foot.block<3, 1>(0, i)) +
                                                          foot_vel_error.block<3, 1>(0, i).cwiseProduct(data.state->kd_foot.block<3, 1>(0, i));
    }
}

/*! Use contact force to Calculate joint torques */
Eigen::Matrix<double, NUM_DOF, 1> Solver::Calculate_joint_torques(FSM_data &data)
{
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

    joint_torques.setZero();
    mpc_init_counter++;
    // for the first 10 ticks, just return zero torques.
    if (mpc_init_counter < 10)
    {
        data.state->joint_torques = joint_torques;
    }
    else
    {
        // for each leg, if it is a swing leg (contact[i] is false), use foot_force_swing getting joint_torque
        // for each leg, if it is a stance leg (contact[i] is true), use foot_contact_force getting joint_torque
        for (int i = 0; i < NUM_LEG; i++)
        {
            Eigen::Matrix3d jac = data.state->foot_jacobian.block<3, 3>(3 * i, 3 * i);
            if (data.state->contacts[i] == 1)
            {
                // stance
                joint_torques.segment<3>(i * 3) = jac.transpose() * -data.state->foot_contact_force.block<3, 1>(0, i);
            }
            else
            {
                // swing
                Eigen::Vector3d force_tgt = data.state->km_foot.cwiseProduct(
                    data.state->foot_forces_swing.block<3, 1>(0, i));
                // X = A.lu().solve(b)
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt); // LU分解, jac * tau = F
            }
        }
        // add gravity compensation
        joint_torques += data.state->torques_gravity;

        // prevent nan
        for (int i = 0; i < 12; i++)
        {
            if (!isnan(joint_torques[i]))
                return joint_torques;
        }
    }
}

/*! Contact force -> [fx fy fz] * 4 */
Eigen::Matrix<double, 3, NUM_LEG> Solver::Calculate_contact_force(FSM_data &data, double dt)
{

    Eigen::Matrix<double, 3, NUM_LEG> foot_contact_force;
    Eigen::Vector3d rpy_error = data.state->command_rpy - data.state->cur_rpy;

    // limit euler error to pi/2
    if (rpy_error(2) > 3.1415926 * 1.5)
    {
        rpy_error(2) = data.state->command_rpy(2) - 3.1415926 * 2 - data.state->cur_rpy(2);
    }
    else if (rpy_error(2) < -3.1415926 * 1.5)
    {
        rpy_error(2) = data.state->command_rpy(2) + 3.1415926 * 2 - data.state->cur_rpy(2);
    }

    //leg_control_type 0 : 不使用MPC，1：使用MPC
    if (data.state->leg_control_type == 0)
    {
        // no use
    }
    else if (data.state->leg_control_type == 1)
    {
        ConvexMPC mpc_solver = ConvexMPC(data.state->q_weights, data.state->r_weights);
        mpc_solver.Reset();

        // initialize the mpc data.state at the first time step，x[0]
        // data.state->mpc_states.resize(13);
        data.state->mpc_states << data.state->cur_rpy.x(), data.state->cur_rpy.y(), data.state->cur_rpy.z(),
            data.state->cur_position.x(), data.state->cur_position.y(), data.state->cur_position.z(),
            data.state->w_angle_vel.x(), data.state->w_angle_vel.y(), data.state->w_angle_vel.z(),
            data.state->cur_vel.x(), data.state->cur_vel.y(), data.state->cur_vel.z(),
            -9.8;
        std::cout << data.state->mpc_states << std::endl;
        //  this should be roughly close to the average dt of main controller
        double mpc_dt;
        // if in gazebo
        mpc_dt = dt;
        
        // initialize the desired mpc states trajectory
        for (int i = 0; i < PLAN_HORIZON; i++)
        {
            data.state->mpc_states_list.segment(i * 13, 13) << data.state->command_rpy.x(),
                data.state->command_rpy.y(),
                data.state->cur_rpy.z() + data.state->command_angle_vel.z() * mpc_dt * (i + 1),
                data.state->cur_position.x() + data.state->command_vel.x() * mpc_dt * (i + 1),
                data.state->cur_position.y() + data.state->command_vel.y() * mpc_dt * (i + 1),
                data.state->command_position.z(),
                data.state->command_angle_vel.x(),
                data.state->command_angle_vel.y(),
                data.state->command_angle_vel.z(),
                data.state->command_vel.x(),
                data.state->command_vel.y(),
                0,
                -9.8;
        }

        // a single A_mat_c is computed for the entire reference trajectory
        mpc_solver.Calculate_A_mat_c(data.state->cur_rpy);
        //std::cout<<mpc_solver.A_mat_c<<std::endl;
        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller
        for (int i = 0; i < PLAN_HORIZON; i++)
        {

            // Calculate current B_mat_c Matrix
            mpc_solver.Calculate_B_mat_c(data.state->robot_mass,
                                         data.state->inertia_tensor,
                                         data.state->rotate_matrix,
                                         data.state->foot_p_abs);
            //std::cout<<mpc_solver.A_mat_c<<std::endl;
            // data.state space Discretization
            mpc_solver.State_space_discrete(mpc_dt);

            // store current B_d Matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }
        // Calculate QP Matrix
        mpc_solver.Calculate_qp(data);

        // Solve the QP Problem
        if (!solver.isInitialized())
        {
            // Setting
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            // Set the initial data of the QP solver
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);

            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);

            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);

            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            //std::cout<<"constraint"<<std::endl<<mpc_solver.linear_constraints<<std::endl;
            // std::cout<<"Hessian"<<std::endl<<mpc_solver.hessian<<std::endl;
            // std::cout<<"gradient"<<std::endl<<mpc_solver.gradient<<std::endl;
            //std::cout<<"lb"<<std::endl<<mpc_solver.lb.transpose()<<std::endl;
            //std::cout<<"ub"<<std::endl<<mpc_solver.ub.transpose()<<std::endl;
            solver.initSolver();
            printf("initialize QP !\n");
        }
        else
        {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }

        // solver.solve();
        solver.solveProblem();
        
        // Get Solution
        Eigen::VectorXd solution = solver.getSolution();

        for (int i = 0; i < NUM_LEG; i++)
        {
            if (!isnan(solution.segment<3>(i * 3).norm()))
            {
                foot_contact_force.block<3, 1>(0, i) = data.state->rotate_matrix.transpose() * solution.segment<3>(i * 3);
            }
        }
        //            std::cout << data.state->foot_contact_force << std::endl;
    }

    return foot_contact_force;
}
