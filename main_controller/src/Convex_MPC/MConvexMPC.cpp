#include <Convex_MPC/MConvexMPC.h>

void MPC_SLOVER::initialize()
{
    //各个矩阵resize
    A_qp.resize(12 * horizon, 12);
    B_qp.resize(12 * horizon, 12 * horizon);
    G_qp.resize(12 * horizon, 1);
    X_d.resize(12 * horizon);
    U_b.resize(20 * horizon);

    H.resize(12 * horizon, 12 * horizon);
    g.resize(12 * horizon);
    // C_sparse.resize(20 * horizon, 12 * horizon);
    C.resize(20 * horizon, 12 * horizon);
    lowerC.resize(20 * horizon);
    upperC.resize(20 * horizon);

    K.resize(12 * horizon, 12 * horizon);
    L.resize(12 * horizon, 12 * horizon);

    A_qp.setZero();
    B_qp.setZero();
    G_qp.setZero();
    X_d.setZero();
    U_b.setZero();
    H.setZero();
    g.setZero();
    C.setZero();
    lowerC.setZero();
    upperC.setZero();

    Matrix<double, 5, 3> f_block;
    f_block << muinv, 0, 1.f,
        -muinv, 0, 1.f,
        0, muinv, 0,
        0, -muinv, 0,
        0, 0, 1.f;

    for (uint16_t i = 0; i < horizon * 4; i++)
    {
        C.block(i * 5, i * 3, 5, 3) = f_block;
    }

    C_sparse = C.sparseView();

    // init K
    for (int i = 0; i < 12 * horizon; i++)
    {
        K.insert(i, i) = alpha;
    }
    // init L
    for (int i = 0; i < 12 * horizon; i++)
    {
        L.insert(i, i) = L_weights(i % 12);
    }
}

// continuous time state space matrices.
void MPC_SLOVER::ct_ss_mats()
{
    Act.setZero();
    Act(3, 9) = 1.f;
    Act(4, 10) = 1.f;
    Act(5, 11) = 1.f;
    Act.block(0, 6, 3, 3) = R_yaw.transpose();
    Bct.setZero();
    Matrix<double, 3, 3> I_inv = I_world.inverse();
    for (uint16_t b = 0; b < 4; b++)
    {
        Bct.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet_world.col(b));
        Bct.block(9, b * 3, 3, 3) = Matrix<double, 3, 3>::Identity() / mass;
    }
}

void MPC_SLOVER::c2qp()
{
    ABc.setZero();
    ABc.block(0, 0, 12, 12) = Act;
    ABc.block(0, 12, 12, 12) = Bct;
    ABc = dtMPC * ABc;
    expmm = ABc.exp();

    Adt = expmm.block(0, 0, 12, 12);
    Bdt = expmm.block(0, 12, 12, 12);
    Gdt.setZero();
    Gdt(11) = -9.8 * dtMPC;

    Matrix<double, 12, 12> powerMats[20];
    powerMats[0].setIdentity();

    for (int i = 1; i < horizon + 1; i++)
    {
        powerMats[i] = Adt * powerMats[i - 1];
    }
    for (uint16_t r = 0; r < horizon; r++)
    {
        A_qp.block(12 * r, 0, 12, 12) = powerMats[r + 1]; // Adt.pow(r+1);
        for (uint16_t c = 0; c <= r; c++)
        {
            B_qp.block(12 * r, 12 * c, 12, 12) = powerMats[r - c] * Bdt;
        }
        G_qp.segment(12 * r, 12) = powerMats[r] * Gdt;
    }
#ifdef DBG
    std::cout << "A_c" << std::endl
              << Ac << std::endl;
    std::cout << "B_c" << std::endl
              << Bc << std::endl;
    std::cout << "A_dt" << std::endl
              << Adt << std::endl;
    std::cout << "B_dt" << std::endl
              << Bdt << std::endl;
#endif
}

void MPC_SLOVER::solve_mpc(double phi, Vec12<double> &x_0_, Mat34<double> &r_feet_w_, int *contact_state, double *traj, Vec12<double> &force_)
{
    // input initial state
    x_0 = x_0_;
    r_feet_world = r_feet_w_;
    R_yaw << std::cos(phi), -std::sin(phi), 0,
        std::sin(phi), std::cos(phi), 0,
        0, 0, 1;
    // expected trajectory
    for (int i = 0; i < horizon * 12; i++)
    {
        X_d(i) = traj[i];
    }
    // update
    I_world = R_yaw * I_body * R_yaw.transpose();
    ct_ss_mats();
    c2qp();
    for (uint16_t i = 0; i < horizon; i++)
    {
        for (int leg = 0; leg < 4; leg++)
        {
            lowerC.segment(i * 20 + leg * 5, 5) << 0, 0, 0, 0, 0;
            upperC.segment(i * 20 + leg * 5, 5) << INFTY, INFTY, 0, 0, 300 * contact_state[4 * horizon + leg];
        }
    }
    H = 2 * (B_qp.transpose() * L * B_qp + K);                // matrix
    g = 2 * B_qp.transpose() * L * (A_qp * x_0 + G_qp - X_d); // vector
    // instantiate the solver
    if (!solver.isInitialized())
    {
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(NumberOfVariables);     //设置A矩阵的列数，即n
        solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
        solver.data()->setHessianMatrix(H);
        solver.data()->setGradient(g);
        solver.data()->setLowerBound(lowerC);
        solver.data()->setUpperBound(upperC);
        solver.data()->setLinearConstraintsMatrix(C_sparse);
        solver.initSolver();
    }
    else
    {
        solver.updateHessianMatrix(H);
        solver.updateGradient(g);
        solver.updateLowerBound(lowerC);
        solver.updateUpperBound(upperC);
    }
    // solve the QP problem
    solver.solveProblem();
    // get the controller input
    QPSolution = solver.getSolution();
    force_ = QPSolution.segment(0, 12);
    // // debug
    // {

    //     static uint32_t dbg_count = 0;
    //     static int i = 0;
    //     if (dbg_count % 20 == 0)
    //     {
    //         system("clear");
    //         std::cout
    //             << "foot_force" << std::endl
    //             << force_.segment.transpose() << std::endl
    //             << std::endl;
    //         dbg_count = 0;
    //     }
    //     dbg_count++;
    // }
}
