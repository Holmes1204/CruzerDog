#ifndef _CONVEX_MPC_
#define _CONVEX_MPC_
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/Sparse>
#include <defination.h>
#include <FSM/FSM_data.h>

using namespace quad;

class FSM_data;

class ConvexMPC
{
public:
    /*! Weight Matrix*/
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;
    // Eigen::MatrixXd Q;
    // Eigen::MatrixXd R;
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::SparseMatrix<double> R_sparse;

    /*! Continue Time Dynamics */
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_c;

    /*! Discrete Time Dynamics*/
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;

    /*! QP Formulation */
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> B_qp;
    Eigen::SparseMatrix<double> hessian;                       // p
    Eigen::SparseMatrix<double> linear_constraints;            // Ac
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub;

    ConvexMPC(Eigen::VectorXd &_q_weights, Eigen::VectorXd &_r_weights);
    ~ConvexMPC();
    /*! Init and Reset Matrix to Zero*/
    void Reset();
    /*! Update A_mat_c*/
    void Calculate_A_mat_c(Eigen::Vector3d _rpy);
    /*! Update B_mat_c*/
    void Calculate_B_mat_c(double _robot_mass, const Eigen::Matrix3d &_inertia_matrix, Eigen::Matrix3d _rotate_matrix,
                           Eigen::Matrix<double, 3, NUM_LEG> _foot_position);
    /*! State Space Discrete */
    void State_space_discrete(double _dt);
    /*! Update qp matrix*/
    void Calculate_qp(FSM_data& data);

private:
    /*! constraint of force */
    double fz_min = 0;
    double fz_max = 180;
    double mu = 0.3;
};



class BezierUtils
{
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils()
    {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(double t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle)
    {
        Eigen::Vector3d foot_pos_target;
        // X-axis
        std::vector<double> bezierX{foot_pos_start(0),
                                   foot_pos_start(0),
                                   foot_pos_final(0),
                                   foot_pos_final(0),
                                   foot_pos_final(0)};
        foot_pos_target(0) = bezier_curve(t, bezierX);

        // Y-axis
        std::vector<double> bezierY{foot_pos_start(1),
                                   foot_pos_start(1),
                                   foot_pos_final(1),
                                   foot_pos_final(1),
                                   foot_pos_final(1)};
        foot_pos_target(1) = bezier_curve(t, bezierY);

        // Z-axis
        std::vector<double> bezierZ{foot_pos_start(2),
                                   foot_pos_start(2),
                                   foot_pos_final(2),
                                   foot_pos_final(2),
                                   foot_pos_final(2)};
        bezierZ[1] += FOOT_SWING_CLEARANCE1;
        bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5 * sin(terrain_pitch_angle);
        foot_pos_target(2) = bezier_curve(t, bezierZ);

        return foot_pos_target;
    }

    bool reset_foot_pos_curve() { curve_constructed = false; }

private:
    bool curve_constructed;
    double bezier_degree;

    
    double bezier_curve(double t, const std::vector<double> &P)
    {
        std::vector<double> coefficients{1, 4, 6, 4, 1};
        double y = 0;
        for (int i = 0; i <= bezier_degree; i++)
        {
            y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
        }
        return y;
    }

};


class Solver
{
public:
    OsqpEigen::Solver solver;
    /*! QP Formation */
    Eigen::DiagonalMatrix<double, 6> Q;
    Eigen::Matrix<double, 3, NUM_LEG> last_force;
    BezierUtils bezierUtils[NUM_LEG];

    double R;
    // ground friction coefficient
    double mu;
    double F_min;
    double F_max;
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    // MPC does not start for the first 10 ticks to prevent uninitialized NAN goes into joint_torques
    int mpc_init_counter;

    Solver();
    ~Solver();
    void InitParams();
    /*! For Testing */
    void update_plan(FSM_data& data, double dt);
    void generate_swing_legs_ctrl(FSM_data &state, double dt);
    /*! Use contact force to Calculate joint torques */
    Eigen::Matrix<double, NUM_DOF, 1> Calculate_joint_torques(FSM_data& data);
    /*! Contact force -> [fx fy fz] * 4 */
    Eigen::Matrix<double, 3, NUM_LEG> Calculate_contact_force(FSM_data& data, double dt);
};



#endif //
