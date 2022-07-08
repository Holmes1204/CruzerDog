#ifndef _FSM_DATA_
#define _FSM_DATA_
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <defination.h>
#include <Robot/Robot.h>
#include <Convex_MPC/ConvexMPC.h>
#include "Leg_Control/Leg_Control.h"
#include <State_Estimator/A1BasicEKF.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>

using namespace quad;

class state_data;
class FSM_data;
class A1BasicEKF;
class Solver;
class Robot;

class state_data
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*! For test parameters */
    double counter_per_gait;
    double counter_per_swing;
    int counter;
    
    Eigen::Vector4f gait_counter;
    Eigen::Vector4f gait_counter_speed;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;

    /*! interior params */
    double robot_mass;
    Eigen::Matrix3d inertia_tensor;

    /*! Gait Type*/
    STATE_TYPE gait_type;
    int leg_control_type;

    /*! Command from Joy */
    Eigen::Vector3d command_position;
    Eigen::Vector3d command_vel;
    Eigen::Vector3d command_vel_b;
    Eigen::Vector3d command_angle_vel;
    Eigen::Vector3d command_rpy;

    /*! Current State */
    Eigen::Vector3d cur_position;
    Eigen::Vector3d cur_vel;
    Eigen::Vector3d cur_vel_b;
    Eigen::Vector3d b_angle_vel; // in robot frame
    Eigen::Vector3d w_angle_vel; // in world frame
    Eigen::Vector3d cur_rpy;
    Eigen::Vector3d b_acc; // in robot frame

    /*! Kinematic Parameters */
    Eigen::Quaterniond quaternion;
    Eigen::Matrix3d rotate_matrix;
    Eigen::Matrix3d rotate_matrix_z;

    /*! Leg Controller Parameters */
    
    Eigen::Vector4f foot_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
    Eigen::Matrix<double, 3, NUM_LEG> foot_contact_force;
    // Data
    Eigen::Matrix<double, 3, NUM_LEG> foot_p;       // in world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_p_robot; // in robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_p_abs;   // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_p_bias;
    Eigen::Matrix<double, 3, NUM_LEG> foot_v;
    Eigen::Matrix<double, 3, NUM_LEG> foot_v_robot;
    Eigen::Matrix<double, 3, NUM_LEG> foot_v_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_q;
    Eigen::Matrix<double, 3, NUM_LEG> foot_qd;
    Eigen::Matrix<double, 12, 12> foot_jacobian;
    Eigen::Matrix<double, 12, 12> foot_jacobian_inv;
    // Command in robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pDes;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pDes_robot;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pDes_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vDes;
    Eigen::Matrix<double, 3, NUM_LEG> foot_qDes;
    Eigen::Matrix<double, 3, NUM_LEG> foot_qdDes;

    /*! Gait Phase Parameters */
    int contacts[NUM_LEG]; // True, if foot in contact
    bool plan_contacts[NUM_LEG];
    bool early_contacts[NUM_LEG]; // True, if foot hit objects during swing
    Eigen::Vector4f phase_variable;

    /*! MPC Parameters */
    double plan_dt;
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;
    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;//x[0]
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_list;//X

    /*! State Estimate Parameters */
    bool estimate_contacts[NUM_LEG];
    Eigen::Vector3d estimate_position; // in world frame
    Eigen::Vector3d estimate_vel;      // in world frame

    /*! PD Parameters */
    Eigen::Matrix<double, NUM_DOF, 1> joint_position;
    Eigen::Matrix<double, NUM_DOF, 1> joint_velocity;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;
    // Still have problems in some parameters
    double kp_lin_x;
    double kd_lin_x;
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;
    Eigen::Matrix<double, 3, NUM_LEG> kp_foot;
    Eigen::Matrix<double, 3, NUM_LEG> kd_foot;
    Eigen::Matrix<double, 3, 1> km_foot;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;
    
    //method
    state_data();
    ~state_data();
    void Reset();
    void InitParams();
    void gait_counter_reset();

};




class FSM_data
{
private:
    /* data */
public:
    //GaitScheduler* _gaitScheduler;
    //RobotControlParameters* controlParameters;
    //DesiredStateCommand<T>* _desiredStateCommand;
    //LegController<T>* _legController;
    state_data* state;
    LegController* _legController;
    A1BasicEKF* model_StateEstimate;
    Solver* mpc_solver;
    Robot* _quadruped;
    
    FSM_data();
    ~FSM_data();
};


#endif//