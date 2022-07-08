#include <Robot/Robot.h>
LegController::LegController()
{

    model_kinematic = new KinematicModel();
    model_LegKinematic = new LegKinematics();
    
    for (int i = 0; i < 4; i++)
    {
        this->command[i].zero();
        this->data[i].zero();
    }
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    double motor_offset[4] = {};
    // 0-FL  1-FR  2-RR  3-RL
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;

    for (int i = 0; i < quad::NUM_LEG; i++)
    {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], quad::HIP_LENGTH, quad::KNEE_LENGTH;
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }
}
LegController::~LegController()
{
    delete this->model_kinematic;
    delete this->model_LegKinematic;
}

void LegController::FirstUpdateData(FSM_data&data)
{
    this->dt_ = data.state->plan_dt;
    for (int foot = 0; foot < 4; foot++)
    {
        this->data[foot].q(0) = 0.0;
        this->data[foot].q(1) = 30 / quad::Rad2Deg;
        this->data[foot].q(2) = -60 / quad::Rad2Deg;
        this->data[foot].p = this->model_LegKinematic->fk(
            this->data[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
        this->data[foot].J = this->model_LegKinematic->jac(
            this->data[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
        this->data[foot].J_inv = this->data[foot].J.inverse();
        this->data[foot].v = this->data[foot].J * this->data[foot].qd;

        /*! Update to state interior */
        data.state->foot_p_robot.block<3, 1>(0, foot) = this->data[foot].p;
        data.state->foot_v_robot.block<3, 1>(0, foot) = this->data[foot].v;
        data.state->foot_q.block<3, 1>(0, foot) = this->data[foot].q;
        data.state->foot_qd.block<3, 1>(0, foot) = this->data[foot].qd;
    }
}

void LegController::UpdateData()
{
    for (int foot = 0; foot < 4; foot++)
    {
        this->data[foot].p = this->model_LegKinematic->fk(
            this->data[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
        this->data[foot].J = this->model_LegKinematic->jac(
            this->data[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
        this->data[foot].J_inv = this->data[foot].J.inverse();
        this->data[foot].v = this->data[foot].J * this->data[foot].qd;
    }
}
void LegController::UpdateCommand()
{
    for (int foot = 0; foot < 4; foot++)
    {
        this->command[foot].qDes = this->model_kinematic->InverseKinematic(foot,
                                                                                this->command[foot].pDes);
        Eigen::Vector2d temp_v;
        temp_v.x() = this->command[foot].vDes.x();
        temp_v.y() = this->command[foot].vDes.z();
        Eigen::Vector2d temp_ = this->data[foot].J_temp.inverse() * temp_v;
        this->command[foot].qdDes.x() = 0.0;
        this->command[foot].qdDes.y() = temp_.x();
        this->command[foot].qdDes.z() = temp_.y();
    }
}

void LegController::ComputeJacobian(Eigen::Vector3d &q, Eigen::Matrix2d *J, Eigen::Matrix3d *J_3DoF, int leg)
{
    double l1 = quad::HIP_LENGTH;
    double l2 = quad::KNEE_LENGTH;

    double s1 = std::sin(q(1));
    double s2 = std::sin(q(2));
    double c1 = std::cos(q(1));
    double c2 = std::cos(q(2));
    double s12 = s1 * c2 + c1 * s2;
    double c12 = c1 * c2 - s1 * s2;

    J->operator()(0, 0) = -(l1 * c1 + l2 * c12);
    J->operator()(0, 1) = -(l2 * c12);
    J->operator()(1, 0) = l1 * s1 + l2 * s12;
    J->operator()(1, 1) = l2 * s12;

    J_3DoF->operator()(0, 0) = 0;
    J_3DoF->operator()(0, 1) = -(l1 * c1 + l2 * c12);
    J_3DoF->operator()(0, 2) = -(l2 * c12);
    J_3DoF->operator()(1, 0) = l2 * c12 + l1 * c1;
    J_3DoF->operator()(1, 1) = 0;
    J_3DoF->operator()(1, 2) = 0;
    J_3DoF->operator()(2, 0) = 0;
    J_3DoF->operator()(2, 1) = l1 * s1 + l2 * s12;
    J_3DoF->operator()(2, 2) = l2 * s12;
}
