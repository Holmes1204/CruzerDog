#include <FSM/FSM_tpcl.h>
#include <FSM/FSM_data.h>

FSM_topic_control::FSM_topic_control(ros::NodeHandle &nh, FSM_data &data) : nh_(nh), data_(data)
{
        this->joy_sub_ = this->joy_sub_ = this->nh_.subscribe("/joy", 2, &FSM_topic_control::Joy_Callback, this);
        this->em_cmd_pub_ = this->nh_.advertise<wtr_serial_msg::em_ev>("/em_ev", 1);
        this->em_fdb_sub_ = this->nh_.subscribe("/em_fb_raw", 2, &FSM_topic_control::em_fdb_callback, this);
        unitree_sim_set_up();
}

FSM_topic_control::~FSM_topic_control()
{
}

// joy_control
void FSM_topic_control::Joy_Callback(const sensor_msgs::Joy &msg)
{
        //复杂的控制逻辑源于对实现过程的精准把握
        this->data_.state->command_vel.x() = 0;
        this->data_.state->command_vel.y() = 0;
        this->data_.state->command_vel.z() = 0;
        if (msg.buttons[0] == 1 && msg.buttons[1] == 1)
        {
                this->data_.global_state_switch++;
        }
        if (msg.buttons[3] == 1 && msg.buttons[4] == 1)
        {
                this->data_.global_gait_switch++;
        }

        // Debug
        // system("clear");
        // std::cout << "v_x" << msg << std::endl;
}
// real
void FSM_topic_control::em_fdb_callback(const wtr_serial_msg::em_fb_raw &msg)
{
        // std::cout<<"--------------------\n";
        for (int leg = 0; leg < 4; leg++)
        {
                //未考虑电机开始时的偏置，之后添加
                data_._legController->data[leg].q(0) = 0;
                data_._legController->data[leg].qd(0) = 0;

                data_._legController->data[leg].q(1) = mt_fdb.em_pos_fb_raw[leg * 2];
                data_._legController->data[leg].qd(1) = mt_fdb.em_vel_fb_raw[leg * 2];

                data_._legController->data[leg].q(2) = mt_fdb.em_pos_fb_raw[leg * 2 + 1];
                data_._legController->data[leg].qd(2) = mt_fdb.em_vel_fb_raw[leg * 2 + 1];

                // imu的解算之后再加
                // leg[leg].compute_Jacobian(leg);
                // leg[leg].get_foot_pos(leg);
                // leg[leg].get_foot_vel();
                // debug
                // std::cout<<"leg"<<leg<<std::endl;
                // std::cout<<leg[leg].J<<std::endl;
                // std::cout<<"q "<<leg[leg].q.transpose()<<"\n"<<"d "<<leg[leg].qd.transpose()<<std::endl;
                // std::cout<<"p "<<leg[leg].p.transpose()<<"\n"<<"v "<<leg[leg].v.transpose()<<std::endl;
        }
        return;
}

void FSM_topic_control::em_cmd_send()
{
        for (int leg = 0; leg < 4; leg++)
        {
                // tauFF
                Eigen::Vector3d legTorque = data_._legController->command[leg].tau_FF;
                // forceFF
                Eigen::Vector3d footForce = data_._legController->command[leg].force_FF;
                // cartesian PD
                footForce +=
                    data_._legController->command[leg].kpCartesian *
                    (data_._legController->command[leg].p_Des - data_._legController->data[leg].p);
                footForce +=
                    data_._legController->command[leg].kdCartesian *
                    (data_._legController->command[leg].v_Des - data_._legController->data[leg].v);
                // Torque
                legTorque += data_._legController->data[leg].J.transpose() * footForce;
                // estimate torque
                data_._legController->data[leg].tauEstimate =
                    legTorque +
                    data_._legController->command[leg].kpJoint *
                        (data_._legController->command[leg].q_Des - data_._legController->data[leg].q) +
                    data_._legController->command[leg].kdJoint *
                        (data_._legController->command[leg].qd_Des - data_._legController->data[leg].qd);

                mt_cmd.em_ev_pos[2 * leg] = data_._legController->command[leg].q_Des(1);
                mt_cmd.em_ev_vel[2 * leg] = data_._legController->command[leg].qd_Des(1);
                mt_cmd.em_ev_kp[2 * leg] = data_._legController->command[leg].kpJoint(1, 1);
                mt_cmd.em_ev_kd[2 * leg] = data_._legController->command[leg].kdJoint(1, 1);
                mt_cmd.em_ev_trq[2 * leg] = legTorque(1);

                mt_cmd.em_ev_pos[2 * leg + 1] = data_._legController->command[leg].q_Des(2);
                mt_cmd.em_ev_vel[2 * leg + 1] = data_._legController->command[leg].qd_Des(2);
                mt_cmd.em_ev_kp[2 * leg + 1] = data_._legController->command[leg].kpJoint(2, 2);
                mt_cmd.em_ev_kd[2 * leg + 1] = data_._legController->command[leg].kdJoint(2, 2);
                mt_cmd.em_ev_trq[2 * leg + 1] = legTorque(2);
                // on or off
                mt_cmd.em_state[0] = 0;
                mt_cmd.em_state[1] = 0;
        }
        em_cmd_pub_.publish(mt_cmd);
        return;
}

// unitree_sim
void FSM_topic_control::cheater_estimator_callback(const gazebo_msgs::ModelStates &msg)
{
        geometry_msgs::Pose model_pose = msg.pose.back();
        geometry_msgs::Twist model_twist = msg.twist.back();
        StateEstimateResult &result = this->data_.model_StateEstimate->result;
        result.position.x() = model_pose.position.x;
        result.position.y() = model_pose.position.y;
        result.position.z() = model_pose.position.z;
        result.orientation.x() = model_pose.orientation.x;
        result.orientation.y() = model_pose.orientation.y;
        result.orientation.z() = model_pose.orientation.z;
        result.orientation.w() = model_pose.orientation.w;
        result.rpy = ori::quatToRPY(result.orientation);
        result.omegaWorld.x() = model_twist.angular.x;
        result.omegaWorld.y() = model_twist.angular.y;
        result.omegaWorld.z() = model_twist.angular.z;
        result.vWorld.x() = model_twist.linear.x;
        result.vWorld.y() = model_twist.linear.y;
        result.vWorld.z() = model_twist.linear.z;
        result.rBody = result.orientation.toRotationMatrix().transpose();
        // {

        //         static uint32_t dbg_count = 0;
        //         static int i = 0;
        //         if (dbg_count % 40 == 0)
        //         {
        //                 system("clear");
        //                 std::cout << "P" << std::endl
        //                           << result.position.transpose() << std::endl
        //                           << "RPY" << std::endl
        //                           << result.rpy.transpose() << std::endl
        //                           << "v" << std::endl
        //                           << result.vWorld.transpose() << std::endl
        //                           << "omega" << std::endl
        //                           << result.omegaWorld.transpose() << std::endl
        //                           << "rBody" << std::endl
        //                           << result.rBody << std::endl
        //                           << std::endl;
        //                 dbg_count = 0;
        //         }
        //         dbg_count++;
        // }
}

void FSM_topic_control::unitree_sim_set_up()
{

        // joint publisher
        this->joint_sim_unitree_pub[0] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/FL_thigh_controller/command", 1);
        this->joint_sim_unitree_pub[1] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/FL_calf_controller/command", 1);
        this->joint_sim_unitree_pub[2] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/FR_thigh_controller/command", 1);
        this->joint_sim_unitree_pub[3] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/FR_calf_controller/command", 1);
        this->joint_sim_unitree_pub[4] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/RL_thigh_controller/command", 1);
        this->joint_sim_unitree_pub[5] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/RL_calf_controller/command", 1);
        this->joint_sim_unitree_pub[6] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/RR_thigh_controller/command", 1);
        this->joint_sim_unitree_pub[7] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_8dof_gazebo/RR_calf_controller/command", 1);
        // subscriber
        this->imu_sim_unitree_sub = this->nh_.subscribe("/trunk_imu", 5, &FSM_topic_control::Imu_Callback, this);
        this->cheater_estimator_sub = this->nh_.subscribe("/gazebo/model_states", 5, &FSM_topic_control::cheater_estimator_callback, this);
        this->footForce_sim_unitree_sub[0] = this->nh_.subscribe("/visual/FR_foot_contact/the_force", 5, &FSM_topic_control::FRfootCallback, this);
        this->footForce_sim_unitree_sub[1] = this->nh_.subscribe("/visual/FL_foot_contact/the_force", 5, &FSM_topic_control::FLfootCallback, this);
        this->footForce_sim_unitree_sub[2] = this->nh_.subscribe("/visual/RR_foot_contact/the_force", 5, &FSM_topic_control::RRfootCallback, this);
        this->footForce_sim_unitree_sub[3] = this->nh_.subscribe("/visual/RL_foot_contact/the_force", 5, &FSM_topic_control::RLfootCallback, this);
        this->joint_sim_unitree_sub[0] = this->nh_.subscribe("/a1_8dof_gazebo/FL_thigh_controller/state", 1, &FSM_topic_control::FL_thigh_state_callback, this);
        this->joint_sim_unitree_sub[1] = this->nh_.subscribe("/a1_8dof_gazebo/FL_calf_controller/state", 1, &FSM_topic_control::FL_calf_state_callback, this);
        this->joint_sim_unitree_sub[2] = this->nh_.subscribe("/a1_8dof_gazebo/FR_thigh_controller/state", 1, &FSM_topic_control::FR_thigh_state_callback, this);
        this->joint_sim_unitree_sub[3] = this->nh_.subscribe("/a1_8dof_gazebo/FR_calf_controller/state", 1, &FSM_topic_control::FR_calf_state_callback, this);
        this->joint_sim_unitree_sub[4] = this->nh_.subscribe("/a1_8dof_gazebo/RL_thigh_controller/state", 1, &FSM_topic_control::RL_thigh_state_callback, this);
        this->joint_sim_unitree_sub[5] = this->nh_.subscribe("/a1_8dof_gazebo/RL_calf_controller/state", 1, &FSM_topic_control::RL_calf_state_callback, this);
        this->joint_sim_unitree_sub[6] = this->nh_.subscribe("/a1_8dof_gazebo/RR_thigh_controller/state", 1, &FSM_topic_control::RR_thigh_state_callback, this);
        this->joint_sim_unitree_sub[7] = this->nh_.subscribe("/a1_8dof_gazebo/RR_calf_controller/state", 1, &FSM_topic_control::RR_calf_state_callback, this);
}

void FSM_topic_control::unitree_sim_data_decode()
{
        // motor data decod
        for (int leg = 0; leg < 4; leg++)
        {
                //未考虑电机开始时的偏置，之后添加
                data_._legController->data[leg].q(0) = 0;
                data_._legController->data[leg].qd(0) = 0;

                data_._legController->data[leg].q(1) = low_state.motorState[leg * 2].q;
                data_._legController->data[leg].qd(1) = low_state.motorState[leg * 2].dq;

                data_._legController->data[leg].q(2) = low_state.motorState[leg * 2 + 1].q;
                data_._legController->data[leg].qd(2) = low_state.motorState[leg * 2 + 1].dq;
                // todo: 这样写太麻烦了应该简化
                data_._legController->data[leg].J = data_._quadruped->cal_jacobian(data_._legController->data[leg].q, leg);
                data_._legController->data[leg].p = data_._quadruped->forward_kinematic(data_._legController->data[leg].q, leg);
                data_._legController->data[leg].v = data_._legController->data[leg].J * data_._legController->data[leg].qd;
        }
        // debug
        // {

        //         static uint32_t dbg_count = 0;
        //         static int i=0;
        //         if (dbg_count % 40 == 0)
        //         {
        //                 system("clear");
        //                 std::cout << "J" << std::endl
        //                           << data_._legController->data[i].J << std::endl
        //                           << "p" << std::endl
        //                           << data_._legController->data[i].p.transpose() << std::endl
        //                           << "v" << std::endl
        //                           << data_._legController->data[i].v.transpose() << std::endl
        //                           << "q" << std::endl
        //                           << data_._legController->data[i].q.transpose() << std::endl
        //                           << "qd" << std::endl
        //                           << data_._legController->data[i].qd.transpose() << std::endl
        //                           << std::endl;
        //                 dbg_count=0;
        //         }
        //         dbg_count++;
        // }
}

void FSM_topic_control::unitree_sim_send_cmd()
{
        for (int leg = 0; leg < 4; leg++)
        {
                // tauFF
                Eigen::Vector3d legTorque = data_._legController->command[leg].tau_FF;
                // forceFF
                Eigen::Vector3d footForce = data_._legController->command[leg].force_FF;
                // cartesian PD
                footForce +=
                    data_._legController->command[leg].kpCartesian *
                    (data_._legController->command[leg].p_Des - data_._legController->data[leg].p);
                footForce +=
                    data_._legController->command[leg].kdCartesian *
                    (data_._legController->command[leg].v_Des - data_._legController->data[leg].v);
                // Torque
                legTorque += data_._legController->data[leg].J.transpose() * footForce;
                // estimate torque
                data_._legController->data[leg].tauEstimate =
                    legTorque +
                    data_._legController->command[leg].kpJoint *
                        (data_._legController->command[leg].q_Des - data_._legController->data[leg].q) +
                    data_._legController->command[leg].kdJoint *
                        (data_._legController->command[leg].qd_Des - data_._legController->data[leg].qd);

                low_cmd.motorCmd[2 * leg].mode = 0x0A;
                low_cmd.motorCmd[2 * leg].q = data_._legController->command[leg].q_Des(1);
                low_cmd.motorCmd[2 * leg].dq = data_._legController->command[leg].qd_Des(1);
                low_cmd.motorCmd[2 * leg].Kp = data_._legController->command[leg].kpJoint(1, 1);
                low_cmd.motorCmd[2 * leg].Kd = data_._legController->command[leg].kdJoint(1, 1);
                low_cmd.motorCmd[2 * leg].tau = legTorque(1);
                joint_sim_unitree_pub[2 * leg].publish(low_cmd.motorCmd[2 * leg]);

                low_cmd.motorCmd[2 * leg + 1].mode = 0x0A;
                low_cmd.motorCmd[2 * leg + 1].q = data_._legController->command[leg].q_Des(2);
                low_cmd.motorCmd[2 * leg + 1].dq = data_._legController->command[leg].qd_Des(2);
                low_cmd.motorCmd[2 * leg + 1].Kp = data_._legController->command[leg].kpJoint(2, 2);
                low_cmd.motorCmd[2 * leg + 1].Kd = data_._legController->command[leg].kdJoint(2, 2);
                low_cmd.motorCmd[2 * leg + 1].tau = legTorque(2);
                joint_sim_unitree_pub[2 * leg + 1].publish(low_cmd.motorCmd[2 * leg + 1]);
        }
}

// FL_thigh -> 0
void FSM_topic_control::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[0].mode = msg.mode;
        low_state.motorState[0].q = msg.q;
        low_state.motorState[0].dq = msg.dq;
        low_state.motorState[0].tauEst = msg.tauEst;
        // std::cout << 1 << std::endl;
}
// FL_calf -> 1
void FSM_topic_control::FL_calf_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[1].mode = msg.mode;
        low_state.motorState[1].q = msg.q;
        low_state.motorState[1].dq = msg.dq;
        low_state.motorState[1].tauEst = msg.tauEst;
        // std::cout << 2 << std::endl;
}

// FR thigh ->2
void FSM_topic_control::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[2].mode = msg.mode;
        low_state.motorState[2].q = msg.q;
        low_state.motorState[2].dq = msg.dq;
        low_state.motorState[2].tauEst = msg.tauEst;
        // std::cout << 4 << std::endl;
}
// FR calf ->3
void FSM_topic_control::FR_calf_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[3].mode = msg.mode;
        low_state.motorState[3].q = msg.q;
        low_state.motorState[3].dq = msg.dq;
        low_state.motorState[3].tauEst = msg.tauEst;
        // std::cout << 5 << std::endl;
}

// RL thibh 4

void FSM_topic_control::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[4].mode = msg.mode;
        low_state.motorState[4].q = msg.q;
        low_state.motorState[4].dq = msg.dq;
        low_state.motorState[4].tauEst = msg.tauEst;
        // std::cout << 7 << std::endl;
}
// RL calf 5
void FSM_topic_control::RL_calf_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[5].mode = msg.mode;
        low_state.motorState[5].q = msg.q;
        low_state.motorState[5].dq = msg.dq;
        low_state.motorState[5].tauEst = msg.tauEst;
        // std::cout << 8 << std::endl;
}

// RR thigh 6
void FSM_topic_control::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[6].mode = msg.mode;
        low_state.motorState[6].q = msg.q;
        low_state.motorState[6].dq = msg.dq;
        low_state.motorState[6].tauEst = msg.tauEst;
        // std::cout << 10 << std::endl;
}
// RR calf 7
void FSM_topic_control::RR_calf_state_callback(const unitree_legged_msgs::MotorState &msg)
{
        low_state.motorState[7].mode = msg.mode;
        low_state.motorState[7].q = msg.q;
        low_state.motorState[7].dq = msg.dq;
        low_state.motorState[7].tauEst = msg.tauEst;
        // std::cout << 11 << std::endl;
}

void FSM_topic_control::Imu_Callback(const sensor_msgs::Imu &msg)
{
        low_state.imu.quaternion[0] = msg.orientation.w;
        low_state.imu.quaternion[1] = msg.orientation.x;
        low_state.imu.quaternion[2] = msg.orientation.y;
        low_state.imu.quaternion[3] = msg.orientation.z;

        low_state.imu.gyroscope[0] = msg.angular_velocity.x;
        low_state.imu.gyroscope[1] = msg.angular_velocity.y;
        low_state.imu.gyroscope[2] = msg.angular_velocity.z;

        low_state.imu.accelerometer[0] = msg.linear_acceleration.x;
        low_state.imu.accelerometer[1] = msg.linear_acceleration.y;
        low_state.imu.accelerometer[2] = msg.linear_acceleration.z;
}
void FSM_topic_control::FRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
        low_state.eeForce[0].x = msg.wrench.force.x;
        low_state.eeForce[0].y = msg.wrench.force.y;
        low_state.eeForce[0].z = msg.wrench.force.z;
        low_state.footForce[0] = msg.wrench.force.z;
}

void FSM_topic_control::FLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
        low_state.eeForce[1].x = msg.wrench.force.x;
        low_state.eeForce[1].y = msg.wrench.force.y;
        low_state.eeForce[1].z = msg.wrench.force.z;
        low_state.footForce[1] = msg.wrench.force.z;
}

void FSM_topic_control::RRfootCallback(const geometry_msgs::WrenchStamped &msg)
{
        low_state.eeForce[2].x = msg.wrench.force.x;
        low_state.eeForce[2].y = msg.wrench.force.y;
        low_state.eeForce[2].z = msg.wrench.force.z;
        low_state.footForce[2] = msg.wrench.force.z;
}

void FSM_topic_control::RLfootCallback(const geometry_msgs::WrenchStamped &msg)
{
        low_state.eeForce[3].x = msg.wrench.force.x;
        low_state.eeForce[3].y = msg.wrench.force.y;
        low_state.eeForce[3].z = msg.wrench.force.z;
        low_state.footForce[3] = msg.wrench.force.z;
}