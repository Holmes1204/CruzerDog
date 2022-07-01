#ifndef _STATE_ESTIMATOR_
#define _STATE_ESTIMATOR_
#include <eigen3/Eigen/Dense>
#include <defination.h>
#include <FSM/FSM_data.h>
using namespace quad;

struct StateEstimateData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond orientation_;
    Eigen::Matrix3d rBody_;
    Eigen::Vector3d omega_body_;
    Eigen::Vector3d omega_world_;
    Eigen::Vector3d rpy_;
    Eigen::Vector3d aBody_;
    Eigen::Vector3d aWorld_;

    // output
    Eigen::Vector3d position_;
    Eigen::Vector3d vWorld_;
    Eigen::Vector3d vBody_;
};

class StateEstimate
{
public:
    const double imu_process_noise_position = 0.02;
    const double imu_process_noise_velocity = 0.02;
    const double foot_process_noise_position = 0.002;
    const double foot_sensor_noise_position = 0.01;
    const double foot_sensor_noise_velocity = 0.1;
    const double foot_height_sensor_noise = 0.001;

    /*    const double imu_process_noise_position = 0;
        const double imu_process_noise_velocity = 0;
        const double foot_process_noise_position = 0;
        const double foot_sensor_noise_position = 0;
        const double foot_sensor_noise_velocity = 0;
        const double foot_height_sensor_noise = 0;*/

    StateEstimateData state_data_;

    StateEstimate();
    ~StateEstimate();

    void UpdateOrientationData(state_data data);
    void InitParameters();

    /*!
     * Get the position and velocity
     * */
    void LinearKF(Eigen::Matrix<double, 3, quad::NUM_LEG> _foot_p, Eigen::Matrix<double, 3, 
                quad::NUM_LEG> _foot_v, Eigen::Vector4d _contact_phase);
    /*!
     * @return : the position of body
     * */
    Eigen::Vector3d getPosition()
    {
        return this->state_data_.position_;
    }

    /*!
     * @return : the velocity in world frame
     * */
    Eigen::Vector3d getVelocity()
    {
        return this->state_data_.vWorld_;
    }

    /*!
     * @return : the velocity in body frame
     * */
    Eigen::Vector3d getVelocity_Body()
    {
        return this->state_data_.vBody_;
    }
    /*!
     * @return : the data of position velocity_w velocity body
     * */
    Eigen::Matrix<double, 3, 3> getData();

private:
    Eigen::Matrix<double, 18, 1> _xhat; // x:=(r v p1 p2 p3 p4)
    Eigen::Matrix<double, 12, 1> _ps;   //
    Eigen::Matrix<double, 12, 1> _vs;   //
    Eigen::Matrix<double, 18, 18> _A;   // Fk
    Eigen::Matrix<double, 18, 18> _Q0;  //
    Eigen::Matrix<double, 18, 18> _P;   //
    Eigen::Matrix<double, 28, 28> _R0;  //
    Eigen::Matrix<double, 18, 3> _B;    //
    Eigen::Matrix<double, 28, 18> _C;   // Hk
};

#endif