#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_
namespace quad{
    /*! State Mode */
    enum STATE_TYPE{
        STEADY,
        STOP,
        INIT,
        STAND,
        LOCOMOTION,
        WALK,
        TROT,
        PACE,
        GALLOP,
        END
    };
    //const float plan_dt=0.01 ;
    const float discrete_dt=0.01;
    // rad to deg -> deg = rad * Rad2Deg
    // deg to rad -> rad = deg / Rad2Deg
    const double Rad2Deg = 57.3;
    const double SAFETY_TOLERANCE = 10.0;
    /*! Command Limit */
    const double JOY_CMD_VELX_MAX = 0.6;
    const double JOY_CMD_VELY_MAX = 0.3;
    const double JOY_CMD_YAW_MAX = 0.8;
    const double JOY_CMD_BODY_HEIGHT_VEL = 0.04;
    const double JOY_CMD_BODY_HEIGHT_MAX = 0.35;
    const double JOY_CMD_BODY_HEIGHT_MIN = 0.1;

    /*! offset of gazebo*/
    const double DEFAULT_SHOULD_ANGLE = 10.0;
    const double HIP_ANGLE_OFFSET = 45.0;
    const double KNEE_ANGLE_OFFSET = 0.0;

    /*! bias of gazebo, use degree*/

    /*! Gait Scheduler */
    const float WALK_BETA = 0.75;
    const float TROT_BETA = 0.5;
    const float PACE_BETA = 0.5;
    const float GALLOP_BETA = 0.5;
    /*! constant */
    const double X_OFFSET = 0.17;
    const double Y_OFFSET = 0.15;
    const double HIP_LENGTH = 0.21;
    const double KNEE_LENGTH = 0.21;
    const int NUM_DOF = 12;
    const int NUM_LEG = 4;
    const double FOOT_SWING_CLEARANCE1 = 0.0f;
    const double FOOT_SWING_CLEARANCE2 = 0.4f;
    const double FOOT_DELTA_X_LIMIT = 0.1;
    const double FOOT_DELTA_Y_LIMIT = 0.1;
    /*! MPC */
    const int PLAN_HORIZON = 10;
    const int MPC_STATE_DIM = 13;
    const int MPC_CONSTRAINT_DIM = 20;
}

#endif 
