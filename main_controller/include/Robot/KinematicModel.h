#ifndef _FORWARDKINEMATIC_
#define _FORWARDKINEMATIC_

#include <defination.h>
using namespace quad;

class KinematicModel{
public:
    const double X_OFFSET = quad::X_OFFSET;
    const double Y_OFFSET = quad::Y_OFFSET;
    double l1 = quad::HIP_LENGTH;
    double l2 = quad::KNEE_LENGTH;
    Eigen::Vector3d init_p_[4];
    Eigen::Vector3d cur_p_[4];
    Eigen::Vector3d temp_p_[4];
    Eigen::Vector3d des_q_[4];

    KinematicModel(){
        Initialize();
    }
    ~KinematicModel(){
    }
    void Initialize();
    /*!
     * Return the leg coordinate in robot frame
     */
    Eigen::Vector3d ForwardKinematic(int _leg, Eigen::Vector3d _q);
    /*!
     * theta1->[-pi/4,pi/4] theta2->[-pi/2, 0]
     */
    Eigen::Vector3d InverseKinematic(int _leg, Eigen::Vector3d _p);
};


#endif 
