#include <FSM/FSM_data.h>

FSM_data::FSM_data()
{
    model_StateEstimate = new A1BasicEKF();
    Mat3<double> inerial;
    inerial <<0.0158533,-3.66e-5, -6.11e-5, -3.66e-5 ,0.0377999 ,-2.75e-5 ,-6.11e-5 ,- 2.75e-5 ,0.0456542;
    _quadruped = new Robot(13.0, 0.2, 0.2, 0.1805, 0.1308, inerial);
    _legController = new LegController();
}

FSM_data::~FSM_data()
{
    
}



