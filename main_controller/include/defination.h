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
    const double dt = 0.002;
    const double df = 500; 
    const int horizon = 10;
}

#endif 
