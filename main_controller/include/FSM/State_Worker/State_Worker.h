#ifndef _STATEWORKER_
#define _STATEWORKER_

#include "../FSM_data.h"
#include "../FSM_tpcl.h"

class StateWorker{
public:
    virtual void run() = 0;
    virtual bool is_finished() = 0;
    virtual void send()=0;
    bool is_working = 0;
    StateWorker(){};
    ~StateWorker(){};
};

#endif
