#ifndef _FSM_
#define _FSM_
#include <vector>
#include "defination.h"
#include "FSM/State_Worker/State_Worker.h"
#include "FSM/FSM_data.h"
#include "FSM/FSM_tpcl.h"

class FSM{
private:

public:
    std::vector<StateWorker*> Workers;
    int flow = 0;
    //New ADD
    FSM_data global_data;
    FSM_topic_control topic_contrl;
    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop();
    void build_ScheduleTable(int Schedule, ...);
    void Update_MPC();
    void Update_StateEstimate();
    void Send_CMD();
    void Update_LegController();
};
#endif