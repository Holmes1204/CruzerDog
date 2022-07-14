#ifndef _FSM_CTRL_
#define _FSM_CTRL_

#include <iostream>
#include <FSM/FSM_data.h>     // Contains all of the control related data
#include <FSM/safety_check.h> // Checks the robot state and commands for safety
#include <FSM/State_Worker/Locomotion_Worker.h>
#include <FSM/State_Worker/Stand_Worker.h>
#include <FSM/State_Worker/Steady_Worker.h>
// FSM States

/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode
{
    NORMAL,
    TRANSITIONING,
    ESTOP,
    EDAMP
};

/**
 *
 */
struct FSM_StatesList
{
    FSM_State *invalid;
    SteadyWorker *steady;
    StandWorker *standUp;
    Locomotion *locomotion;
};

/**
 *
 */

struct FSM_ControllerList
{
};

/**
 * Control FSM handles the FSM states from a higher level
 */

class FSM_ctrl
{
public:
    FSM_ctrl(
        // Quadruped *_quadruped,
        // StateEstimatorContainer *_stateEstimator,
        // LegController *_legController,
        // GaitScheduler *_gaitScheduler,
        // DesiredStateCommand *_desiredStateCommand,
    );
    ~FSM_ctrl();
    // Initializes the Control FSM instance
    void initialize();

    // Runs the FSM logic and handles the state transitions and normal runs
    void runFSM();

    // This will be removed and put into the SafetyChecker class
    FSM_OperatingMode safetyPreCheck();
    //
    FSM_OperatingMode safetyPostCheck();

    // Gets the next FSM_State from the list of created states when requested
    FSM_State *getNextState(FSM_StateName stateName);

    // Prints the current FSM status
    void printInfo(int opt);

    // Contains all of the control related data
    FSM_data data_;
    FSM_topic_control tpcl_;
    // FSM state information
    FSM_StatesList statesList;   // holds all of the FSM States
    FSM_State *currentState;     // current FSM state
    FSM_State *nextState;        // next FSM state
    FSM_StateName nextStateName; // next FSM state name

    // Checks all of the inputs and commands for safety
    SafetyChecker *safetyChecker;

    TransitionData transitionData;

private:
    // Operating mode of the FSM
    FSM_OperatingMode operatingMode;

    // Choose how often to print info, every N iterations
    int printNum = 10000; // N*(0.001s) in simulation time

    // Track the number of iterations since last info print
    int printIter = 0; // make larger than printNum to not print

    int iter = 0;

    // lcm::LCM state_estimator_lcm;
    // state_estimator_lcmt _state_estimator;
};

#endif