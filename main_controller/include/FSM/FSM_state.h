#ifndef _FSM_STATE_
#define _FSM_STATE_
#include <vector>
#include "defination.h"
#include <FSM/FSM_state.h>
#include "FSM/FSM_data.h"
#include "FSM/FSM_tpcl.h"
#include "eigen_types.h"


struct TransitionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransitionData() { zero(); }

  // Zero out all of the data
  void zero() {
    // Flag to mark when transition is done
    done = false;
    // Timing parameters
    t0 = 0.0;         // time that transition started
    tCurrent = 0.0;   // current time since transition started
    tDuration = 0.0;  // overall transition duration
    // Robot state at the beginning of transition
    comState0 = Vec12<double>::Zero();  // center of mass state
    qJoints0 = Mat34<double>::Zero();   // joint positions
    pFoot0 = Mat34<double>::Zero();     // foot positions

    // Current robot state
    comState = Vec12<double>::Zero();  // center of mass state
    qJoints = Mat34<double>::Zero();   // joint positions
    pFoot = Mat34<double>::Zero();     // foot positions
  }

  // Flag to mark when transition is done
  bool done = false;

  // Timing parameters
  double t0;         // time that transition started
  double tCurrent;   // current time since transition started
  double tDuration;  // overall transition duration

  // Robot state at the beginning of transition
  Vec12<double> comState0;  // center of mass state
  Mat34<double> qJoints0;   // joint positions
  Mat34<double> pFoot0;     // foot positions

  // Current robot state
  Vec12<double> comState;  // center of mass state
  Mat34<double> qJoints;   // joint positions
  Mat34<double> pFoot;     // foot positions
};

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName
{
    INVALID,
    STEADY,
    STAND_UP,
    LOCOMOTION,
};

/**
 *
 */

class FSM_State
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Generic constructor for all states
    FSM_State(FSM_data *data, FSM_StateName stateNameIn, std::string stateStringIn);

    // Run the normal behavior for the state
    virtual void run() = 0; //{}

    // Behavior to be carried out when entering a state
    virtual void onEnter() = 0; // {}

    // Behavior to be carried out when exiting a state
    virtual void onExit() = 0; // {}

    // Manages state specific transitions
    virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

    // Runs the transition behaviors and returns true when done transitioning
    virtual TransitionData transition() { return transitionData; }

    //chck
    void turnOnAllSafetyChecks();
    void turnOffAllSafetyChecks();

    // Holds all of the relevant control data
    FSM_data *data_;

    // FSM State info
    FSM_StateName stateName;     // enumerated name of the current state
    FSM_StateName nextStateName; // enumerated name of the next state
    std::string stateString;     // state name string

    // Transition parameters
    double transitionDuration; // transition duration time
    double tStartTransition;   // time transition starts
    TransitionData transitionData;

    // Pre controls safety checks
    // Post control safety checks
    bool checkSafeOrientation = false; // check roll and pitch
    bool checkPDesFoot = false;         // do not command footsetps too far
    bool checkQDes = false;
    bool checkForceFeedForward = false; // do not command huge forces
    bool checkLegSingularity = false;   // do not let leg

    // Leg controller command placeholders for the whole robot (3x4 matrices)
    Mat34<double> jointFeedForwardTorques; // feed forward joint torques
    Mat34<double> jointPositions;          // joint angle positions
    Mat34<double> jointVelocities;         // joint angular velocities
    Mat34<double> footFeedForwardForces;   // feedforward forces at the feet
    Mat34<double> footPositions;           // cartesian foot positions
    Mat34<double> footVelocities;          // cartesian foot velocities
    // Footstep locations for next step
    Mat34<double> footstepLocations;

private:
    Mat3<float> kpMat;
    Mat3<float> kdMat;
};

#endif