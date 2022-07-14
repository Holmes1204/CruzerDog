#include "FSM/FSM_state.h"
#include "FSM/FSM_ctrl.h"
#include "FSM/FSM_tpcl.h"
#include "FSM/FSM_data.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */

FSM_State::FSM_State(FSM_data *data,
                     FSM_StateName stateNameIn,
                     std::string stateStringIn)
    : data_(data),
      stateName(stateNameIn),
      stateString(stateStringIn)
{
    transitionData.zero();
    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn<< std::endl;
}


/**>>
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */

void FSM_State::turnOnAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = true; // check roll and pitch
    // Post control safety checks
    checkPDesFoot = true;         // do not command footsetps too far
    checkForceFeedForward = true; // do not command huge forces
    checkLegSingularity = true;   // do not let leg
}

/**
 *
 */
void FSM_State::turnOffAllSafetyChecks()
{
    // Pre controls safety checks
    checkSafeOrientation = false; // check roll and pitch
    // Post control safety checks
    checkPDesFoot = false;         // do not command footsetps too far
    checkForceFeedForward = false; // do not command huge forces
    checkLegSingularity = false;   // do not let leg
}
