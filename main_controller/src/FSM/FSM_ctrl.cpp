#include "FSM/FSM_state.h"
#include "FSM/FSM_ctrl.h"
#include "FSM/FSM_tpcl.h"
#include "FSM/FSM_data.h"

FSM_ctrl::FSM_ctrl() : tpcl_(data_)
{
    // Initialize and add all of the FSM States to the state list
    statesList.invalid = nullptr;
    statesList.steady = new SteadyWorker(&data_);
    statesList.standUp = new StandWorker(&data_);
    statesList.locomotion = new Locomotion(&data_);
    safetyChecker = new SafetyChecker(&data_);// safety check
    // Initialize the FSM with the Passive FSM State
    initialize();
}
FSM_ctrl::~FSM_ctrl()
{
    delete statesList.steady;
    delete statesList.standUp;
    delete statesList.locomotion;
}
/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */

void FSM_ctrl::initialize()
{
    // Initialize a new FSM State with the control data_
    currentState = statesList.steady;

    // Enter the new current state cleanly
    currentState->onEnter();

    // Initialize to not be in transition
    nextState = currentState;

    // Initialize FSM mode to normal operation
    operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
void FSM_ctrl::runFSM()
{
    // Check the robot state for safe operation
    operatingMode = safetyPreCheck();
    // Run the robot control code if operating mode is not unsafe
    if (operatingMode != FSM_OperatingMode::ESTOP)
    {
        // Run normal controls if no transition is detected
        if (operatingMode == FSM_OperatingMode::NORMAL)
        {
            // Check the current state for any transition
            nextStateName = currentState->checkTransition();
            // Detect a commanded transition
            if (nextStateName != currentState->stateName)
            {
                // Set the FSM operating mode to transitioning
                operatingMode = FSM_OperatingMode::TRANSITIONING;

                // Get the next FSM State by name
                nextState = getNextState(nextStateName);
                
                // Print transition initialized info
                // printInfo(1);
            }
            else
            {
                // Run the iteration for the current state normally
                currentState->run();
            }
        }
        // Run the transition code while transition is occuring
        if (operatingMode == FSM_OperatingMode::TRANSITIONING)
        {
            transitionData = currentState->transition();

            // Check the robot state for safe operation
            safetyPostCheck();
            // Run the state transition
            if (transitionData.done)
            {
                // Exit the current state cleanly
                currentState->onExit();

                // Print finalizing transition info
                // printInfo(2);
                // Complete the transition
                currentState = nextState;
                // Enter the new current state cleanly
                currentState->onEnter();
                // Return the FSM to normal operation mode
                operatingMode = FSM_OperatingMode::NORMAL;
            }
        }
        else
        {
            // Check the robot state for safe operation
            safetyPostCheck();
        }
    }
    else
    { // if ESTOP
        currentState = statesList.steady;
        currentState->onEnter();
        nextStateName = currentState->stateName;
    }

    // Print the current state of the FSM
    printInfo(0);

    // Increase the iteration counter
    iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */

FSM_OperatingMode FSM_ctrl::safetyPreCheck()
{
    // Check for safe orientation if the current state requires it
    if (currentState->checkSafeOrientation)
    {
        if (!safetyChecker->checkSafeOrientation())
        {
            operatingMode = FSM_OperatingMode::ESTOP;
            std::cout << "broken: Orientation Safety Ceck FAIL" << std::endl;
        }
    }

    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */

FSM_OperatingMode FSM_ctrl::safetyPostCheck()
{
    // Check for safe desired foot positions
    // if (currentState->checkPDesFoot)
    // {
    //     safetyChecker->checkPDesFoot();
    // }
    if (currentState->checkPDesFoot)
    {
        safetyChecker->checkQDesFoot();
    }
    // Check for safe desired feedforward forces
    // if (currentState->checkForceFeedForward)
    // {
    //     safetyChecker->checkForceFeedForward();
    // }
    // Default is to return the current operating mode
    return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
FSM_State *FSM_ctrl::getNextState(FSM_StateName stateName)
{
    switch (stateName)
    {
    case FSM_StateName::INVALID:
        return statesList.invalid;

    case FSM_StateName::STEADY:
        return statesList.steady;

    case FSM_StateName::STAND_UP:
        return statesList.standUp;

    case FSM_StateName::LOCOMOTION:
        return statesList.locomotion;

    default:
        return statesList.invalid;
    }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
void FSM_ctrl::printInfo(int opt)
{
    switch (opt)
    {
    case 0: // Normal printing case at regular intervals
        // Increment printing iteration
        printIter++;

        // Print at commanded frequency
        if (printIter == printNum)
        {
            std::cout << "[CONTROL FSM] Printing FSM Info...\n";
            std::cout
                << "---------------------------------------------------------\n";
            std::cout << "Iteration: " << iter << "\n";
            if (operatingMode == FSM_OperatingMode::NORMAL)
            {
                std::cout << "Operating Mode: NORMAL in " << currentState->stateString
                          << "\n";
            }
            else if (operatingMode == FSM_OperatingMode::TRANSITIONING)
            {
                std::cout << "Operating Mode: TRANSITIONING from "
                          << currentState->stateString << " to "
                          << nextState->stateString << "\n";
            }
            else if (operatingMode == FSM_OperatingMode::ESTOP)
            {
                std::cout << "Operating Mode: ESTOP\n";
            }
            // std::cout << "Gait Type: " << data_._gaitScheduler->gaitData.gaitName
            //   << "\n";
            // std::cout << std::endl;

            // Reset iteration counter
            printIter = 0;
        }

        // Print robot info about the robot's status
        // data_._gaitScheduler->printGaitInfo();
        // data_._desiredStateCommand->printStateCommandInfo();

        break;

    case 1: // Initializing FSM State transition
        std::cout << "[CONTROL FSM] Transition initialized from "
                  << currentState->stateString << " to " << nextState->stateString
                  << "\n"
                  << std::endl;

        break;

    case 2: // Finalizing FSM State transition
        std::cout << "[CONTROL FSM] Transition finalizing from "
                  << currentState->stateString << " to " << nextState->stateString
                  << "\n"
                  << std::endl;

        break;
    }
}
