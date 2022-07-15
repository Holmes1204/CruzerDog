/**
 * Checks the robot state for safe operation command after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for command that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

#include <FSM/safety_check.h>
#include "FSM/FSM_state.h"
#include "FSM/FSM_ctrl.h"
#include "FSM/FSM_tpcl.h"
#include "FSM/FSM_data.h"
/**
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkSafeOrientation()
{
  if (abs(data->model_StateEstimate->getResult().rpy(0)) >= 0.5 ||
      abs(data->model_StateEstimate->getResult().rpy(1)) >= 0.5)
  {
    printf("Orientation safety check failed!\n");
    return false;
  }
  else
  {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkPDesFoot()
{
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  double maxAngle = 1.0472; // 60 degrees (should be changed)
  double maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++)
  {
    // Keep the foot from going too far from the body in +x
    if (data->_legController->command[leg].p_Des(0) > maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(0)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->command[leg].p_Des(0) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -x
    if (data->_legController->command[leg].p_Des(0) < -maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(0)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->command[leg].p_Des(0) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in +y
    if (data->_legController->command[leg].p_Des(1) > maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(1)
                << " | modified: " << maxPDes << std::endl;
      data->_legController->command[leg].p_Des(1) = maxPDes;
      safePDesFoot = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->command[leg].p_Des(1) < -maxPDes)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(1)
                << " | modified: " << -maxPDes << std::endl;
      data->_legController->command[leg].p_Des(1) = -maxPDes;
      safePDesFoot = false;
    }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    if (data->_legController->command[leg].p_Des(2) >
        -data->_quadruped->_maxLegLength / 4)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(2)
                << " | modified: " << -data->_quadruped->_maxLegLength / 4
                << std::endl;
      data->_legController->command[leg].p_Des(2) =
          -data->_quadruped->_maxLegLength / 4;
      safePDesFoot = false;
    }

    // Keep the foot within the kinematic limits
    if (data->_legController->command[leg].p_Des(2) <
        -data->_quadruped->_maxLegLength)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].p_Des(2)
                << " | modified: " << -data->_quadruped->_maxLegLength
                << std::endl;
      data->_legController->command[leg].p_Des(2) =
          -data->_quadruped->_maxLegLength;
      safePDesFoot = false;
    }
  }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
bool SafetyChecker::checkForceFeedForward()
{
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  double maxLateralForce = 0;
  double maxVerticalForce = 0;

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++)
  {
    // Limit the lateral forces in +x body frame
    if (data->_legController->command[leg].force_FF(0) >
        maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(0)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->command[leg].force_FF(0) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -x body frame
    if (data->_legController->command[leg].force_FF(0) <
        -maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 0 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(0)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->command[leg].force_FF(0) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in +y body frame
    if (data->_legController->command[leg].force_FF(1) >
        maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(1)
                << " | modified: " << maxLateralForce << std::endl;
      data->_legController->command[leg].force_FF(1) = maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the lateral forces in -y body frame
    if (data->_legController->command[leg].force_FF(1) <
        -maxLateralForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 1 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(1)
                << " | modified: " << -maxLateralForce << std::endl;
      data->_legController->command[leg].force_FF(1) =
          -maxLateralForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in +z body frame
    if (data->_legController->command[leg].force_FF(2) >
        maxVerticalForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(2)
                << " | modified: " << -maxVerticalForce << std::endl;
      data->_legController->command[leg].force_FF(2) =
          maxVerticalForce;
      safeForceFeedForward = false;
    }

    // Limit the vertical forces in -z body frame
    if (data->_legController->command[leg].force_FF(2) <
        -maxVerticalForce)
    {
      std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].force_FF(2)
                << " | modified: " << maxVerticalForce << std::endl;
      data->_legController->command[leg].force_FF(2) =
          -maxVerticalForce;
      safeForceFeedForward = false;
    }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

bool SafetyChecker::checkQDesFoot()
{
  // Assumed safe to start
  bool safeQDes = true;
  //
  // Safety parameters
  static const double max_thigh = M_PI_2;
  static const double min_thigh = -M_PI_4;
  static const double max_calf = -0.85;
  static const double min_calf = -2.60;
  // Check all of the legs
  for (int leg = 0; leg < 4; leg++)
  {

    // Keep the foot from going too far from the body in +y
    if (data->_legController->command[leg].q_Des(1) > max_thigh)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 1 << "\n";

      std::cout << "   commanded: "
                << data->_legController->command[leg].q_Des(1) 
                << " | modified: " << max_thigh
                << std::endl;
      data->_legController->command[leg].q_Des(1) = max_thigh;
      safeQDes = false;
    }

    // Keep the foot from going too far from the body in -y
    if (data->_legController->command[leg].q_Des(1) < min_thigh)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 1 << "\n";

      std::cout << "   commanded: "
                << data->_legController->command[leg].q_Des(1) 
                << " | modified: " << min_thigh
                << std::endl;
      data->_legController->command[leg].q_Des(1) = min_thigh;
      safeQDes = false;
    }

    if (data->_legController->command[leg].q_Des(2) > max_calf)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].q_Des(2)
                << " | modified: " << max_calf
                << std::endl;
      data->_legController->command[leg].q_Des(2) = max_calf;
      safeQDes = false;
    }

    if (data->_legController->command[leg].q_Des(2) < min_calf)
    {
      std::cout << "[CONTROL FSM] Safety: QDes leg: " << leg
                << " | coordinate: " << 2 << "\n";
      std::cout << "   commanded: "
                << data->_legController->command[leg].q_Des(2)
                << " | modified: " << min_calf
                << std::endl;
      data->_legController->command[leg].q_Des(2) = min_calf;
      safeQDes = false;
    }
  }

  // Return true if all desired positions are safe
  return safeQDes;
}