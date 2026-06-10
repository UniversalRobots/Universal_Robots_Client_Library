#include "ur_client_library/ur/datatypes.h"
#include "ur_client_library/log.h"

#include <string>

namespace urcl
{
namespace control
{

struct PDControllerGains
{
  vector6d_t kp;
  vector6d_t kd;
};

// UR3 PD controller gains, needs to be tuned for the specific purpose.
constexpr PDControllerGains UR3_PD_CONTROLLER_PARAMETERS_JOINT_SPACE{
  /*.kp*/ { 560.0, 560.0, 350.0, 163.0, 163.0, 163.0 },
  /*.kd*/ { 47.32, 47.32, 37.42, 25.5, 25.5, 25.5 }
};

// UR5 PD controller gains, needs to be tuned for the specific purpose.
constexpr PDControllerGains UR5_PD_CONTROLLER_PARAMETERS_JOINT_SPACE{
  /*.kp*/ { 900.0, 900.0, 900.0, 500.0, 500.0, 500.0 },
  /*.kd*/ { 60.0, 60.0, 60.0, 44.72, 44.72, 44.72 }
};

// UR10 PD controller gains, needs to be tuned for the specific purpose.
constexpr PDControllerGains UR10_PD_CONTROLLER_PARAMETERS_JOINT_SPACE{
  /*.kp*/ { 1300.0, 1300.0, 900.0, 560.0, 560.0, 560.0 },
  /*.kd*/ { 72.11, 72.11, 60.0, 47.32, 47.32, 47.32 }
};

constexpr vector6d_t MAX_UR3_JOINT_TORQUE = { 54.0, 54.0, 28.0, 9.0, 9.0, 9.0 };

constexpr vector6d_t MAX_UR5_JOINT_TORQUE = { 150.0, 150.0, 150.0, 28.0, 28.0, 28.0 };

constexpr vector6d_t MAX_UR10_JOINT_TORQUE = { 330.0, 330.0, 150.0, 54.0, 54.0, 54.0 };

/*!
 * \brief Get pd gains for specific robot type
 *
 * \param robot_type Robot type to get gains for
 *
 * \returns PD gains for the specific robot type
 */
inline PDControllerGains getPdGainsFromRobotType(RobotType robot_type)
{
  switch (robot_type)
  {
    case RobotType::UR3:
      return UR3_PD_CONTROLLER_PARAMETERS_JOINT_SPACE;
    case RobotType::UR5:
      return UR5_PD_CONTROLLER_PARAMETERS_JOINT_SPACE;
    case RobotType::UR10:
      return UR10_PD_CONTROLLER_PARAMETERS_JOINT_SPACE;
    default:
      std::stringstream ss;
      ss << "Default gains has not been tuned for robot type " << robotTypeString(robot_type)
         << ". Defaulting to UR10 gains.";
      URCL_LOG_WARN(ss.str().c_str());
      return UR10_PD_CONTROLLER_PARAMETERS_JOINT_SPACE;
  }
}

/*!
 * \brief Get max torques specific robot type
 *
 * \param robot_type Robot type to get max torque for
 *
 * \returns max torque for the specific robot type
 */
inline vector6d_t getMaxTorquesFromRobotType(RobotType robot_type)
{
  switch (robot_type)
  {
    case RobotType::UR3:
      return MAX_UR3_JOINT_TORQUE;
    case RobotType::UR5:
      return MAX_UR5_JOINT_TORQUE;
    case RobotType::UR10:
      return MAX_UR10_JOINT_TORQUE;
    case RobotType::UR16:
      // Same joints as ur10
      return MAX_UR10_JOINT_TORQUE;
    default:
      std::stringstream ss;
      ss << "Max joint torques not collected for robot type " << robotTypeString(robot_type)
         << ". Defaulting to UR10 max joint torques.";
      URCL_LOG_WARN(ss.str().c_str());
      return MAX_UR10_JOINT_TORQUE;
  }
}

}  // namespace control
}  // namespace urcl