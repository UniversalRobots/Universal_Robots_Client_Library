// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2021-06-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_CONTROL_MODE_H_INCLUDED
#define UR_CLIENT_LIBRARY_CONTROL_MODE_H_INCLUDED

#include <algorithm>
#include <vector>

namespace urcl
{
namespace comm
{
/*!
 * \brief Control modes as interpreted from the script runnning on the robot.
 */
enum class ControlMode : int32_t
{
  MODE_STOPPED = -2,        ///< When this is set, the program is expected to stop and exit.
  MODE_UNINITIALIZED = -1,  ///< Startup default until another mode is sent to the script.
  MODE_IDLE = 0,            ///< Set when no controller is currently active controlling the robot.
  MODE_SERVOJ = 1,          ///< Set when servoj control is active.
  MODE_SPEEDJ = 2,          ///< Set when speedj control is active.
  MODE_FORWARD = 3,         ///< Set when trajectory forwarding is active.
  MODE_SPEEDL = 4,          ///< Set when cartesian velocity control is active.
  MODE_POSE = 5,            ///< Set when cartesian pose control is active.
  MODE_FREEDRIVE = 6,       ///< Set when freedrive mode is active.
  MODE_TOOL_IN_CONTACT =
      7,            ///< Used only internally in the script, when robot is in tool contact, clear by endToolContact()
  MODE_TORQUE = 8,  ///< Set when torque control is active.
  END               ///< This is not an actual control mode, but used internally to get the number of control modes
};

/*!
 * \brief Class used to separate the control modes into realtime and non realtime.
 */
class ControlModeTypes
{
public:
  // Control modes that require realtime communication
  static const inline std::vector<ControlMode> REALTIME_CONTROL_MODES = {
    ControlMode::MODE_SERVOJ, ControlMode::MODE_SPEEDJ, ControlMode::MODE_SPEEDL, ControlMode::MODE_POSE,
    ControlMode::MODE_TORQUE
  };

  // Control modes that doesn't require realtime communication
  static const inline std::vector<ControlMode> NON_REALTIME_CONTROL_MODES = { ControlMode::MODE_IDLE,
                                                                              ControlMode::MODE_FORWARD,
                                                                              ControlMode::MODE_FREEDRIVE };

  // Control modes which doesn't move the robot, meaning they are neither realtime or non realtime
  static const inline std::vector<ControlMode> STATIONARY_CONTROL_MODES = { ControlMode::MODE_STOPPED,
                                                                            ControlMode::MODE_UNINITIALIZED,
                                                                            ControlMode::MODE_TOOL_IN_CONTACT };

  /*!
   * \brief Check if the control mode is realtime
   *
   * \param control_mode Current control mode
   *
   * \returns true if the control mode is realtime, false otherwise
   */
  static bool isControlModeRealtime(ControlMode control_mode)
  {
    return (std::find(ControlModeTypes::REALTIME_CONTROL_MODES.begin(), ControlModeTypes::REALTIME_CONTROL_MODES.end(),
                      control_mode) != ControlModeTypes::REALTIME_CONTROL_MODES.end());
  }

  /*!
   * \brief Check if the control mode is non realtime
   *
   * \param control_mode Current control mode
   *
   * \returns true if the control mode is non realtime, false otherwise
   */
  static bool isControlModeNonRealtime(ControlMode control_mode)
  {
    return (std::find(ControlModeTypes::NON_REALTIME_CONTROL_MODES.begin(),
                      ControlModeTypes::NON_REALTIME_CONTROL_MODES.end(),
                      control_mode) != ControlModeTypes::NON_REALTIME_CONTROL_MODES.end());
  }
};

}  // namespace comm
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_CONTROL_MODE_H_INCLUDED
