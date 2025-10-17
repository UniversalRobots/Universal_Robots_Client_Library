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
 * \author  Mads Holm Peters mahp@universal-robots.com
 * \date    2022-08-12
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_SCRIPT_COMMAND_INTERFACE_H_INCLUDED
#define UR_CLIENT_LIBRARY_SCRIPT_COMMAND_INTERFACE_H_INCLUDED

#include "ur_client_library/control/reverse_interface.h"
#include "ur_client_library/ur/tool_communication.h"
#include "ur_client_library/ur/version_information.h"

namespace urcl
{
namespace control
{
/*!
 * \brief Types for encoding until tool contact execution result.
 */
enum class ToolContactResult : int32_t
{

  UNTIL_TOOL_CONTACT_RESULT_SUCCESS = 0,   ///< Successful execution
  UNTIL_TOOL_CONTACT_RESULT_CANCELED = 1,  ///< Canceled by user
};

/*!
 * \brief The ScriptCommandInterface class starts a TCPServer for a robot to connect to and this connection is then used
 * to forward script commands to the robot, which will be executed locally on the robot.
 *
 * The script commands will be executed in a separate thread in the external control script.
 */
class ScriptCommandInterface : public ReverseInterface
{
public:
  ScriptCommandInterface() = delete;
  /*!
   * \brief Creates a ScriptCommandInterface object, including a new TCPServer
   *
   * \param port Port to start the server on
   */
  [[deprecated("Use ReverseInterfaceConfig instead of port, handle_program_state and step_time parameters")]]
  ScriptCommandInterface(uint32_t port);

  /*!
   * \brief Creates a ScriptCommandInterface object, including a new TCPServer
   *
   * \param config Configuration for the ReverseInterface, including the port to start the server
   * on.
   */
  ScriptCommandInterface(const ReverseInterfaceConfig& config);

  /*!
   * \brief Zero the force torque sensor
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool zeroFTSensor();

  /*!
   * \brief Set the active payload mass and center of gravity
   *
   * \param mass mass in kilograms
   * \param cog  Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the displacement (in meters) from the
   * toolmount
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool setPayload(const double mass, const vector3d_t* cog);

  /*!
   * \brief Set the tool voltage.
   *
   * \param voltage Tool voltage
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool setToolVoltage(const ToolVoltage voltage);

  /*!
   * \brief Set robot to be controlled in force mode.
   *
   * \param task_frame A pose vector that defines the force frame relative to the base frame
   * \param selection_vector A 6d vector of 0s and 1s. 1 means that the robot will be compliant in the corresponding
   * axis of the task frame
   * \param wrench The forces/torques the robot will apply to its environment. The robot adjusts its position
   * along/about compliant axis in order to achieve the specified force/torque. Values have no effect for non-compliant
   * axes
   * \param type An integer [1;3] specifying how the robot interprets the force frame
   *  1: The force frame is transformed in a way such that its y-axis is aligned with a vector pointing from the robot
   *  tcp towards the origin of the force frame
   *  2: The force frame is not transformed.
   *  3: The force frame is transformed in a way such that its x-axis is the projection of the robot tcp velocity vector
   *  onto the x-y plane of the force frame
   * \param limits 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the
   * axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual
   * tcp position and the one set by the program
   *
   * \param damping_factor Sets the damping parameter in force mode. In range [0,1], default value is 0.025
   * A value of 1 is full damping, so the robot will decelerate quickly if no force is present. A value of 0
   * is no damping, here the robot will maintain the speed.
   *
   * \param gain_scaling_factor Scales the gain in force mode. scaling parameter is in range [0,2], default
   * is 0.5. A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard
   * surfaces.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool startForceMode(const vector6d_t* task_frame, const vector6uint32_t* selection_vector, const vector6d_t* wrench,
                      const unsigned int type, const vector6d_t* limits, double damping_factor,
                      double gain_scaling_factor);

  /*!
   * \brief Stop force mode and put the robot into normal operation mode.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool endForceMode();

  /*!
   * \brief This will make the robot look for tool contact in the tcp directions that the robot is currently
   * moving. Once a tool contact has been detected all movements will be canceled. Call endToolContact to enable
   * movements again.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool startToolContact();

  /*!
   * \brief This will stop the robot from looking for a tool contact, it will also enable sending move commands to the
   * robot again if the robot's tool is in contact
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool endToolContact();

  /*!
   * \brief Set friction compensation for the torque_command. If true the torque command will compensate for friction,
   * if false it will not.
   *
   * \param friction_compensation_enabled Will set a friction_compensation_enabled variable in urscript, which will be
   * used when calling torque_command
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool setFrictionCompensation(const bool friction_compensation_enabled);

  /*!
   * \brief Enable or disable RTDE input for the force torque sensor.
   *
   * When enabled, the force torque sensor values will be read from the RTDE input registers instead of the actual
   * sensor. This can be used to connect an external force torque sensor to the robot or to
   * simulate a force torque sensor when using URSim.
   *
   * \param enabled Whether to enable or disable RTDE input for the force torque sensor.
   * \param sensor_mass Mass of the force torque sensor in kilograms.
   * \param sensor_measuring_offset The offset of the force torque sensor in meters, a vector [x, y, z] specifying the
   * displacement from the tool flange frame.
   * \param sensor_cog The center of gravity of the force torque sensor in meters, a vector [x, y, z] specifying the
   * displacement from the tool flange frame.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool ftRtdeInputEnable(const bool enabled, const double sensor_mass = 0.0,
                         const vector3d_t& sensor_measuring_offset = { 0.0, 0.0, 0.0 },
                         const vector3d_t& sensor_cog = { 0.0, 0.0, 0.0 });

  /*!
   * \brief  Returns whether a client/robot is connected to this server.
   *
   */
  bool clientConnected();

  /*!
   * \brief Set the tool contact result callback object
   *
   * \param callback Callback function that will be triggered when the robot enters tool contact
   */
  void setToolContactResultCallback(std::function<void(ToolContactResult)> callback)
  {
    handle_tool_contact_result_ = callback;
  }

protected:
  virtual void connectionCallback(const socket_t filedescriptor) override;

  virtual void disconnectionCallback(const socket_t filedescriptor) override;

  virtual void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv) override;

private:
  /*!
   * \brief Available script commands
   */
  enum class ScriptCommand : int32_t
  {

    ZERO_FTSENSOR = 0,              ///< Zero force torque sensor
    SET_PAYLOAD = 1,                ///< Set payload
    SET_TOOL_VOLTAGE = 2,           ///< Set tool voltage
    START_FORCE_MODE = 3,           ///< Start force mode
    END_FORCE_MODE = 4,             ///< End force mode
    START_TOOL_CONTACT = 5,         ///< Start detecting tool contact
    END_TOOL_CONTACT = 6,           ///< End detecting tool contact
    SET_FRICTION_COMPENSATION = 7,  ///< Set friction compensation
    FT_RTDE_INPUT_ENABLE = 8,       ///< Enable FT RTDE input
  };

  /*!
   * \brief Checks if the robot version is higher than the minimum required version for Polyscope 5
   * or Polyscope X.
   *
   * If the robot version is lower than the minimum required version, this function
   * will log a warning message.
   * In case of a PolyScope 5 robot, the robot's software version will be checked against \p
   * min_polyscope5, and in case of a PolyScope X robot, it will be checked against \p
   * min_polyscopeX.
   *
   * \param min_polyscope5 Minimum required version for PolyScope 5
   * \param min_polyscopeX Minimum required version for PolyScope X
   * \param command_name Name of the command being checked, used for logging
   *
   * \returns True if the robot version is higher than the versions provided, false otherwise.
   */
  bool robotVersionSupportsCommandOrWarn(const VersionInformation& min_polyscope5,
                                         const VersionInformation& min_polyscopeX, const std::string& command_name);

  bool client_connected_;
  static const int MAX_MESSAGE_LENGTH = 28;

  std::function<void(ToolContactResult)> handle_tool_contact_result_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_SCRIPT_COMMAND_INTERFACE_H_INCLUDED
