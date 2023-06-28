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
  ScriptCommandInterface(uint32_t port);

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
   * \param limits (Float) 6d vector. For compliant axes, these values are the maximum allowed tcp speed along/about the
   * axis. For non-compliant axes, these values are the maximum allowed deviation along/about an axis between the actual
   * tcp position and the one set by the program
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool startForceMode(const vector6d_t* task_frame, const vector6uint32_t* selection_vector, const vector6d_t* wrench,
                      const unsigned int type, const vector6d_t* limits);

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
  virtual void connectionCallback(const int filedescriptor) override;

  virtual void disconnectionCallback(const int filedescriptor) override;

  virtual void messageCallback(const int filedescriptor, char* buffer, int nbytesrecv) override;

private:
  /*!
   * \brief Available script commands
   */
  enum class ScriptCommand : int32_t
  {

    ZERO_FTSENSOR = 0,       ///< Zero force torque sensor
    SET_PAYLOAD = 1,         ///< Set payload
    SET_TOOL_VOLTAGE = 2,    ///< Set tool voltage
    START_FORCE_MODE = 3,    ///< Start force mode
    END_FORCE_MODE = 4,      ///< End force mode
    START_TOOL_CONTACT = 5,  ///< Start detecting tool contact
    END_TOOL_CONTACT = 6,    ///< End detecting tool contact
  };

  bool client_connected_;
  static const int MAX_MESSAGE_LENGTH = 26;

  std::function<void(ToolContactResult)> handle_tool_contact_result_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_SCRIPT_COMMAND_INTERFACE_H_INCLUDED