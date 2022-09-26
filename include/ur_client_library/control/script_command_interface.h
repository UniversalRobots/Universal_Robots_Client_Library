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
 * \brief The ScriptCommandInterface class starts a TCPServer for a robot to connect to and this connection is then used
 * to forward script commands to the robot, which will be executed locally on the robot.
 *
 * The script commands will be executed in a seperate thread in the external control script.
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
   * \brief Returns whether a client/robot is connected to this server.
   */
  bool clientConnected();

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

    ZERO_FTSENSOR = 0,     ///< Zero force torque sensor
    SET_PAYLOAD = 1,       ///< Set payload
    SET_TOOL_VOLTAGE = 2,  ///< Set tool voltage
  };

  bool client_connected_;
  static const int MAX_MESSAGE_LENGTH = 5;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_SCRIPT_COMMAND_INTERFACE_H_INCLUDED