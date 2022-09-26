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

#include <ur_client_library/control/script_command_interface.h>

namespace urcl
{
namespace control
{
ScriptCommandInterface::ScriptCommandInterface(uint32_t port) : ReverseInterface(port, [](bool foo) { return foo; })
{
  client_connected_ = false;
}

bool ScriptCommandInterface::zeroFTSensor()
{
  const int message_length = 1;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;
  int32_t val = htobe32(toUnderlying(ScriptCommand::ZERO_FTSENSOR));
  b_pos += append(b_pos, val);

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }
  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ScriptCommandInterface::setPayload(const double mass, const vector3d_t* cog)
{
  const int message_length = 5;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;
  int32_t val = htobe32(toUnderlying(ScriptCommand::SET_PAYLOAD));
  b_pos += append(b_pos, val);

  val = htobe32(static_cast<int32_t>(mass * MULT_JOINTSTATE));
  b_pos += append(b_pos, val);

  for (auto const& center_of_mass : *cog)
  {
    val = htobe32(static_cast<int32_t>(center_of_mass * MULT_JOINTSTATE));
    b_pos += append(b_pos, val);
  }

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }
  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ScriptCommandInterface::setToolVoltage(const ToolVoltage voltage)
{
  const int message_length = 2;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;
  int32_t val = htobe32(toUnderlying(ScriptCommand::SET_TOOL_VOLTAGE));
  b_pos += append(b_pos, val);

  val = htobe32(toUnderlying(voltage) * MULT_JOINTSTATE);
  b_pos += append(b_pos, val);

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }
  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ScriptCommandInterface::clientConnected()
{
  return client_connected_;
}

void ScriptCommandInterface::connectionCallback(const int filedescriptor)
{
  if (client_fd_ < 0)
  {
    URCL_LOG_DEBUG("Robot connected to ScriptCommandInterface.");
    client_fd_ = filedescriptor;
    client_connected_ = true;
  }
  else
  {
    URCL_LOG_ERROR("Connection request to ScriptCommandInterface received while connection already established. Only "
                   "one connection is allowed at a time. Ignoring this request.");
  }
}

void ScriptCommandInterface::disconnectionCallback(const int filedescriptor)
{
  URCL_LOG_DEBUG("Connection to ScriptCommandInterface dropped.", filedescriptor);
  client_fd_ = -1;
  client_connected_ = false;
}

void ScriptCommandInterface::messageCallback(const int filedescriptor, char* buffer, int nbytesrecv)
{
  URCL_LOG_WARN("Message on script command interface received. The script command interfacee currently does not "
                "support any message handling. This message will be ignored.");
}

}  // namespace control
}  // namespace urcl