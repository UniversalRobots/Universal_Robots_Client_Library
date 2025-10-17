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
#include <math.h>

namespace urcl
{
namespace control
{
ScriptCommandInterface::ScriptCommandInterface(uint32_t port) : ScriptCommandInterface(ReverseInterfaceConfig{ port })
{
}

ScriptCommandInterface::ScriptCommandInterface(const ReverseInterfaceConfig& config) : ReverseInterface(config)
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

  val = htobe32(static_cast<int32_t>(round(mass * MULT_JOINTSTATE)));
  b_pos += append(b_pos, val);

  for (auto const& center_of_mass : *cog)
  {
    val = htobe32(static_cast<int32_t>(round(center_of_mass * MULT_JOINTSTATE)));
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

bool ScriptCommandInterface::startForceMode(const vector6d_t* task_frame, const vector6uint32_t* selection_vector,
                                            const vector6d_t* wrench, const unsigned int type, const vector6d_t* limits,
                                            double damping_factor, double gain_scaling_factor)
{
  const int message_length = 28;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::START_FORCE_MODE));
  b_pos += append(b_pos, val);

  for (auto const& frame : *task_frame)
  {
    val = htobe32(static_cast<int32_t>(round(frame * MULT_JOINTSTATE)));
    b_pos += append(b_pos, val);
  }

  for (auto const& selection : *selection_vector)
  {
    val = htobe32(static_cast<int32_t>(selection * MULT_JOINTSTATE));
    b_pos += append(b_pos, val);
  }

  for (auto const& force_torque : *wrench)
  {
    val = htobe32(static_cast<int32_t>(round(force_torque * MULT_JOINTSTATE)));
    b_pos += append(b_pos, val);
  }

  val = htobe32(static_cast<int32_t>(type * MULT_JOINTSTATE));
  b_pos += append(b_pos, val);

  for (auto const& lim : *limits)
  {
    val = htobe32(static_cast<int32_t>(round(lim * MULT_JOINTSTATE)));
    b_pos += append(b_pos, val);
  }

  val = htobe32(static_cast<int32_t>(round(damping_factor * MULT_JOINTSTATE)));
  b_pos += append(b_pos, val);

  val = htobe32(static_cast<int32_t>(round(gain_scaling_factor * MULT_JOINTSTATE)));
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

bool ScriptCommandInterface::endForceMode()
{
  const int message_length = 1;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::END_FORCE_MODE));
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

bool ScriptCommandInterface::startToolContact()
{
  const int message_length = 1;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::START_TOOL_CONTACT));
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

bool ScriptCommandInterface::endToolContact()
{
  const int message_length = 1;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::END_TOOL_CONTACT));
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

bool ScriptCommandInterface::setFrictionCompensation(const bool friction_compensation_enabled)
{
  if (!robotVersionSupportsCommandOrWarn(urcl::VersionInformation::fromString("5.23.0"),
                                         urcl::VersionInformation::fromString("10.10.0"), __func__))
  {
    return false;
  }
  const int message_length = 2;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::SET_FRICTION_COMPENSATION));
  b_pos += append(b_pos, val);

  val = htobe32(friction_compensation_enabled);
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

bool ScriptCommandInterface::ftRtdeInputEnable(const bool enabled, const double sensor_mass,
                                               const vector3d_t& sensor_measuring_offset, const vector3d_t& sensor_cog)
{
  const int message_length = 9;
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int32_t val = htobe32(toUnderlying(ScriptCommand::FT_RTDE_INPUT_ENABLE));
  b_pos += append(b_pos, val);

  val = htobe32(enabled);
  b_pos += append(b_pos, val);

  val = htobe32(static_cast<int32_t>(round(sensor_mass * MULT_JOINTSTATE)));
  b_pos += append(b_pos, val);

  for (auto const& mass_component : sensor_measuring_offset)
  {
    val = htobe32(static_cast<int32_t>(round(mass_component * MULT_JOINTSTATE)));
    b_pos += append(b_pos, val);
  }

  for (auto const& cog_component : sensor_cog)
  {
    val = htobe32(static_cast<int32_t>(round(cog_component * MULT_JOINTSTATE)));
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

bool ScriptCommandInterface::clientConnected()
{
  return client_connected_;
}

void ScriptCommandInterface::connectionCallback(const socket_t filedescriptor)
{
  if (client_fd_ == INVALID_SOCKET)
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

void ScriptCommandInterface::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_DEBUG("Connection to ScriptCommandInterface dropped.", filedescriptor);
  client_fd_ = INVALID_SOCKET;
  client_connected_ = false;
}

void ScriptCommandInterface::messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv)
{
  if (nbytesrecv == 4)
  {
    int32_t* status = reinterpret_cast<int*>(buffer);
    URCL_LOG_DEBUG("Received message %d on Script command interface", be32toh(*status));

    if (handle_tool_contact_result_)
    {
      handle_tool_contact_result_(static_cast<ToolContactResult>(be32toh(*status)));
    }
    else
    {
      URCL_LOG_DEBUG("Tool contact execution finished with result %d, but no callback was given.", be32toh(*status));
    }
  }
  else
  {
    URCL_LOG_WARN("Received %d bytes on script command interface. Expecting 4 bytes, so ignoring this message",
                  nbytesrecv);
  }
}

bool ScriptCommandInterface::robotVersionSupportsCommandOrWarn(const VersionInformation& min_polyscope5,
                                                               const VersionInformation& min_polyscopeX,
                                                               const std::string& command_name)
{
  if (robot_software_version_ < min_polyscope5 ||
      (robot_software_version_.major > 5 && robot_software_version_ < min_polyscopeX))
  {
    URCL_LOG_WARN("%s is only available for robots with PolyScope %s / %s or "
                  "later. This robot's version is %s. This command will have no effect.",
                  command_name.c_str(), min_polyscope5.toString().c_str(), min_polyscopeX.toString().c_str(),
                  robot_software_version_.toString().c_str());
    return false;
  }
  return true;
}

}  // namespace control
}  // namespace urcl