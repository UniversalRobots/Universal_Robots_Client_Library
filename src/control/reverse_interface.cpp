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
 * \author  Felix Exner exner@fzi.de
 * \date    2021-06-01
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/control/reverse_interface.h>
#include <math.h>

namespace urcl
{
namespace control
{
ReverseInterface::ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state,
                                   std::chrono::milliseconds step_time)
  : ReverseInterface(ReverseInterfaceConfig{ port, handle_program_state, step_time })
{
}

ReverseInterface::ReverseInterface(const ReverseInterfaceConfig& config)
  : client_fd_(INVALID_SOCKET)
  , server_(config.port)
  , robot_software_version_(config.robot_software_version)
  , handle_program_state_(config.handle_program_state)
  , step_time_(config.step_time)
  , keep_alive_count_modified_deprecated_(false)
{
  handle_program_state_(false);
  server_.setMessageCallback(std::bind(&ReverseInterface::messageCallback, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3));
  server_.setConnectCallback(std::bind(&ReverseInterface::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&ReverseInterface::disconnectionCallback, this, std::placeholders::_1));
  server_.setMaxClientsAllowed(1);
  server_.start();
}

bool ReverseInterface::write(const vector6d_t* positions, const comm::ControlMode control_mode,
                             const RobotReceiveTimeout& robot_receive_timeout)
{
  const int message_length = 7;
  if (client_fd_ == INVALID_SOCKET)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int read_timeout = 100;
  // If control mode is stopped, we shouldn't verify robot receive timeout
  if (control_mode != comm::ControlMode::MODE_STOPPED)
  {
    read_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(control_mode, step_time_);
  }

  // This can be removed once we remove the setkeepAliveCount() method
  auto read_timeout_resolved = read_timeout;
  if (keep_alive_count_modified_deprecated_)
  {
    // Translate keep alive count into read timeout. 20 milliseconds was the "old read timeout"
    read_timeout_resolved = 20 * keepalive_count_;
  }

  // The first element is always the read timeout.
  int32_t val = read_timeout_resolved;
  val = htobe32(val);
  b_pos += append(b_pos, val);

  if (positions != nullptr)
  {
    for (auto const& pos : *positions)
    {
      int32_t val = static_cast<int32_t>(round(pos * MULT_JOINTSTATE));
      val = htobe32(val);
      b_pos += append(b_pos, val);
    }
  }
  else
  {
    b_pos += 6 * sizeof(int32_t);
  }

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH - 1; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }

  val = htobe32(toUnderlying(control_mode));
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ReverseInterface::writeTrajectoryControlMessage(const TrajectoryControlMessage trajectory_action,
                                                     const int point_number,
                                                     const RobotReceiveTimeout& robot_receive_timeout)
{
  const int message_length = 3;
  if (client_fd_ == INVALID_SOCKET)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int read_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlMode::MODE_FORWARD, step_time_);

  // This can be removed once we remove the setkeepAliveCount() method
  auto read_timeout_resolved = read_timeout;
  if (keep_alive_count_modified_deprecated_)
  {
    // Translate keep alive count into read timeout. 20 milliseconds was the "old read timeout"
    read_timeout_resolved = 20 * keepalive_count_;
  }

  // The first element is always the read timeout.
  int32_t val = read_timeout_resolved;
  val = htobe32(val);
  b_pos += append(b_pos, val);

  val = htobe32(toUnderlying(trajectory_action));
  b_pos += append(b_pos, val);

  val = htobe32(point_number);
  b_pos += append(b_pos, val);

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH - 1; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }

  val = htobe32(toUnderlying(comm::ControlMode::MODE_FORWARD));
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ReverseInterface::writeFreedriveControlMessage(const FreedriveControlMessage freedrive_action,
                                                    const RobotReceiveTimeout& robot_receive_timeout)
{
  const int message_length = 2;
  if (client_fd_ == INVALID_SOCKET)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * MAX_MESSAGE_LENGTH];
  uint8_t* b_pos = buffer;

  int read_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlMode::MODE_FREEDRIVE, step_time_);

  // This can be removed once we remove the setkeepAliveCount() method
  auto read_timeout_resolved = read_timeout;
  if (keep_alive_count_modified_deprecated_)
  {
    // Translate keep alive count into read timeout. 20 milliseconds was the "old read timeout"
    read_timeout_resolved = 20 * keepalive_count_;
  }

  // The first element is always the read timeout.
  int32_t val = read_timeout_resolved;
  val = htobe32(val);
  b_pos += append(b_pos, val);

  val = htobe32(toUnderlying(freedrive_action));
  b_pos += append(b_pos, val);

  // writing zeros to allow usage with other script commands
  for (size_t i = message_length; i < MAX_MESSAGE_LENGTH - 1; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }

  val = htobe32(toUnderlying(comm::ControlMode::MODE_FREEDRIVE));
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

void ReverseInterface::setKeepaliveCount(const uint32_t count)
{
  URCL_LOG_WARN("DEPRECATION NOTICE: Setting the keepalive count has been deprecated. Instead you should set the "
                "timeout directly in the write commands. Please change your code to set the read timeout in the write "
                "commands "
                "directly. This keepalive count will overwrite the timeout passed to the write functions.");
  keepalive_count_ = count;
  keep_alive_count_modified_deprecated_ = true;
}

void ReverseInterface::connectionCallback(const socket_t filedescriptor)
{
  if (client_fd_ == INVALID_SOCKET)
  {
    URCL_LOG_INFO("Robot connected to reverse interface. Ready to receive control commands.");
    client_fd_ = filedescriptor;
    handle_program_state_(true);
  }
  else
  {
    URCL_LOG_ERROR("Connection request to ReverseInterface received while connection already established. Only one "
                   "connection is allowed at a time. Ignoring this request.");
  }
}

void ReverseInterface::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_INFO("Connection to reverse interface dropped.", filedescriptor);
  client_fd_ = INVALID_SOCKET;
  handle_program_state_(false);
  for (auto handler : disconnect_callbacks_)
  {
    handler.function(filedescriptor);
  }
}

void ReverseInterface::messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv)
{
  URCL_LOG_WARN("Message on ReverseInterface received. The reverse interface currently does not support any message "
                "handling. This message will be ignored.");
}

}  // namespace control
}  // namespace urcl
