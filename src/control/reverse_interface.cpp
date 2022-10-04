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

namespace urcl
{
namespace control
{
ReverseInterface::ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state)
  : client_fd_(-1), server_(port), handle_program_state_(handle_program_state), keepalive_count_(1)
{
  handle_program_state_(false);
  server_.setMessageCallback(std::bind(&ReverseInterface::messageCallback, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3));
  server_.setConnectCallback(std::bind(&ReverseInterface::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&ReverseInterface::disconnectionCallback, this, std::placeholders::_1));
  server_.setMaxClientsAllowed(1);
  server_.start();
}

bool ReverseInterface::write(const vector6d_t* positions, const comm::ControlMode control_mode)
{
  if (client_fd_ == -1)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * 8];
  uint8_t* b_pos = buffer;

  // The first element is always the keepalive signal.
  int32_t val = htobe32(keepalive_count_);
  b_pos += append(b_pos, val);

  if (positions != nullptr)
  {
    for (auto const& pos : *positions)
    {
      int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE);
      val = htobe32(val);
      b_pos += append(b_pos, val);
    }
  }
  else
  {
    b_pos += 6 * sizeof(int32_t);
  }

  val = htobe32(toUnderlying(control_mode));
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

bool ReverseInterface::writeTrajectoryControlMessage(const TrajectoryControlMessage trajectory_action,
                                                     const int point_number)
{
  if (client_fd_ == -1)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * 8];
  uint8_t* b_pos = buffer;

  // The first element is always the keepalive signal.
  int32_t val = htobe32(keepalive_count_);
  b_pos += append(b_pos, val);

  val = htobe32(toUnderlying(trajectory_action));
  b_pos += append(b_pos, val);

  val = htobe32(point_number);
  b_pos += append(b_pos, val);

  // writing zeros to allow usage in control loop with other control messages
  for (size_t i = 0; i < 4; i++)
  {
    val = htobe32(0);
    b_pos += append(b_pos, val);
  }

  val = htobe32(toUnderlying(comm::ControlMode::MODE_FORWARD));
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

void ReverseInterface::connectionCallback(const int filedescriptor)
{
  if (client_fd_ < 0)
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

void ReverseInterface::disconnectionCallback(const int filedescriptor)
{
  URCL_LOG_INFO("Connection to reverse interface dropped.", filedescriptor);
  client_fd_ = -1;
  handle_program_state_(false);
}

void ReverseInterface::messageCallback(const int filedescriptor, char* buffer, int nbytesrecv)
{
  URCL_LOG_WARN("Message on ReverseInterface received. The reverse interface currently does not support any message "
                "handling. This message will be ignored.");
}
}  // namespace control
}  // namespace urcl
