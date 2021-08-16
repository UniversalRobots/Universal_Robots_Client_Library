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

#include <ur_client_library/control/trajectory_point_interface.h>

namespace urcl
{
namespace control
{
TrajectoryPointInterface::TrajectoryPointInterface(uint32_t port) : ReverseInterface(port, [](bool foo) { return foo; })
{
}

bool TrajectoryPointInterface::writeTrajectoryPoint(const vector6d_t* positions, const float goal_time,
                                                    const float blend_radius, const bool cartesian)
{
  if (client_fd_ == -1)
  {
    return false;
  }
  uint8_t buffer[sizeof(int32_t) * 9];
  uint8_t* b_pos = buffer;

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

  int32_t val = static_cast<int32_t>(goal_time * MULT_TIME);
  val = htobe32(val);
  b_pos += append(b_pos, val);

  val = static_cast<int32_t>(blend_radius * MULT_TIME);
  val = htobe32(val);
  b_pos += append(b_pos, val);

  if (cartesian)
  {
    val = CARTESIAN_POINT;
  }
  else
  {
    val = JOINT_POINT;
  }

  val = htobe32(val);
  b_pos += append(b_pos, val);

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

void TrajectoryPointInterface::connectionCallback(const int filedescriptor)
{
  if (client_fd_ < 0)
  {
    URCL_LOG_DEBUG("Robot connected to trajectory interface.");
    client_fd_ = filedescriptor;
  }
  else
  {
    URCL_LOG_ERROR("Connection request to TrajectoryPointInterface received while connection already established. Only "
                   "one connection is allowed at a time. Ignoring this request.");
  }
}

void TrajectoryPointInterface::disconnectionCallback(const int filedescriptor)
{
  URCL_LOG_DEBUG("Connection to trajectory interface dropped.", filedescriptor);
  client_fd_ = -1;
}

void TrajectoryPointInterface::messageCallback(const int filedescriptor, char* buffer, int nbytesrecv)
{
  if (nbytesrecv == 4)
  {
    int32_t* status = reinterpret_cast<int*>(buffer);
    URCL_LOG_DEBUG("Received message %d on TrajectoryPointInterface", be32toh(*status));

    if (handle_trajectory_end_)
    {
      handle_trajectory_end_(static_cast<TrajectoryResult>(be32toh(*status)));
    }
    else
    {
      URCL_LOG_DEBUG("Trajectory execution finished with result %d, but no callback was given.");
    }
  }
  else
  {
    URCL_LOG_WARN("Received %d bytes on TrajectoryPointInterface. Expecting 4 bytes, so ignoring this message",
                  nbytesrecv);
  }
}
}  // namespace control
}  // namespace urcl
