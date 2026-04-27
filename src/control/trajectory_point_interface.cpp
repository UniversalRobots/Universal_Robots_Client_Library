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
#include <ur_client_library/exceptions.h>
#include <urcl_3rdparty/portable_endian.h>
#include <math.h>
#include <cstdint>
#include <stdexcept>
#include "ur_client_library/comm/socket_t.h"
#include "ur_client_library/control/motion_primitives.h"

namespace urcl
{
namespace control
{
namespace
{
// Flatten a MotionTarget into a 6-element block, regardless of whether it holds a Q or a Pose.
vector6d_t motionTargetToBlock(const urcl::MotionTarget& target)
{
  return std::visit(
      [](const auto& alternative) -> vector6d_t {
        using T = std::decay_t<decltype(alternative)>;
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          return { alternative.x, alternative.y, alternative.z, alternative.rx, alternative.ry, alternative.rz };
        }
        else
        {
          static_assert(std::is_same_v<T, urcl::Q>, "Unhandled MotionTarget alternative");
          return { alternative.values[0], alternative.values[1], alternative.values[2],
                   alternative.values[3], alternative.values[4], alternative.values[5] };
        }
      },
      target);
}
}  // namespace

std::string trajectoryResultToString(const TrajectoryResult result)
{
  switch (result)
  {
    case TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN:
      return "UNKNOWN";
    case TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
      return "SUCCESS";
    case TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
      return "CANCELED";
    case TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
      return "FAILURE";
    default:
      throw std::invalid_argument("Illegal Trajectory result");
  }
}

TrajectoryPointInterface::TrajectoryPointInterface(uint32_t port) : ReverseInterface(ReverseInterfaceConfig{ port })
{
}

bool TrajectoryPointInterface::writeMotionPrimitive(const std::shared_ptr<control::MotionPrimitive> primitive)
{
  if (!primitive->validate())
  {
    URCL_LOG_ERROR("Motion primitive validation failed.");
    return false;
  }

  if (primitive->duration.count() > MAX_GOAL_TIME_)
  {
    URCL_LOG_ERROR("Motion primitive duration is too long. Maximum allowed duration is %f seconds.", MAX_GOAL_TIME_);
    return false;
  }

  if (client_fd_ == -1)
  {
    return false;
  }
  std::array<int32_t, MESSAGE_LENGTH> buffer;

  // We write three blocks of 6 doubles and some additional data
  vector6d_t first_block = { 0, 0, 0, 0, 0, 0 };
  vector6d_t second_block = { 0, 0, 0, 0, 0, 0 };
  vector6d_t third_block = { 0, 0, 0, 0, 0, 0 };

  switch (primitive->type)
  {
    case MotionType::MOVEJ:
    case MotionType::MOVEJ_POSE:
    case MotionType::MOVEL:
    case MotionType::MOVEL_JOINT:
    case MotionType::MOVEP:
    case MotionType::MOVEP_JOINT:
    case MotionType::OPTIMOVEJ:
    case MotionType::OPTIMOVEJ_POSE:
    case MotionType::OPTIMOVEL:
    case MotionType::OPTIMOVEL_JOINT:
    {
      auto with_target = std::static_pointer_cast<control::MotionPrimitiveWithTarget>(primitive);
      first_block = motionTargetToBlock(with_target->getTarget());
      second_block.fill(primitive->velocity);
      third_block.fill(primitive->acceleration);
      break;
    }
    case MotionType::MOVEC:
    case MotionType::MOVEC_JOINT:
    case MotionType::MOVEC_POSE_JOINT:
    case MotionType::MOVEC_JOINT_POSE:
    {
      auto movec_primitive = std::static_pointer_cast<control::MoveCPrimitive>(primitive);
      first_block = motionTargetToBlock(movec_primitive->getTarget());
      second_block = motionTargetToBlock(movec_primitive->getVia());
      third_block = {
        primitive->velocity, primitive->acceleration, static_cast<double>(movec_primitive->mode), 0, 0, 0
      };
      break;
    }
    case control::MotionType::SPLINE:
    {
      auto spline_primitive = std::static_pointer_cast<control::SplinePrimitive>(primitive);
      first_block = spline_primitive->target_positions;
      second_block = spline_primitive->target_velocities;
      if (spline_primitive->target_accelerations.has_value())
      {
        third_block = spline_primitive->target_accelerations.value();
      }
      break;
    }
    default:
      throw UnsupportedMotionType();
  }

  size_t index = 0;
  for (auto const& pos : first_block)
  {
    int32_t val = static_cast<int32_t>(round(pos * MULT_JOINTSTATE));
    buffer[index] = htobe32(val);
    index++;
  }
  for (auto const& item : second_block)
  {
    int32_t val = static_cast<int32_t>(round(item * MULT_JOINTSTATE));
    buffer[index] = htobe32(val);
    index++;
  }
  for (auto const& item : third_block)
  {
    int32_t val = static_cast<int32_t>(round(item * MULT_JOINTSTATE));
    buffer[index] = htobe32(val);
    index++;
  }

  int32_t val = static_cast<int32_t>(round(primitive->duration.count() * MULT_TIME));
  buffer[index] = htobe32(val);
  index++;

  if (primitive->type == MotionType::SPLINE)
  {
    val = static_cast<int32_t>(std::static_pointer_cast<control::SplinePrimitive>(primitive)->getSplineType());
  }
  else
  {
    val = static_cast<int32_t>(round(primitive->blend_radius * MULT_TIME));
  }
  buffer[index] = htobe32(val);
  index++;

  val = static_cast<int32_t>(primitive->type);
  buffer[index] = htobe32(val);
  index++;

  size_t written;

  // We stored the data in a int32_t vector, but write needs a uint8_t buffer
  return server_.write(client_fd_, (uint8_t*)buffer.data(), buffer.size() * sizeof(int32_t), written);
}

bool TrajectoryPointInterface::writeTrajectoryPoint(const vector6d_t* positions, const float acceleration,
                                                    const float velocity, const float goal_time,
                                                    const float blend_radius, const bool cartesian)
{
  std::shared_ptr<MotionPrimitive> primitive;
  if (cartesian)
  {
    primitive = std::make_shared<MoveLPrimitive>(
        urcl::Pose{ (*positions)[0], (*positions)[1], (*positions)[2], (*positions)[3], (*positions)[4],
                    (*positions)[5] },
        blend_radius, std::chrono::duration<double>(static_cast<double>(goal_time)), acceleration, velocity);
  }
  else
  {
    primitive = std::make_shared<MoveJPrimitive>(*positions, blend_radius,
                                                 std::chrono::duration<double>(static_cast<double>(goal_time)),
                                                 acceleration, velocity);
  }

  return writeMotionPrimitive(primitive);
}

bool TrajectoryPointInterface::writeTrajectoryPoint(const vector6d_t* positions, const float goal_time,
                                                    const float blend_radius, const bool cartesian)
{
  return writeTrajectoryPoint(positions, 1.4f, 1.05f, goal_time, blend_radius, cartesian);
}

bool TrajectoryPointInterface::writeTrajectorySplinePoint(const vector6d_t* positions, const vector6d_t* velocities,
                                                          const vector6d_t* accelerations, const float goal_time)
{
  if (positions == nullptr)
  {
    throw urcl::UrException("TrajectoryPointInterface::writeTrajectorySplinePoint is only getting a nullptr for "
                            "positions\n");
  }

  if (velocities == nullptr)
  {
    throw urcl::UrException("TrajectoryPointInterface::writeTrajectorySplinePoint is only getting a nullptr for "
                            "velocities\n");
  }

  std::optional<vector6d_t> target_accelerations;
  if (accelerations != nullptr)
  {
    target_accelerations = *accelerations;
  }

  return writeMotionPrimitive(std::make_shared<SplinePrimitive>(
      *positions, *velocities, target_accelerations, std::chrono::duration<double>(static_cast<double>(goal_time))));
}

void TrajectoryPointInterface::connectionCallback(const socket_t filedescriptor)
{
  if (client_fd_ == INVALID_SOCKET)
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

void TrajectoryPointInterface::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_DEBUG("Connection to trajectory interface dropped.");
  for (auto handler : disconnect_callbacks_)
  {
    handler.function(filedescriptor);
  }
  client_fd_ = INVALID_SOCKET;
}

void TrajectoryPointInterface::messageCallback([[maybe_unused]] const socket_t filedescriptor, char* buffer,
                                               int nbytesrecv)
{
  if (nbytesrecv == 4)
  {
    int32_t* status = reinterpret_cast<int32_t*>(buffer);
    URCL_LOG_DEBUG("Received message %d on TrajectoryPointInterface", be32toh(*status));

    if (!trajectory_end_callbacks_.empty())
    {
      for (auto handler : trajectory_end_callbacks_)
      {
        handler.function(static_cast<TrajectoryResult>(be32toh(*status)));
      }
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

void TrajectoryPointInterface::setTrajectoryEndCallback(std::function<void(TrajectoryResult)> callback)
{
  addTrajectoryEndCallback(callback);
}

uint32_t TrajectoryPointInterface::addTrajectoryEndCallback(const std::function<void(TrajectoryResult)>& callback)
{
  trajectory_end_callbacks_.push_back({ next_done_callback_id_, callback });
  return next_done_callback_id_++;
}

void TrajectoryPointInterface::removeTrajectoryEndCallback(const uint32_t handler_id)
{
  trajectory_end_callbacks_.remove_if(
      [handler_id](const HandlerFunction<void(TrajectoryResult)>& h) { return h.id == handler_id; });
}

}  // namespace control
}  // namespace urcl
