// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

#include "ur_client_library/ur/instruction_executor.h"
#include <mutex>
#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/log.h"
#include "ur_client_library/ur/robot_receive_timeout.h"
void urcl::InstructionExecutor::trajDoneCallback(const urcl::control::TrajectoryResult& result)
{
  URCL_LOG_DEBUG("Trajectory result received: %s", control::trajectoryResultToString(result).c_str());
  if (trajectory_running_)
  {
    std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
    trajectory_done_cv_.notify_all();
    trajectory_result_ = result;
    trajectory_running_ = false;
  }
}
void urcl::InstructionExecutor::trajDisconnectCallback(const int filedescriptor)
{
  URCL_LOG_INFO("Trajectory disconnect");
  std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
  trajectory_done_cv_.notify_all();
  if (trajectory_running_)
  {
    trajectory_result_ = urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE;
    trajectory_running_ = false;
  }
}
bool urcl::InstructionExecutor::executeMotion(
    const std::vector<std::shared_ptr<control::MotionPrimitive>>& motion_sequence)
{
  if (!driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                              motion_sequence.size()))
  {
    URCL_LOG_ERROR("Cannot send trajectory control command. No client connected?");
    std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
    trajectory_result_ = urcl::control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE;
    return false;
  }

  for (const auto& primitive : motion_sequence)
  {
    try
    {
      driver_->writeMotionPrimitive(primitive);
    }
    catch (const UnsupportedMotionType&)
    {
      URCL_LOG_ERROR("Unsupported motion type");
      // The hardware will complain about missing trajectory points and return a failure for
      // trajectory execution. Hence, we need to step into the running loop below.
    }
  }
  trajectory_running_ = true;
  cancel_requested_ = false;

  while (trajectory_running_ && !cancel_requested_)
  {
    driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!cancel_requested_)
  {
    std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
    URCL_LOG_INFO("Trajectory done with result %s", control::trajectoryResultToString(trajectory_result_).c_str());
    return trajectory_result_ == urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS;
  }
  return false;
}
bool urcl::InstructionExecutor::moveJ(const urcl::vector6d_t& target, const double acceleration, const double velocity,
                                      const double time, const double blend_radius)
{
  return executeMotion({ std::make_shared<control::MoveJPrimitive>(
      target, blend_radius, std::chrono::milliseconds(static_cast<int>(time * 1000)), acceleration, velocity) });
}

bool urcl::InstructionExecutor::moveL(const urcl::Pose& target, const double acceleration, const double velocity,
                                      const double time, const double blend_radius)
{
  return executeMotion({ std::make_shared<control::MoveLPrimitive>(
      target, blend_radius, std::chrono::milliseconds(static_cast<int>(time * 1000)), acceleration, velocity) });
}

bool urcl::InstructionExecutor::moveP(const urcl::Pose& target, const double acceleration, const double velocity,
                                      const double blend_radius)
{
  return executeMotion({ std::make_shared<control::MovePPrimitive>(target, blend_radius, acceleration, velocity) });
}

bool urcl::InstructionExecutor::moveC(const urcl::Pose& via, const urcl::Pose& target, const double acceleration,
                                      const double velocity, const double blend_radius, const int32_t mode)
{
  return executeMotion(
      { std::make_shared<control::MoveCPrimitive>(via, target, blend_radius, acceleration, velocity, mode) });
}

bool urcl::InstructionExecutor::optimoveJ(const urcl::vector6d_t& target, const double acceleration,
                                          const double velocity, const double blend_radius)
{
  return executeMotion({ std::make_shared<control::OptimoveJPrimitive>(target, blend_radius, acceleration, velocity) });
}

bool urcl::InstructionExecutor::optimoveL(const urcl::Pose& target, const double acceleration, const double velocity,
                                          const double blend_radius)
{
  return executeMotion({ std::make_shared<control::OptimoveLPrimitive>(target, blend_radius, acceleration, velocity) });
}

bool urcl::InstructionExecutor::cancelMotion()
{
  cancel_requested_ = true;
  if (trajectory_running_)
  {
    URCL_LOG_INFO("Cancel motion");
    driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_CANCEL, -1,
                                           RobotReceiveTimeout::millisec(2000));
    std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
    if (trajectory_done_cv_.wait_for(lock, std::chrono::milliseconds(200)) == std::cv_status::no_timeout)
    {
      return trajectory_result_ == urcl::control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED;
    }
    else
    {
      URCL_LOG_ERROR("Sent a canceling request to the robot but waiting for the answer timed out.");
      return false;
    }
  }
  URCL_LOG_WARN("Canceling motion requested without a motion running.");
  return false;
}
