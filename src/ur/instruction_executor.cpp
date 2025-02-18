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
#include "ur_client_library/control/trajectory_point_interface.h"
void urcl::InstructionExecutor::trajDoneCallback(const urcl::control::TrajectoryResult& result)
{
  std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
  trajectory_result_ = result;
  trajectory_running_ = false;
}
void urcl::InstructionExecutor::trajDisconnectCallback(const int filedescriptor)
{
  URCL_LOG_INFO("Trajectory disconnect");
  std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
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
    switch (primitive->type)
    {
      case control::MotionType::MOVEJ:
      {
        auto movej_primitive = std::static_pointer_cast<control::MoveJPrimitive>(primitive);
        driver_->writeTrajectoryPoint(movej_primitive->target_joint_configuration, primitive->acceleration,
                                      primitive->velocity, false, primitive->duration.count(), primitive->blend_radius);
        break;
      }
      case control::MotionType::MOVEL:
      {
        auto movel_primitive = std::static_pointer_cast<control::MoveLPrimitive>(primitive);
        urcl::vector6d_t pose_vec = { movel_primitive->target_pose.x,  movel_primitive->target_pose.y,
                                      movel_primitive->target_pose.z,  movel_primitive->target_pose.rx,
                                      movel_primitive->target_pose.ry, movel_primitive->target_pose.rz };
        driver_->writeTrajectoryPoint(pose_vec, primitive->acceleration, primitive->velocity, true,
                                      primitive->duration.count(), primitive->blend_radius);
        break;
      }
      default:
        URCL_LOG_ERROR("Unsupported motion type");
        // The hardware will complain about missing trajectory points and return a failure for
        // trajectory execution. Hence, we need to step into the running loop below.
    }
  }
  trajectory_running_ = true;

  while (trajectory_running_)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
  }
  {
    std::unique_lock<std::mutex> lock(trajectory_result_mutex_);
    URCL_LOG_INFO("Trajectory done with result %s", control::trajectoryResultToString(trajectory_result_).c_str());
    return trajectory_result_ == urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS;
  }
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
