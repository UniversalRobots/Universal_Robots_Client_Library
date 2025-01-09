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

#ifndef UR_CLIENT_LIBRARY_INSTRUCTION_EXECUTOR_H_INCLUDED
#define UR_CLIENT_LIBRARY_INSTRUCTION_EXECUTOR_H_INCLUDED

#include "ur_client_library/ur/ur_driver.h"
#include "ur_client_library/control/motion_primitives.h"

namespace urcl
{
class InstructionExecutor
{
public:
  InstructionExecutor() = delete;
  InstructionExecutor(std::shared_ptr<urcl::UrDriver> driver) : driver_(driver)
  {
    driver_->registerTrajectoryDoneCallback(
        std::bind(&InstructionExecutor::trajDoneCallback, this, std::placeholders::_1));
    driver_->registerTrajectoryInterfaceDisconnectedCallback(
        std::bind(&InstructionExecutor::trajDisconnectCallback, this, std::placeholders::_1));
  }

  /**
   * \brief Execute a sequence of motion primitives.
   *
   * This function will execute a sequence of motion primitives. The robot will move according to the given motion
   * primitives. The function will return once the robot has reached the final target.
   *
   * \param motion_sequence The sequence of motion primitives to execute
   */
  bool executeMotion(const std::vector<std::shared_ptr<control::MotionPrimitive>>& motion_sequence);

  /**
   * \brief Move the robot to a joint target.
   *
   * This function will move the robot to the given joint target. The robot will move with the given acceleration and
   * velocity. The function will return once the robot has reached the target.
   *
   * \param target The joint target to move to.
   * \param acceleration Joint acceleration of leading axis [rad/s^2]
   * \param velocity Joint speed of leading axis [rad/s]
   * \param time The time to reach the target. If set to 0, the robot will move with the given acceleration and
   * velocity.
   * \param blend_radius The blend radius to use for the motion.
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveJ(const urcl::vector6d_t& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double time = 0, const double blend_radius = 0);

  /**
   * \brief Move the robot to a pose target.
   *
   * This function will move the robot to the given pose target. The robot will move with the given acceleration and
   * velocity. The function will return once the robot has reached the target.
   *
   * \param target The pose target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param time The time to reach the target. If set to 0, the robot will move with the given acceleration and
   * velocity.
   * \param blend_radius The blend radius to use for the motion.
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveL(const urcl::Pose& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double time = 0, const double blend_radius = 0);

private:
  void trajDoneCallback(const urcl::control::TrajectoryResult& result);
  void trajDisconnectCallback(const int filedescriptor);
  std::shared_ptr<urcl::UrDriver> driver_;
  std::atomic<bool> trajectory_running_ = false;
  std::mutex trajectory_result_mutex_;
  urcl::control::TrajectoryResult trajectory_result_;
};
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_INSTRUCTION_EXECUTOR_H_INCLUDED