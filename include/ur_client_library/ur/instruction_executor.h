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
    traj_done_callback_handler_id_ = driver_->registerTrajectoryDoneCallback(
        std::bind(&InstructionExecutor::trajDoneCallback, this, std::placeholders::_1));
    disconnected_handler_id_ = driver_->registerTrajectoryInterfaceDisconnectedCallback(
        std::bind(&InstructionExecutor::trajDisconnectCallback, this, std::placeholders::_1));
  }

  ~InstructionExecutor()
  {
    driver_->unregisterTrajectoryDoneCallback(traj_done_callback_handler_id_);
    driver_->unregisterTrajectoryInterfaceDisconnectedCallback(disconnected_handler_id_);
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
   * \brief Move the robot to a joint or Cartesian target using movej.
   *
   * This overload accepts a \ref urcl::MotionTarget which can hold either a \ref urcl::Q (joint
   * target, forwarded as ``movej(q, ...)``) or a \ref urcl::Pose (Cartesian target, forwarded as
   * ``movej(p[...], ...)``). In the latter case the robot will internally solve inverse
   * kinematics. The robot will move with the given acceleration and velocity. The function will
   * return once the robot has reached the target.
   *
   * \note A braced-initializer-list of six doubles (``moveJ({...}, ...)``) will bind to the
   * \ref vector6d_t overload above, not to this overload. To call this overload with a pose, pass
   * an explicit \ref urcl::Pose; with a joint target, pass an explicit \ref urcl::Q or use the
   * vector6d_t overload.
   *
   * \param target The joint or Cartesian target to move to.
   * \param acceleration Joint acceleration of leading axis [rad/s^2]
   * \param velocity Joint speed of leading axis [rad/s]
   * \param time The time to reach the target. If set to 0, the robot will move with the given acceleration and
   * velocity.
   * \param blend_radius The blend radius to use for the motion.
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveJ(const MotionTarget& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double time = 0, const double blend_radius = 0);

  /**
   * \brief Move the robot to a pose target using movel
   *
   * This function will move the robot to the given pose target. The robot will move with the given acceleration and
   * velocity or a motion time. The function will return once the robot has reached the target.
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

  /**
   * \brief Move the robot to a Cartesian or joint target using movel.
   *
   * This overload accepts a \ref urcl::MotionTarget. A \ref urcl::Pose is forwarded as
   * ``movel(p[...], ...)``; a \ref urcl::Q is forwarded as ``movel(q, ...)`` (the robot
   * internally runs forward kinematics to perform a linear motion in tool space towards the joint
   * configuration's resulting pose).
   *
   * \note A braced-initializer-list of six doubles (``moveL({...}, ...)``) will bind to the
   * \ref urcl::Pose overload above. To target a joint configuration, pass an explicit
   * \ref urcl::Q.
   *
   * \param target The target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param time The time to reach the target. If set to 0, the robot will move with the given acceleration and
   * velocity.
   * \param blend_radius The blend radius to use for the motion.
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveL(const MotionTarget& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double time = 0, const double blend_radius = 0);

  /**
   * \brief Move the robot to a pose target using movep
   *
   * This function will move the robot to the given pose target. The robot will move with the given acceleration and
   * velocity. The function will return once the robot has reached the target.
   *
   * \param target The pose target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveP(const urcl::Pose& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double blend_radius = 0.0);

  /**
   * \brief Move the robot to a Cartesian or joint target using movep.
   *
   * This overload accepts a \ref urcl::MotionTarget. A \ref urcl::Pose is forwarded as
   * ``movep(p[...], ...)``; a \ref urcl::Q is forwarded as ``movep(q, ...)``.
   *
   * \note A braced-initializer-list of six doubles (``moveP({...}, ...)``) will bind to the
   * \ref urcl::Pose overload above. To target a joint configuration, pass an explicit
   * \ref urcl::Q.
   *
   * \param target The target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveP(const MotionTarget& target, const double acceleration = 1.4, const double velocity = 1.04,
             const double blend_radius = 0.0);

  /**
   * \brief Move the robot to a pose target using movec
   *
   * This function will move the robot to the given pose target in a circular motion going through via. The robot will
   * move with the given acceleration and velocity. The function will return once the robot has reached the target.
   *
   * \param via The circle will be defined by the current pose (the end pose of the previous motion), the target and the
   * via point.
   * \param target The pose target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveC(const urcl::Pose& via, const urcl::Pose& target, const double acceleration = 1.4,
             const double velocity = 1.04, const double blend_radius = 0.0, const int32_t mode = 0);

  /**
   * \brief Move the robot in a circular motion using movec.
   *
   * This overload accepts \ref urcl::MotionTarget values for both the via point and the target.
   * Each of them can independently be a \ref urcl::Pose (passed as ``p[...]``) or a
   * \ref urcl::Q (passed as a joint configuration, translated into a pose by the controller's
   * forward kinematics before executing the circular motion).
   *
   * \note Braced-initializer-lists of six doubles will bind to the \ref urcl::Pose overload above.
   * Wrap with an explicit \ref urcl::Q to target joint configurations.
   *
   * \param via The via point defining the circle.
   * \param target The target to move to.
   * \param acceleration Tool acceleration [m/s^2]
   * \param velocity Tool speed [m/s]
   * \param blend_radius The blend radius to use for the motion.
   * \param mode The movec mode as defined in the URScript manual.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool moveC(const MotionTarget& via, const MotionTarget& target, const double acceleration = 1.4,
             const double velocity = 1.04, const double blend_radius = 0.0, const int32_t mode = 0);

  /**
   * \brief Move the robot to a joint target using optimoveJ.
   *
   * This function will move the robot to the given joint target using the optimoveJ motion
   * primitive. The robot will move with the given acceleration and velocity fractions.
   *
   * \param target The joint target to move to.
   * \param acceleration_fraction The fraction of the maximum acceleration to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param velocity_fraction The fraction of the maximum velocity to use for the motion_sequence
   * (0.0 < fraction <= 1.0).
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool optimoveJ(const urcl::vector6d_t& target, const double acceleration_fraction = 0.3,
                 const double velocity_fraction = 0.3, const double blend_radius = 0);

  /**
   * \brief Move the robot to a joint or Cartesian target using optimoveJ.
   *
   * This overload accepts a \ref urcl::MotionTarget. A \ref urcl::Q is forwarded as a joint
   * target; a \ref urcl::Pose is forwarded as a Cartesian target (``optimovej(p[...], ...)``).
   *
   * \note A braced-initializer-list of six doubles binds to the \ref vector6d_t overload above.
   *
   * \param target The target to move to.
   * \param acceleration_fraction The fraction of the maximum acceleration to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param velocity_fraction The fraction of the maximum velocity to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool optimoveJ(const MotionTarget& target, const double acceleration_fraction = 0.3,
                 const double velocity_fraction = 0.3, const double blend_radius = 0);

  /**
   * \brief Move the robot to a pose target using optimoveL.
   *
   * This function will move the robot to the given pose target using the optimoveL motion
   * primitive. The robot will move with the given acceleration and velocity fractions.
   *
   * \param target The pose target to move to.
   * \param acceleration_fraction The fraction of the maximum acceleration to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param velocity_fraction The fraction of the maximum velocity to use for the motion_sequence
   * (0.0 < fraction <= 1.0).
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool optimoveL(const urcl::Pose& target, const double acceleration_fraction = 0.3,
                 const double velocity_fraction = 0.3, const double blend_radius = 0);

  /**
   * \brief Move the robot to a Cartesian or joint target using optimoveL.
   *
   * This overload accepts a \ref urcl::MotionTarget. A \ref urcl::Pose is forwarded as a
   * Cartesian target; a \ref urcl::Q is forwarded as a joint target (``optimovel(q, ...)``).
   *
   * \note A braced-initializer-list of six doubles binds to the \ref urcl::Pose overload above.
   *
   * \param target The target to move to.
   * \param acceleration_fraction The fraction of the maximum acceleration to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param velocity_fraction The fraction of the maximum velocity to use for the motion
   * (0.0 < fraction <= 1.0).
   * \param blend_radius The blend radius to use for the motion.
   *
   * \return True if the robot has reached the target, false otherwise.
   */
  bool optimoveL(const MotionTarget& target, const double acceleration_fraction = 0.3,
                 const double velocity_fraction = 0.3, const double blend_radius = 0);

  /**
   * \brief Cancel the current motion.
   *
   * If no motion is running, false will be returned.
   * If a motion is running, it will be canceled and it will wait for a TRAJECTORY_CANCELED result
   * with a timeout of one second.
   *
   * If another result or no result is received, false will be returned.
   */
  bool cancelMotion();

  /**
   * \brief Check if a trajectory is currently running.
   */
  bool isTrajectoryRunning() const
  {
    return trajectory_running_;
  }

protected:
  void trajDoneCallback(const urcl::control::TrajectoryResult& result);
  void trajDisconnectCallback(const socket_t filedescriptor);

  uint32_t traj_done_callback_handler_id_;
  uint32_t disconnected_handler_id_;

  std::shared_ptr<urcl::UrDriver> driver_;
  std::atomic<bool> trajectory_running_ = false;
  std::atomic<bool> cancel_requested_ = false;
  std::mutex trajectory_result_mutex_;
  std::condition_variable trajectory_done_cv_;
  urcl::control::TrajectoryResult trajectory_result_;
};
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_INSTRUCTION_EXECUTOR_H_INCLUDED
