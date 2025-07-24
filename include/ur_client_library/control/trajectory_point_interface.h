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

#ifndef UR_CLIENT_LIBRARY_TRAJECTORY_INTERFACE_H_INCLUDED
#define UR_CLIENT_LIBRARY_TRAJECTORY_INTERFACE_H_INCLUDED

#include <list>
#include <optional>

#include "ur_client_library/control/motion_primitives.h"
#include "ur_client_library/control/reverse_interface.h"
#include "ur_client_library/comm/control_mode.h"
#include "ur_client_library/types.h"
#include "ur_client_library/log.h"

namespace urcl
{
namespace control
{
/*!
 * \brief Types for encoding trajectory execution result.
 */
enum class TrajectoryResult : int32_t
{

  TRAJECTORY_RESULT_UNKNOWN = -1,  ///< No result received, yet
  TRAJECTORY_RESULT_SUCCESS = 0,   ///< Successful execution
  TRAJECTORY_RESULT_CANCELED = 1,  ///< Canceled by user
  TRAJECTORY_RESULT_FAILURE = 2    ///< Aborted due to error during execution
};

std::string trajectoryResultToString(const TrajectoryResult result);

/*!
 * \brief The TrajectoryPointInterface class handles trajectory forwarding to the robot. Full
 * trajectories are forwarded to the robot controller and are executed there.
 */
class TrajectoryPointInterface : public ReverseInterface
{
public:
  static const int32_t MULT_TIME = 1000;
  static const int MESSAGE_LENGTH = 21;

  TrajectoryPointInterface() = delete;
  /*!
   * \brief Creates a TrajectoryPointInterface object including a TCPServer.
   *
   * \param port Port the Server is started on
   */
  TrajectoryPointInterface(uint32_t port);

  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  virtual ~TrajectoryPointInterface() = default;

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param positions A vector of joint or cartesian targets for the robot
   * \param goal_time The goal time to reach the target
   * \param blend_radius The radius to be used for blending between control points
   * \param cartesian True, if the written point is specified in cartesian space, false if in joint space
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeTrajectoryPoint(const vector6d_t* positions, const float goal_time, const float blend_radius,
                            const bool cartesian);

  /*!
   * \brief Writes information for a robot motion to the robot to be read by the URScript program.
   *
   * \param positions A vector of joint or cartesian targets for the robot
   * \param acceleration Joint acceleration of leading axis [rad/s^2] / tool acceleration [m/s^2]
   * for Cartesian motions
   * \param velocity Joint speed of leading axis [rad/s] / tool speed [m/s] for Cartesian motions
   * \param goal_time The goal time to reach the target. When a non-zero goal time is specified,
   * this has priority over speed and acceleration settings.
   * \param blend_radius The radius to be used for blending between control points
   * \param cartesian True, if the written point is specified in Cartesian space, false if in joint space
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeTrajectoryPoint(const vector6d_t* positions, const float acceleration = 1.4, const float velocity = 1.05,
                            const float goal_time = 0, const float blend_radius = 0, const bool cartesian = false);

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program including
   * velocity and acceleration information. Depending on the information given the robot will do quadratic (positions
   * only), cubic (positions and velocities) or quintic (positions, velocities and accelerations) interpolation.
   *
   * \param positions A vector of joint or Cartesian target positions for the robot.
   * \param velocities A vector of joint or Cartesian target velocities for the robot.
   * \param accelerations A vector of joint or Cartesian target accelerations for the robot.
   * \param goal_time The goal time to reach the target point.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeTrajectorySplinePoint(const vector6d_t* positions, const vector6d_t* velocities,
                                  const vector6d_t* accelerations, const float goal_time);

  /*!
   * \brief Writes a motion primitive to the robot to be read by the URScript program.
   *
   * \param primitive A shared pointer to a motion primitive object. This can contain any motion
   * primitive type that is supported. Currently, that is MoveJ, MoveL, MoveP and MoveC.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeMotionPrimitive(const std::shared_ptr<control::MotionPrimitive> primitive);

  void setTrajectoryEndCallback(std::function<void(TrajectoryResult)> callback);

  uint32_t addTrajectoryEndCallback(const std::function<void(TrajectoryResult)>& callback);

  void removeTrajectoryEndCallback(const uint32_t callback_id);

protected:
  virtual void connectionCallback(const socket_t filedescriptor) override;

  virtual void disconnectionCallback(const socket_t filedescriptor) override;

  virtual void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv) override;

private:
  std::list<HandlerFunction<void(TrajectoryResult)>> trajectory_end_callbacks_;
  uint32_t next_done_callback_id_ = 0;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_TRAJECTORY_INTERFACE_H_INCLUDED
