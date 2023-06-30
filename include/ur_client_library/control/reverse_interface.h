// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
#define UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED

#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/comm/control_mode.h"
#include "ur_client_library/types.h"
#include "ur_client_library/log.h"
#include <cstring>
#include <endian.h>
#include <condition_variable>

namespace urcl
{
namespace control
{
/*!
 * \brief Control messages for forwarding and aborting trajectories.
 */
enum class TrajectoryControlMessage : int32_t
{
  TRAJECTORY_CANCEL = -1,  ///< Represents command to cancel currently active trajectory.
  TRAJECTORY_NOOP = 0,     ///< Represents no new control command.
  TRAJECTORY_START = 1,    ///< Represents command to start a new trajectory.
};

/*!
 * \brief Control messages for starting and stopping freedrive mode.
 */
enum class FreedriveControlMessage : int32_t
{
  FREEDRIVE_STOP = -1,  ///< Represents command to stop freedrive mode.
  FREEDRIVE_NOOP = 0,   ///< Represents keep running in freedrive mode.
  FREEDRIVE_START = 1,  ///< Represents command to start freedrive mode.
};

/*!
 * \brief The ReverseInterface class handles communication to the robot. It starts a server and
 * waits for the robot to connect via its URCaps program.
 */
class ReverseInterface
{
public:
  static const int32_t MULT_JOINTSTATE = 1000000;

  ReverseInterface() = delete;
  /*!
   * \brief Creates a ReverseInterface object including a TCPServer.
   *
   * \param port Port the Server is started on
   * \param handle_program_state Function handle to a callback on program state changes.
   */
  ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state);

  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  virtual ~ReverseInterface() = default;

  /*!
   * \brief Writes needed information to the robot to be read by the URCaps program.
   *
   * \param positions A vector of joint targets for the robot
   * \param control_mode Control mode assigned to this command. See documentation of comm::ControlMode
   * for details on possible values.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  virtual bool write(const vector6d_t* positions, const comm::ControlMode control_mode = comm::ControlMode::MODE_IDLE);

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param trajectory_action 1 if a trajectory is to be started, -1 if it should be stopped
   * \param point_number The number of points of the trajectory to be executed
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeTrajectoryControlMessage(const TrajectoryControlMessage trajectory_action, const int point_number = 0);

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param freedrive_action 1 if freedrive mode is to be started, -1 if it should be stopped and 0 to keep it running
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeFreedriveControlMessage(const FreedriveControlMessage freedrive_action);

  /*!
   * \brief Set the Keepalive count. This will set the number of allowed timeout reads on the robot.
   *
   * \param count Number of allowed timeout reads on the robot.
   */
  virtual void setKeepaliveCount(const uint32_t& count)
  {
    keepalive_count_ = count;
  }

protected:
  virtual void connectionCallback(const int filedescriptor);

  virtual void disconnectionCallback(const int filedescriptor);

  virtual void messageCallback(const int filedescriptor, char* buffer, int nbytesrecv);

  int client_fd_;
  comm::TCPServer server_;

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  static const int MAX_MESSAGE_LENGTH = 8;

  std::function<void(bool)> handle_program_state_;
  uint32_t keepalive_count_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
