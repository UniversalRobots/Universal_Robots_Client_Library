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
#include "ur_client_library/ur/robot_receive_timeout.h"
#include "ur_client_library/ur/version_information.h"
#include <cstring>
#include <endian.h>
#include <condition_variable>
#include <list>

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

struct ReverseInterfaceConfig
{
  uint32_t port = 50001;  //!< Port the server is started on
  std::function<void(bool)> handle_program_state = [](bool) {
    return;
  };  //!< Function handle to a callback on program state changes.
  std::chrono::milliseconds step_time = std::chrono::milliseconds(8);  //!< The robots step time
  uint32_t keepalive_count = 0;                                      //!< Number of allowed timeout reads on the robot.
  VersionInformation robot_software_version = VersionInformation();  //!< The robot software version.
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
   * \param step_time The robots step time
   */
  [[deprecated("Use ReverseInterfaceConfig instead of port, handle_program_state and step_time parameters")]]
  ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state,
                   std::chrono::milliseconds step_time = std::chrono::milliseconds(8));

  ReverseInterface(const ReverseInterfaceConfig& config);

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
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. Use with caution when dealing with realtime commands as the robot
   * expects to get a new control signal each control cycle. Note the timeout cannot be higher than 1 second for
   * realtime commands.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  virtual bool write(const vector6d_t* positions, const comm::ControlMode control_mode = comm::ControlMode::MODE_IDLE,
                     const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(20));

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param trajectory_action 1 if a trajectory is to be started, -1 if it should be stopped
   * \param point_number The number of points of the trajectory to be executed
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. If you want to make the read function blocking then use RobotReceiveTimeout::off()
   * function to create the RobotReceiveTimeout object
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool
  writeTrajectoryControlMessage(const TrajectoryControlMessage trajectory_action, const int point_number = 0,
                                const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(200));

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param freedrive_action 1 if freedrive mode is to be started, -1 if it should be stopped and 0 to keep it running
   * \param robot_receive_timeout The read timeout configuration for the reverse socket running in the external
   * control script on the robot. If you want to make the read function blocking then use RobotReceiveTimeout::off()
   * function to create the RobotReceiveTimeout object
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool
  writeFreedriveControlMessage(const FreedriveControlMessage freedrive_action,
                               const RobotReceiveTimeout& robot_receive_timeout = RobotReceiveTimeout::millisec(200));

  /*!
   * \brief Set the Keepalive count. This will set the number of allowed timeout reads on the robot.
   *
   * \param count Number of allowed timeout reads on the robot.
   */
  [[deprecated("Set keepaliveCount is deprecated, instead use the robot receive timeout directly in the write "
               "commands.")]] virtual void
  setKeepaliveCount(const uint32_t count);

  /*!
   * \brief Register a callback for the robot-based disconnection.
   *
   * The callback will be called when the robot disconnects from the reverse interface.
   *
   * \param disconnection_fun The function to be called on disconnection.
   *
   * \returns A unique handler ID for the registered callback. This can be used to unregister the
   * callback later.
   */
  uint32_t registerDisconnectionCallback(std::function<void(const int)> disconnection_fun)
  {
    disconnect_callbacks_.push_back({ next_disconnect_callback_id_, disconnection_fun });
    return next_disconnect_callback_id_++;
  }

  /*! \brief Unregisters a disconnection callback.
   *
   * \param handler_id The ID of the handler to be unregistered as obtained from
   * registerDisconnectionCallback.
   */
  void unregisterDisconnectionCallback(const uint32_t handler_id)
  {
    disconnect_callbacks_.remove_if(
        [handler_id](const HandlerFunction<void(const int)>& h) { return h.id == handler_id; });
  }

  /*!
   * \brief Checks if the reverse interface is connected to the robot.
   *
   * \returns True, if the interface is connected, false otherwise.
   */
  bool isConnected() const
  {
    return client_fd_ != INVALID_SOCKET;
  }

protected:
  virtual void connectionCallback(const socket_t filedescriptor);

  virtual void disconnectionCallback(const socket_t filedescriptor);

  virtual void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv);

  std::list<HandlerFunction<void(const int)>> disconnect_callbacks_;
  uint32_t next_disconnect_callback_id_ = 0;
  socket_t client_fd_;
  comm::TCPServer server_;

  VersionInformation robot_software_version_;

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  static const int MAX_MESSAGE_LENGTH = 8;

  std::function<void(bool)> handle_program_state_;
  std::chrono::milliseconds step_time_;

  uint32_t keepalive_count_;
  bool keep_alive_count_modified_deprecated_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
