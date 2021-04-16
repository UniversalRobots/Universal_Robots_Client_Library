// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
#include "ur_client_library/types.h"
#include "ur_client_library/log.h"
#include <cstring>
#include <endian.h>
#include <condition_variable>

namespace urcl
{
namespace comm
{
/*!
 * \brief Control modes as interpreted from the script runnning on the robot.
 */
enum class ControlMode : int32_t
{
  MODE_STOPPED = -2,        ///< When this is set, the program is expected to stop and exit.
  MODE_UNINITIALIZED = -1,  ///< Startup default until another mode is sent to the script.
  MODE_IDLE = 0,            ///< Set when no controller is currently active controlling the robot.
  MODE_SERVOJ = 1,          ///< Set when servoj control is active.
  MODE_SPEEDJ = 2           ///< Set when speedj control is active.
};

/*!
 * \brief The ReverseInterface class handles communication to the robot. It starts a server and
 * waits for the robot to connect via its URCaps program.
 */
class ReverseInterface
{
public:
  ReverseInterface() = delete;
  /*!
   * \brief Creates a ReverseInterface object including a TCPServer.
   *
   * \param port Port the Server is started on
   * \param handle_program_state Function handle to a callback on program state changes.
   */
  ReverseInterface(uint32_t port, std::function<void(bool)> handle_program_state)
    : client_fd_(-1), server_(port), handle_program_state_(handle_program_state), keepalive_count_(1)
  {
    handle_program_state_(false);
    server_.setMessageCallback(
        std::bind(&ReverseInterface::messageCallback, this, std::placeholders::_1, std::placeholders::_2));
    server_.setConnectCallback(std::bind(&ReverseInterface::connectionCallback, this, std::placeholders::_1));
    server_.setDisconnectCallback(std::bind(&ReverseInterface::disconnectionCallback, this, std::placeholders::_1));
    server_.setMaxClientsAllowed(1);
    server_.start();
  }
  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  ~ReverseInterface()
  {
  }

  /*!
   * \brief Writes needed information to the robot to be read by the URCaps program.
   *
   * \param positions A vector of joint targets for the robot
   * \param control_mode Control mode assigned to this command. See documentation of ::ControlMode
   * for details on possible values.
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool write(const vector6d_t* positions, const ControlMode control_mode = ControlMode::MODE_IDLE)
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

  /*!
   * \brief Set the Keepalive count. This will set the number of allowed timeout reads on the robot.
   *
   * \param count Number of allowed timeout reads on the robot.
   */
  void setKeepaliveCount(const uint32_t& count)
  {
    keepalive_count_ = count;
  }

private:
  void connectionCallback(const int filedescriptor)
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

  void disconnectionCallback(const int filedescriptor)
  {
    URCL_LOG_INFO("Connection to reverse interface dropped.", filedescriptor);
    client_fd_ = -1;
    handle_program_state_(false);
  }

  void messageCallback(const int filedescriptor, char* buffer)
  {
    URCL_LOG_WARN("Messge on ReverseInterface received. The reverse interface currently does not support any message "
                  "handling. This message will be ignored.");
  }

  int client_fd_;
  TCPServer server_;

  static const int32_t MULT_JOINTSTATE = 1000000;

  template <typename T>
  size_t append(uint8_t* buffer, T& val)
  {
    size_t s = sizeof(T);
    std::memcpy(buffer, &val, s);
    return s;
  }

  std::function<void(bool)> handle_program_state_;
  uint32_t keepalive_count_;
};

}  // namespace comm
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_REVERSE_INTERFACE_H_INCLUDED
