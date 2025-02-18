/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <memory>

#include "ur_client_library/comm/socket_t.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief State the socket can be in
 */
enum class SocketState
{
  Invalid,       ///< Socket is initialized or setup failed
  Connected,     ///< Socket is connected and ready to use
  Disconnected,  ///< Socket is disconnected and cannot be used
  Closed         ///< Connection to socket got closed
};

/*!
 * \brief Class for TCP socket abstraction
 */
class TCPSocket
{
private:
  std::atomic<socket_t> socket_fd_;
  std::atomic<SocketState> state_;
  std::chrono::milliseconds reconnection_time_;
  bool reconnection_time_modified_deprecated_ = false;

  void setupOptions();

protected:
  static bool open(socket_t socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, static_cast<socklen_t>(address_len)) == 0;
  }

  bool setup(const std::string& host, const int port, const size_t max_num_tries = 0,
             const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME);

  std::unique_ptr<timeval> recv_timeout_;

public:
  static constexpr std::chrono::milliseconds DEFAULT_RECONNECTION_TIME{ 10000 };
  /*!
   * \brief Creates a TCPSocket object
   */
  TCPSocket();
  virtual ~TCPSocket();

  /*!
   * \brief Getter for the state of the socket.
   *
   * \returns Returns the current state of the socket
   */
  SocketState getState()
  {
    return state_;
  }

  /*!
   * \brief Getter for the file descriptor of the socket.
   *
   * \returns The file descriptor of the socket
   */
  socket_t getSocketFD()
  {
    return socket_fd_;
  }

  /*!
   * \brief Determines the local IP address of the currently configured socket
   *
   * \returns The local IP address of the socket associated to the current file descriptor
   */
  std::string getIP() const;

  /*!
   * \brief Reads one byte from the socket
   *
   * \param[out] character Target buffer
   *
   * \returns True on success, false otherwise
   */
  bool read(char* character);

  /*!
   * \brief Reads data from the socket
   *
   * \param[out] buf Buffer where the data shall be stored
   * \param[in] buf_len Number of bytes allocated for the buffer
   * \param[out] read Number of bytes actually read
   *
   * \returns True on success, false otherwise
   */
  bool read(uint8_t* buf, const size_t buf_len, size_t& read);

  /*!
   * \brief Writes to the socket
   *
   * \param[in] buf Buffer of bytes to write
   * \param[in] buf_len Number of bytes in the buffer
   * \param[out] written Number of bytes actually written
   *
   * \returns True on success, false otherwise
   */
  bool write(const uint8_t* buf, const size_t buf_len, size_t& written);

  /*!
   * \brief Closes the connection to the socket.
   */
  void close();

  /*!
   * \brief Setup Receive timeout used for this socket.
   *
   * \param timeout Timeout used for setting things up
   */
  void setReceiveTimeout(const timeval& timeout);

  /*!
   * \brief Set reconnection time, if the server is unavailable during connection this will set the time before
   * trying connect to the server again.
   *
   * \param reconnection_time time in between connection attempts to the server
   */
  [[deprecated("Reconnection time is passed to setup directly now.")]] void
  setReconnectionTime(const std::chrono::milliseconds reconnection_time);
};
}  // namespace comm
}  // namespace urcl
