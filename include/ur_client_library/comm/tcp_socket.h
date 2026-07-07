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
  Invalid,         ///< Socket is initialized but was never connected
  Connecting,      ///< A connection is in progress
  Connected,       ///< Socket is connected and ready to use
  LostConnection,  ///< Connection dropped unexpectedly; auto-reconnect is expected to pick it up
  Disconnecting,   ///< A deliberate disconnect() is in progress
  Closed,          ///< Connection to socket got closed
};

const std::string& socketStateToString(SocketState state);

/*!
 * \brief Class for TCP socket abstraction
 */
class TCPSocket
{
private:
  std::atomic<socket_t> socket_fd_;
  std::atomic<SocketState> state_;
  std::atomic<SocketState> target_state_;
  std::chrono::milliseconds reconnection_time_;
  bool reconnection_time_modified_deprecated_ = false;

  void setupOptions();

  // Performs an interruptible, non-blocking connect on an already-created socket.
  // Polls in short slices so that a concurrent disconnect() aborts the attempt
  // promptly on all platforms (POSIX close() of a blocked connect() is reliable,
  // Winsock's is not). Restores blocking mode on success.
  bool openInterruptible(socket_t socket_fd, struct sockaddr* address, size_t address_len);

  bool setupInternal(const std::string& host, const int port, const size_t max_num_tries,
                     const std::chrono::milliseconds reconnection_time);

protected:
  /*!
   * \brief Atomically moves state_ to `desired`, unless a deliberate disconnect() is in effect.
   *
   * Returns true if the state was set, false if a deliberate stop is
   * active (in which case state_ is left untouched). This is how the connect/retry machinery
   * updates its in-progress state without ever clobbering a teardown signal.
   */
  bool setTargetStateUnlessStopRequested(SocketState desired);

  /*!
   * \brief Query whether there has been a deliberate disconnect() request.
   *
   * True while a deliberate disconnect() is in progress or has completed (the "deliberate-stop
   * set").
   */
  bool isStopRequested() const
  {
    return target_state_ == SocketState::Closed;
  }

  /*!
   * \brief Performs a blocking connect on an already-created socket.
   *
   * This is the platform-native, uninterruptible connect. It is used when the caller has
   * already established that no deliberate disconnect() is in effect, and is willing to block
   * until the connect succeeds or fails. The caller must ensure that the socket is in blocking
   * mode before calling this.
   */
  [[deprecated("Use connect() instead, which is interruptible by a concurrent disconnect()")]]
  static bool open(socket_t socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, static_cast<socklen_t>(address_len)) == 0;
  }

  [[deprecated("Use the public method connect() instead")]]
  bool setup(const std::string& host, const int port, const size_t max_num_tries = 0,
             const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME)
  {
    return connect(host, port, max_num_tries, reconnection_time);
  }

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
   * \brief Establishes a connection to the configured host/port.
   *
   * This is the explicit connection setup method. It clears any prior deliberate
   * disconnect() (moving the socket to Connecting) and then attempts to connect, retrying up to
   * max_num_tries times (unlimited when 0).
   *
   * \param host Host to connect to
   * \param port Port to connect to
   * \param max_num_tries Maximum number of connection attempts before failing. Unlimited when 0.
   * \param reconnection_time Time between connection attempts
   *
   * \returns True on success, false if the connection could not be established or was aborted by
   * a concurrent disconnect()
   */
  bool connect(const std::string& host, const int port, const size_t max_num_tries = 0,
               const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME);

  /*!
   * \brief Reconnects to the configured host/port.
   *
   * This is the explicit reconnection method. It can only be called when the socket is in
   * LostConnection state, and will attempt to reconnect, retrying up to max_num_tries times
   * (unlimited when 0). Thus, when an explicit disconnect() is in effect, this will fail
   * immediately. When a concurrent disconnect() is called while this is in progress, it will abort
   * and return false.
   *
   * \param host Host to connect to
   * \param port Port to connect to
   * \param max_num_tries Maximum number of connection attempts before failing. Unlimited when 0.
   * \param reconnection_time Time between connection attempts
   *
   * \returns True on success, false if the connection could not be established or was aborted by
   * a concurrent disconnect()
   */
  bool reconnect(const std::string& host, const int port, const size_t max_num_tries = 0,
                 const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME);

  /*!
   * \brief Disconnects the client
   *
   * This overwrites connection attempts. When a deliberate disconnect() has been called, the
   * socket will not execute any connection attempts until it has reached the CLOSED state and
   * connect() is called again.
   */
  void close();

  /*!
   * \brief Alias function for close() to disconnect the client
   */
  void disconnect()
  {
    close();
  }

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
