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
  Connecting,      ///< A first-time connect() attempt is in progress
  Connected,       ///< Socket is connected and ready to use
  LostConnection,  ///< Connection dropped unexpectedly; auto-reconnect is expected to pick it up
  Reconnecting,    ///< An automatic reconnect attempt (after a drop) is in progress
  Disconnecting,   ///< A deliberate disconnect() is in progress
  Disconnected,    ///< Deliberately disconnected; will NOT auto-reconnect until connect() is called
  Closed           ///< Neutral low-level close (clearable by a subsequent (re)connect)
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

  // Atomically moves state_ to `desired`, unless a deliberate disconnect() (Disconnecting or
  // Disconnected) is in effect. Returns true if the state was set, false if a deliberate stop is
  // active (in which case state_ is left untouched). This is how the connect/retry machinery
  // updates its in-progress state without ever clobbering a teardown signal.
  bool setStateUnlessStopRequested(SocketState desired);

  // Performs an interruptible, non-blocking connect on an already-created socket.
  // Polls in short slices so that a concurrent disconnect() aborts the attempt
  // promptly on all platforms (POSIX close() of a blocked connect() is reliable,
  // Winsock's is not). Restores blocking mode on success.
  bool openInterruptible(socket_t socket_fd, struct sockaddr* address, size_t address_len);

  // Internal connect/retry machinery. Establishes a connection to host/port, retrying up to
  // max_num_tries times (unlimited when 0). Used by the protected connect()/reconnect() entry
  // points; not called directly by subclasses.
  bool setup(const std::string& host, const int port, const size_t max_num_tries = 0,
             const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME);

protected:
  // True while a deliberate disconnect() is in progress or has completed (the "deliberate-stop
  // set"). The connect/retry machinery checks this to abort, and never overwrites these states,
  // so a teardown disconnect() that races a reconnect attempt is observed reliably.
  bool isStopRequested() const
  {
    const SocketState s = state_.load();
    return s == SocketState::Disconnecting || s == SocketState::Disconnected;
  }

  /*!
   * \brief Explicitly clears a deliberate disconnect() so a subsequent connect() can revive the
   * socket.
   *
   * This is the ONLY way to leave the deliberate-stop set, and it must be called on the controlling
   * thread when no automatic reconnect path is concurrently active (e.g. before a deliberate
   * reconnect following a stop()). connect() never clears the stop on its own, so an automatic
   * reconnect cannot accidentally undo a teardown. No-op unless a deliberate disconnect() is in
   * effect.
   */
  void allowReconnect()
  {
    // Move out of the deliberate-stop set to a neutral, connectable state. Only act on the
    // deliberate-stop states so this is a no-op for a socket that is connecting/connected.
    SocketState expected = SocketState::Disconnected;
    if (!state_.compare_exchange_strong(expected, SocketState::Closed))
    {
      expected = SocketState::Disconnecting;
      state_.compare_exchange_strong(expected, SocketState::Closed);
    }
  }

  static bool open(socket_t socket_fd, struct sockaddr* address, size_t address_len)
  {
    return ::connect(socket_fd, address, static_cast<socklen_t>(address_len)) == 0;
  }

  /*!
   * \brief Establishes a connection to the configured host/port.
   *
   * This is the explicit (re)connect entry point for subclasses. It clears any prior deliberate
   * disconnect() (moving the socket to Connecting) and then attempts to connect, retrying up to
   * max_num_tries times (unlimited when 0). Call this on the controlling thread; the automatic
   * reconnect path uses reconnect() instead.
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
   * \brief Re-establishes a connection after an unexpected drop, without clearing a deliberate
   * disconnect().
   *
   * Used by the automatic reconnect path (e.g. the producer loop). If a deliberate disconnect()
   * is in progress or has completed, this returns false immediately instead of reconnecting, so a
   * concurrent teardown is never undone. Otherwise behaves like setup() but marks the socket as
   * Reconnecting while the attempt is in progress.
   */
  bool reconnect(const std::string& host, const int port, const size_t max_num_tries = 0,
                 const std::chrono::milliseconds reconnection_time = DEFAULT_RECONNECTION_TIME);

  /*!
   * \brief Deliberately disconnects the socket and leaves it ready to connect() again.
   *
   * Moves the socket into the deliberate-stop set (Disconnecting then Disconnected) and closes the
   * underlying file descriptor. Any connect/reconnect attempt currently in progress (blocked in a
   * connect or sleeping between attempts) aborts promptly, and the automatic reconnect path will
   * not reconnect until connect() is called again. Use this at teardown (e.g. from a destructor)
   * before joining a reconnect thread.
   */
  void disconnect();

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
   *
   * Neutral low-level close. Unlike disconnect(), it does not prevent a subsequent automatic
   * reconnect, and it never downgrades a deliberate disconnect() that is already in effect.
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
