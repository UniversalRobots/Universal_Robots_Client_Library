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
 * \date    2021-03-13
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_TCP_SERVER_H_INCLUDED
#define UR_CLIENT_LIBRARY_TCP_SERVER_H_INCLUDED

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>

#include "ur_client_library/comm/socket_t.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief Wrapper class for a TCP socket server.
 *
 * The server can be created given a port and it can register callbacks for 3 events:
 *  - connect: A new client connected
 *  - disconnect: A client disconnected
 *  - message: Data sent from one of the clients
 *
 *  Please note that start() has to be called manually after initialization and callback
 *  registering in order to start handling socket events.
 *
 *  While this server implementation supports multiple (number limited by system's socket
 *  implementation) clients by default, a maximum number of allowed clients can be configured.
 */
class TCPServer
{
public:
  TCPServer() = delete;
  /*!
   * \brief Create a TCPServer object
   *
   * \param port Port on which to operate. The port will be bound to the process creating the
   * object.
   * \param max_num_tries If binding the socket fails, it will be retried this many times. If 0 is
   * specified, binding the socket will be tried indefinitely.
   * \param reconnection_time Wait time in between binding attempts.
   */
  explicit TCPServer(const int port, const size_t max_num_tries = 0,
                     const std::chrono::milliseconds reconnection_time = std::chrono::seconds(1));
  virtual ~TCPServer();

  /*!
   * \brief This callback will be triggered on clients connecting to the server
   *
   * \param func Function handling the event information. The file descriptor created by the
   * connection event will be passed to the function.
   */
  void setConnectCallback(std::function<void(const socket_t)> func)
  {
    new_connection_callback_ = func;
  }

  /*!
   * \brief This callback will be triggered on clients disconnecting from the server
   *
   * \param func Function handling the event information. The file descriptor created by the
   * connection event will be passed to the function.
   */
  void setDisconnectCallback(std::function<void(const socket_t)> func)
  {
    disconnect_callback_ = func;
  }

  /*!
   * \brief This callback will be triggered on messages received on the socket
   *
   * \param func Function handling the event information. The file client's file_descriptor will be
   * passed to the function as well as the actual message received from the client.
   */
  void setMessageCallback(std::function<void(const socket_t, char*, int)> func)
  {
    message_callback_ = func;
  }

  /*!
   * \brief Start event handling.
   *
   * Without calling this function the socket will be advertised and bound to a tcp port, but no
   * handling of connection requests will be performed.
   */
  void start();

  /*!
   * \brief Shut down the event listener thread. After calling this, no events will be handled
   * anymore, but the socket will remain open and bound to the port. Call start() in order to
   * restart event handling.
   */
  void shutdown();

  /*!
   * \brief Writes to a client
   *
   * \param[in] fd File descriptor belonging to the client the data should be sent to. The file
   * descriptor will be given from the connection callback.
   * \param[in] buf Buffer of bytes to write
   * \param[in] buf_len Number of bytes in the buffer
   * \param[out] written Number of bytes actually written
   *
   * \returns True on success, false otherwise
   */
  bool write(const socket_t fd, const uint8_t* buf, const size_t buf_len, size_t& written);

  /*!
   * \brief Get the maximum number of clients allowed to connect to this server
   *
   * \returns The currently configured client limit. 0 means unlimited amount of clients allowed.
   */
  uint32_t getMaxClientsAllowed() const
  {
    return max_clients_allowed_;
  }

  /*!
   * \brief Set the maximum number of clients allowed to connect to this server.
   *
   * 0 means unlimited number of clients allowed.
   *
   */
  void setMaxClientsAllowed(const uint32_t& max_clients_allowed)
  {
    max_clients_allowed_ = max_clients_allowed;
  }

private:
  void init();
  void bind(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time);
  void startListen();

  //! Handles connection events
  void handleConnect();

  void handleDisconnect(const socket_t fd);

  //! read data from socket
  void readData(const socket_t fd);

  //! Event handler. Blocks until activity on any client or connection attempt
  void spin();

  //! Runs spin() as long as keep_running_ is set to true.
  void worker();

  std::atomic<bool> keep_running_;
  std::thread worker_thread_;

  std::atomic<socket_t> listen_fd_;
  int port_;

  socket_t maxfd_;
  fd_set masterfds_;
  fd_set tempfds_;

  uint32_t max_clients_allowed_;
  std::vector<socket_t> client_fds_;

  static const int INPUT_BUFFER_SIZE = 4096;
  char input_buffer_[INPUT_BUFFER_SIZE];

  std::function<void(const socket_t)> new_connection_callback_;
  std::function<void(const socket_t)> disconnect_callback_;
  std::function<void(const socket_t, char* buffer, int nbytesrecv)> message_callback_;
};

}  // namespace comm
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_TCP_SERVER_H_INCLUDED