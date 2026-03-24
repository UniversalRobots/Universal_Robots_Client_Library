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

#include <ur_client_library/log.h>
#include <ur_client_library/comm/tcp_server.h>

#include <algorithm>
#include <iostream>

#include <sstream>
#include <cstring>
#include "ur_client_library/comm/socket_t.h"
#include <fcntl.h>

namespace urcl
{
namespace comm
{
TCPServer::TCPServer(const int port, const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
  : port_(port), maxfd_(0), max_clients_allowed_(0)
{
#ifdef _WIN32
  WSAData data;
  ::WSAStartup(MAKEWORD(1, 1), &data);
#endif  // _WIN32

  init();
  bind(max_num_tries, reconnection_time);
  startListen();
}

TCPServer::~TCPServer()
{
  URCL_LOG_DEBUG("Destroying TCPServer object.");
  shutdown();
}

void TCPServer::init()
{
  listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd_ == INVALID_SOCKET)
  {
    throw makeSocketError("Failed to create socket endpoint");
  }
  int flag = 1;
#ifndef _WIN32
  ur_setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
#endif
  ur_setsockopt(listen_fd_, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(int));

  URCL_LOG_DEBUG("Created socket with FD %d", (int)listen_fd_);

  FD_ZERO(&masterfds_);
  FD_ZERO(&tempfds_);
}

void TCPServer::shutdown()
{
  std::unique_lock<std::mutex> listen_lk(listen_fd_mutex_, std::try_to_lock);
  if (listen_fd_ == INVALID_SOCKET)
  {
    URCL_LOG_INFO("Listen FD already closed by another thread. Nothing to do here.");
    return;
  }
  if (!listen_lk.owns_lock())
  {
    URCL_LOG_WARN("Could not acquire lock for listen FD when shutting down. Is there another thread shutting the "
                  "server down already? Waiting for lock to be released.");
    listen_lk.lock();
    if (listen_fd_ == INVALID_SOCKET)
    {
      URCL_LOG_INFO("Listen FD already closed by another thread. Nothing to do here.");
      return;
    }
  }
  keep_running_ = false;

  socket_t shutdown_socket = ::socket(AF_INET, SOCK_STREAM, 0);
  if (shutdown_socket == INVALID_SOCKET)
  {
    throw makeSocketError("Unable to create shutdown socket.");
  }

#ifdef _WIN32
  unsigned long mode = 1;
  ::ioctlsocket(shutdown_socket, FIONBIO, &mode);
#else
  int flags = ::fcntl(shutdown_socket, F_GETFL, 0);
  if (flags >= 0)
  {
    ::fcntl(shutdown_socket, F_SETFL, flags | O_NONBLOCK);
  }
#endif

  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(static_cast<uint16_t>(port_));

  ::connect(shutdown_socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address));

  // After the event loop has finished the thread will be joinable.
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
    URCL_LOG_DEBUG("Worker thread joined.");
  }

  std::lock_guard<std::mutex> lk(clients_mutex_);
  for (const auto& client_fd : client_fds_)
  {
    ur_close(client_fd);
  }
  // This will effectively deactivate the disconnection handler.
  client_fds_.clear();
  ur_close(shutdown_socket);
  ur_close(listen_fd_);
  listen_fd_ = INVALID_SOCKET;
}

void TCPServer::bind(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;

  // INADDR_ANY is a special constant that signalizes "ANY IFACE",
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(static_cast<uint16_t>(port_));
  int err = -1;
  size_t connection_counter = 0;
  do
  {
    err = ::bind(listen_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (err == -1)
    {
      auto error_code = getLastSocketErrorCode();
      std::ostringstream ss;
      ss << "Failed to bind socket for port " << port_ << " to address. Reason: " << error_code.message();

      if (connection_counter++ < max_num_tries || max_num_tries == 0)
      {
        std::this_thread::sleep_for(reconnection_time);
        ss << "Retrying in " << std::chrono::duration_cast<std::chrono::duration<float>>(reconnection_time).count()
           << " seconds";
        URCL_LOG_WARN("%s", ss.str().c_str());
      }
      else
      {
        throw std::system_error(error_code, ss.str());
      }
    }
  } while (err == -1 && (connection_counter <= max_num_tries || max_num_tries == 0));

  URCL_LOG_DEBUG("Bound %d:%d to FD %d", server_addr.sin_addr.s_addr, port_, (int)listen_fd_);

  FD_SET(listen_fd_, &masterfds_);
  maxfd_ = listen_fd_;
}

void TCPServer::startListen()
{
  int err = listen(listen_fd_, 1);
  if (err == -1)
  {
    std::ostringstream ss;
    ss << "Failed to start listen on port " << port_;
    throw makeSocketError(ss.str());
  }
  struct sockaddr_in sin;
  socklen_t len = sizeof(sin);
  if (getsockname(listen_fd_, (struct sockaddr*)&sin, &len) == -1)
  {
    URCL_LOG_ERROR("getsockname() failed to get port number for listening socket: %s",
                   getLastSocketErrorCode().message().c_str());
  }

  else
  {
    port_ = ntohs(sin.sin_port);
  }
  URCL_LOG_DEBUG("Listening on port %d", port_);
}

void TCPServer::handleConnect()
{
  struct sockaddr_storage client_addr;
  socklen_t addrlen = sizeof(client_addr);
  socket_t client_fd = accept(listen_fd_, (struct sockaddr*)&client_addr, &addrlen);
  if (client_fd == INVALID_SOCKET)
  {
    URCL_LOG_ERROR("Failed to accept connection request on port %d. Reason: %s", port_,
                   getLastSocketErrorCode().message().c_str());
    return;
  }

#ifdef _WIN32
  bool set_size_exceeded = client_fds_.size() >= FD_SETSIZE - 1;  // -1 because listen_fd_ also occupies one
                                                                  // slot in masterfds_
#else
  bool set_size_exceeded = client_fd >= FD_SETSIZE;  // On Unix-like systems, the client FD itself must be less than
                                                     // FD_SETSIZE, otherwise it cannot be added to the fd_set.
#endif

  if (set_size_exceeded)
  {
    URCL_LOG_ERROR("Accepted client FD %d exceeds FD_SETSIZE (%d). Closing connection.", (int)client_fd, FD_SETSIZE);
    ur_close(client_fd);
    return;
  }

  bool accepted = false;

  {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    if (client_fds_.size() < max_clients_allowed_ || max_clients_allowed_ == 0)
    {
      client_fds_.push_back(client_fd);
      FD_SET(client_fd, &masterfds_);
      if (client_fd > maxfd_)
      {
        maxfd_ = client_fd;
      }
      accepted = true;
    }
    else
    {
      URCL_LOG_WARN("Connection attempt on port %d while maximum number of clients (%d) is already connected. Closing "
                    "connection.",
                    port_, max_clients_allowed_);
      ur_close(client_fd);
    }
  }
  {
    std::lock_guard<std::mutex> lk(callback_mutex_);
    if (new_connection_callback_ && accepted)
    {
      new_connection_callback_(client_fd);
    }
  }
}

void TCPServer::spin()
{
  tempfds_ = masterfds_;

  timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // blocks until activity on any socket from tempfds
  int sel = select(static_cast<int>(maxfd_ + 1), &tempfds_, NULL, NULL, &timeout);
  if (sel < 0)
  {
    URCL_LOG_ERROR("select() failed. Shutting down socket event handler.");
    keep_running_ = false;
    return;
  }

  if (!keep_running_ || sel == 0)
  {
    return;
  }

  if (FD_ISSET(listen_fd_, &tempfds_))
  {
    URCL_LOG_DEBUG("Activity on listen FD %d", (int)listen_fd_);
    handleConnect();
  }

  std::vector<socket_t> disconnected_clients;
  std::vector<socket_t> client_fds_with_activity;

  {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    for (const auto& client_fd : client_fds_)
    {
      if (FD_ISSET(client_fd, &tempfds_))
      {
        URCL_LOG_DEBUG("Activity on client FD %d", (int)client_fd);
        client_fds_with_activity.push_back(client_fd);
      }
    }
  }
  // We handle client activity outside the clients_mutex_ lock to avoid holding it during potentially slow I/O and
  // message callbacks.
  // The clients_mutex_ lock is only needed to protect the client_fds_ vector, but once we have copied the FDs with
  // activity to a separate vector, we can safely handle them without holding the lock.
  for (const auto& client_fd : client_fds_with_activity)
  {
    if (!readData(client_fd))
    {
      disconnected_clients.push_back(client_fd);
    }
  }
  for (const auto& client_fd : disconnected_clients)
  {
    handleDisconnect(client_fd);
  }
}

void TCPServer::handleDisconnect(const socket_t fd)
{
  URCL_LOG_DEBUG("%d disconnected.", fd);
  {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    ur_close(fd);
    FD_CLR(fd, &masterfds_);

    for (size_t i = 0; i < client_fds_.size(); ++i)
    {
      if (client_fds_[i] == fd)
      {
        client_fds_.erase(client_fds_.begin() + i);
        break;
      }
    }

    maxfd_ = listen_fd_;
    for (const auto& client_fd : client_fds_)
    {
      if (client_fd > maxfd_)
      {
        maxfd_ = client_fd;
      }
    }
  }

  {
    std::lock_guard<std::mutex> lk(callback_mutex_);
    if (disconnect_callback_ && keep_running_)
    {
      disconnect_callback_(fd);
    }
  }
}

bool TCPServer::readData(const socket_t fd)
{
  memset(input_buffer_, 0, INPUT_BUFFER_SIZE);  // clear input buffer
  int nbytesrecv = recv(fd, input_buffer_, INPUT_BUFFER_SIZE, 0);
  if (nbytesrecv > 0)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    if (message_callback_)
    {
      message_callback_(fd, input_buffer_, nbytesrecv);
    }
  }
  else
  {
    if (nbytesrecv < 0)
    {
      auto check_err = []() {
#ifdef _WIN32
        return WSAGetLastError() == WSAECONNRESET;
#else
        return errno == ECONNRESET;
#endif
      };
      if (check_err())  // if connection gets reset by client, we want to suppress this output
      {
        URCL_LOG_DEBUG("client from FD %d sent a connection reset package.", fd);
      }
      else
      {
        URCL_LOG_ERROR("recv() on FD %d failed.", fd);
      }
    }
    else
    {
      // normal disconnect
    }
    return false;
  }
  return true;
}

void TCPServer::worker()
{
  while (keep_running_)
  {
    spin();
  }
  URCL_LOG_DEBUG("Finished worker thread of TCPServer");
}

void TCPServer::start()
{
  URCL_LOG_DEBUG("Starting worker thread");
  keep_running_ = true;
  worker_thread_ = std::thread(&TCPServer::worker, this);
}

bool TCPServer::write(const socket_t fd, const uint8_t* buf, const size_t buf_len, size_t& written)
{
  written = 0;
  if (fd == INVALID_SOCKET)
  {
    URCL_LOG_ERROR("Invalid socket provided for writing.");
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(clients_mutex_);
    if (std::find(client_fds_.begin(), client_fds_.end(), fd) == client_fds_.end())
    {
      URCL_LOG_ERROR("Trying to write to FD %d, but this client is not connected.", fd);
      return false;
    }
  }

  // We don't use a lock around the send call here, since writing on a closed socket would raise
  // an error anyway, and the client FD is only removed from client_fds_ after the socket is
  // closed. Thus, even if the client gets disconnected right after the check, the send call will
  // just fail and return false, which is the expected behavior.
  return writeUnchecked(fd, buf, buf_len, written);
}

bool TCPServer::writeUnchecked(const socket_t fd, const uint8_t* buf, const size_t buf_len, size_t& written)
{
  size_t remaining = buf_len;

  // handle partial sends
  while (written < buf_len)
  {
    ssize_t sent =
        ::send(fd, reinterpret_cast<const char*>(buf + written), static_cast<socklen_t>(remaining), MSG_NOSIGNAL);

    if (sent <= 0)
    {
      URCL_LOG_ERROR("Sending data through socket failed.");
      return false;
    }

    written += sent;
    remaining -= sent;
  }

  return true;
}

}  // namespace comm
}  // namespace urcl
