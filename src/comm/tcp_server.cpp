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

#include <iostream>

#include <sstream>
#include <cstring>
#include <fcntl.h>
#include <algorithm>
#include <system_error>

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
  ur_close(listen_fd_);
}

void TCPServer::init()
{
  socket_t err = (listen_fd_ = socket(AF_INET, SOCK_STREAM, 0));
  if (err < 0)
  {
    throw std::system_error(std::error_code(errno, std::generic_category()), "Failed to create socket endpoint");
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
  keep_running_ = false;

  socket_t shutdown_socket = ::socket(AF_INET, SOCK_STREAM, 0);
  if (shutdown_socket == INVALID_SOCKET)
  {
    throw std::system_error(std::error_code(errno, std::generic_category()), "Unable to create shutdown socket.");
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
  address.sin_port = htons(port_);

  ::connect(shutdown_socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address));

  // After the event loop has finished the thread will be joinable.
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
    URCL_LOG_DEBUG("Worker thread joined.");
  }
}

void TCPServer::bind(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;

  // INADDR_ANY is a special constant that signalizes "ANY IFACE",
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(port_);
  int err = -1;
  size_t connection_counter = 0;
  do
  {
    err = ::bind(listen_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (err == -1)
    {
      std::ostringstream ss;
      ss << "Failed to bind socket for port " << port_ << " to address. Reason: " << strerror(errno);

      if (connection_counter++ < max_num_tries || max_num_tries == 0)
      {
        std::this_thread::sleep_for(reconnection_time);
        ss << "Retrying in " << std::chrono::duration_cast<std::chrono::duration<float>>(reconnection_time).count()
           << " seconds";
        URCL_LOG_WARN("%s", ss.str().c_str());
      }
      else
      {
        throw std::system_error(std::error_code(errno, std::generic_category()), ss.str());
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
    throw std::system_error(std::error_code(errno, std::generic_category()), ss.str());
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
    std::ostringstream ss;
    ss << "Failed to accept connection request on port  " << port_;
    throw std::system_error(std::error_code(errno, std::generic_category()), ss.str());
  }

  if (client_fds_.size() < max_clients_allowed_ || max_clients_allowed_ == 0)
  {
    client_fds_.push_back(client_fd);
    FD_SET(client_fd, &masterfds_);
    if (client_fd > maxfd_)
    {
      maxfd_ = client_fd;
    }
    if (new_connection_callback_)
    {
      new_connection_callback_(client_fd);
    }
  }
  else
  {
    URCL_LOG_WARN("Connection attempt on port %d while maximum number of clients (%d) is already connected. Closing "
                  "connection.",
                  port_, max_clients_allowed_);
    ur_close(client_fd);
  }
}

void TCPServer::spin()
{
  tempfds_ = masterfds_;

  // blocks until activity on any socket from tempfds
  int sel = select(static_cast<int>(maxfd_ + 1), &tempfds_, NULL, NULL, NULL);
  if (sel < 0)
  {
    URCL_LOG_ERROR("select() failed. Shutting down socket event handler.");
    keep_running_ = false;
    return;
  }

  if (!keep_running_)
  {
    return;
  }

  // Check which fd has an activity
  for (socket_t i = 0; i <= maxfd_; i++)
  {
    if (FD_ISSET(i, &tempfds_))
    {
      URCL_LOG_DEBUG("Activity on FD %d", i);
      if (listen_fd_ == i)
      {
        // Activity on the listen_fd means we have a new connection
        handleConnect();
      }
      else
      {
        readData(i);
      }
    }
  }
}

void TCPServer::handleDisconnect(const socket_t fd)
{
  URCL_LOG_DEBUG("%d disconnected.", fd);
  ur_close(fd);
  if (disconnect_callback_)
  {
    disconnect_callback_(fd);
  }
  FD_CLR(fd, &masterfds_);

  for (size_t i = 0; i < client_fds_.size(); ++i)
  {
    if (client_fds_[i] == fd)
    {
      client_fds_.erase(client_fds_.begin() + i);
      break;
    }
  }
}

void TCPServer::readData(const socket_t fd)
{
  memset(input_buffer_, 0, INPUT_BUFFER_SIZE);  // clear input buffer
  int nbytesrecv = recv(fd, input_buffer_, INPUT_BUFFER_SIZE, 0);
  if (nbytesrecv > 0)
  {
    if (message_callback_)
    {
      message_callback_(fd, input_buffer_, nbytesrecv);
    }
  }
  else
  {
    if (nbytesrecv < 0)
    {
      if (errno == ECONNRESET)  // if connection gets reset by client, we want to suppress this output
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
    handleDisconnect(fd);
  }
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

  size_t remaining = buf_len;

  // handle partial sends
  while (written < buf_len)
  {
    ssize_t sent = ::send(fd, reinterpret_cast<const char*>(buf + written), static_cast<socklen_t>(remaining), 0);

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
