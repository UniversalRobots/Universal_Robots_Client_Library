/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Jarek Potiuk (low bandwidth trajectory follower)
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

#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>

#ifndef _WIN32
#  include <arpa/inet.h>
#  include <netinet/tcp.h>
#  include <fcntl.h>
#  include <sys/select.h>
#endif

#include "ur_client_library/log.h"
#include "ur_client_library/comm/tcp_socket.h"

namespace urcl
{
namespace comm
{
TCPSocket::TCPSocket()
  : socket_fd_(INVALID_SOCKET), state_(SocketState::Invalid), reconnection_time_(std::chrono::seconds(10))
{
#ifdef _WIN32
  WSAData data;
  ::WSAStartup(MAKEWORD(1, 1), &data);
#endif  // _WIN32
}
TCPSocket::~TCPSocket()
{
  close();
}

void TCPSocket::setupOptions()
{
  int flag = 1;
  ur_setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));

  // macOS does not have TCP_QUICKACK
#ifdef TCP_QUICKACK
  ur_setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(int));
#endif

  if (recv_timeout_ != nullptr)
  {
#ifdef _WIN32
    DWORD value = recv_timeout_->tv_sec * 1000;
    value += recv_timeout_->tv_usec / 1000;
    ur_setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &value, sizeof(value));
#else
    ur_setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, recv_timeout_.get(), sizeof(timeval));
#endif
  }
}

bool TCPSocket::setup(const std::string& host, const int port, const size_t max_num_tries,
                      const std::chrono::milliseconds reconnection_time, const std::chrono::milliseconds timeout)
{
  // This can be removed once we remove the setReconnectionTime() method
  auto reconnection_time_resolved = reconnection_time;
  if (reconnection_time_modified_deprecated_)
  {
    URCL_LOG_WARN("TCPSocket::setup(): Reconnection time was modified using `setReconnectionTime()` which is "
                  "deprecated. Please change your code to set reconnection_time through the `setup()` method "
                  "directly. The value passed to this function will be ignored.");
    reconnection_time_resolved = reconnection_time_;
  }

  if (state_ == SocketState::Connected)
    return false;

  URCL_LOG_DEBUG("Setting up connection: %s:%d", host.c_str(), port);

  // gethostbyname() is deprecated so use getadderinfo() as described in:
  // https://beej.us/guide/bgnet/html/#getaddrinfoprepare-to-launch

  const char* host_name = host.empty() ? nullptr : host.c_str();
  std::string service = std::to_string(port);
  struct addrinfo hints, *result;
  std::memset(&hints, 0, sizeof(hints));

  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;

  size_t connect_counter = 0;
  bool connected = false;
  while (!connected)
  {
    if (getaddrinfo(host_name, service.c_str(), &hints, &result) != 0)
    {
      URCL_LOG_ERROR("Failed to get address for %s:%d", host.c_str(), port);
      return false;
    }

    std::error_code socket_error = std::make_error_code(std::errc::address_not_available);

    for (struct addrinfo* p = result; p != nullptr; p = p->ai_next)
    {
      socket_fd_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);

      if (socket_fd_ == -1)
      {
        socket_error = getLastSocketErrorCode();
        continue;
      }
#ifndef _WIN32
      if (socket_fd_ >= FD_SETSIZE)
      {
        socket_error = std::make_error_code(std::errc::value_too_large);
        ::ur_close(socket_fd_);
        socket_fd_ = INVALID_SOCKET;
        continue;
      }
#endif

      bool connect_success = false;

#ifndef _WIN32
      int flags = ::fcntl(socket_fd_, F_GETFL, 0);
      if (flags < 0)
      {
        socket_error = getLastSocketErrorCode();
        ::ur_close(socket_fd_);
        socket_fd_ = INVALID_SOCKET;
        continue;
      }
      ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
#else
      unsigned long mode = 1;
      ::ioctlsocket(socket_fd_, FIONBIO, &mode);
#endif

#ifndef _WIN32
      int connect_res = ::connect(socket_fd_, p->ai_addr, static_cast<socklen_t>(p->ai_addrlen));
      bool is_in_progress = (connect_res != 0 && errno == EINPROGRESS);
#else
      int connect_res = ::connect(socket_fd_, p->ai_addr, static_cast<int>(p->ai_addrlen));
      bool is_in_progress = (connect_res != 0 && ::WSAGetLastError() == WSAEWOULDBLOCK);
#endif

      if (connect_res == 0)
      {
        connect_success = true;
      }
      else if (is_in_progress)
      {
        auto timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count();
        struct timeval tv;
        tv.tv_sec = static_cast<long>(timeout_ms / 1000);
        tv.tv_usec = static_cast<long>((timeout_ms % 1000) * 1000);

        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(socket_fd_, &write_fds);

        fd_set except_fds;
        FD_ZERO(&except_fds);
        FD_SET(socket_fd_, &except_fds);

        int select_res = ::select(static_cast<int>(socket_fd_ + 1), nullptr, &write_fds, &except_fds, &tv);

        if (select_res > 0)
        {
          int so_error = 0;
#ifndef _WIN32
          socklen_t len = sizeof(so_error);
          int opt_res = ::getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &so_error, &len);
#else
          int len = sizeof(so_error);
          int opt_res = ::getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
#endif
          if (opt_res != 0)
          {
            socket_error = getLastSocketErrorCode();
          }
          else if (so_error == 0 && FD_ISSET(socket_fd_, &write_fds))
          {
            connect_success = true;
          }
          else
          {
            if (so_error != 0)
            {
#ifdef _WIN32
              socket_error = std::error_code(so_error, std::system_category());
#else
              socket_error = std::error_code(so_error, std::generic_category());
#endif
            }
            else
            {
              socket_error = std::make_error_code(std::errc::connection_refused);
            }
          }
        }
        else if (select_res == 0)
        {
          socket_error = std::make_error_code(std::errc::timed_out);
        }
        else
        {
          socket_error = getLastSocketErrorCode();
        }
      }
      else
      {
        socket_error = getLastSocketErrorCode();
      }

#ifndef _WIN32
      ::fcntl(socket_fd_, F_SETFL, flags);
#else
      mode = 0;
      ::ioctlsocket(socket_fd_, FIONBIO, &mode);
#endif

      if (connect_success)
      {
        connected = true;
        break;
      }
      else
      {
        ::ur_close(socket_fd_);
        socket_fd_ = INVALID_SOCKET;
      }
    }

    freeaddrinfo(result);

    if (!connected)
    {
      state_ = SocketState::Invalid;
      if (++connect_counter >= max_num_tries && max_num_tries > 0)
      {
        URCL_LOG_ERROR("Failed to establish connection for %s:%d after %d tries", host.c_str(), port, max_num_tries);
        return false;
      }
      else
      {
        std::stringstream ss;
        ss << "Failed to connect to robot on IP " << host_name << ":" << port << ". Reason: " << socket_error.message()
           << ". Retrying in "
           << std::chrono::duration_cast<std::chrono::duration<float>>(reconnection_time_resolved).count()
           << " seconds.";
        URCL_LOG_ERROR("%s", ss.str().c_str());
        std::this_thread::sleep_for(reconnection_time_resolved);
      }
    }
  }
  setupOptions();
  state_ = SocketState::Connected;
  URCL_LOG_DEBUG("Connection established for %s:%d", host.c_str(), port);
  return connected;
}

void TCPSocket::close()
{
  if (socket_fd_ >= 0)
  {
    state_ = SocketState::Closed;
    ::ur_close(socket_fd_);
    socket_fd_ = INVALID_SOCKET;
  }
}

std::string TCPSocket::getIP() const
{
  sockaddr_in name;
  socklen_t len = sizeof(name);
  int res = ::getsockname(socket_fd_, (sockaddr*)&name, &len);

  if (res < 0)
  {
    URCL_LOG_ERROR("Could not get local IP");
    return std::string();
  }

  char buf[128];
  inet_ntop(AF_INET, &name.sin_addr, buf, sizeof(buf));
  return std::string(buf);
}

bool TCPSocket::read(char* character)
{
  size_t read_chars;
  // It's inefficient, but in our case we read very small messages
  // and the overhead connected with reading character by character is
  // negligible - adding buffering would complicate the code needlessly.
  return read((uint8_t*)character, 1, read_chars);
}

bool TCPSocket::read(uint8_t* buf, const size_t buf_len, size_t& read)
{
  read = 0;

  if (state_ != SocketState::Connected)
    return false;

#ifdef _WIN32
  ssize_t res = ::recv(socket_fd_, reinterpret_cast<char*>(buf), static_cast<const socklen_t>(buf_len), 0);
#else
  ssize_t res = ::recv(socket_fd_, buf, buf_len, 0);
#endif

  if (res == 0)
  {
    state_ = SocketState::Disconnected;
    return false;
  }
  else if (res < 0)
  {
    res = 0;
#ifdef _WIN32
    int code = ::WSAGetLastError();
    if (code != WSAETIMEDOUT && code != WSAEWOULDBLOCK)
    {
      state_ = SocketState::Disconnected;
    }
#else
    if (!(errno == EAGAIN || errno == EWOULDBLOCK))
    {
      // any permanent error should be detected early
      state_ = SocketState::Disconnected;
    }
#endif
    return false;
  }

  read = static_cast<size_t>(res);
  return true;
}

bool TCPSocket::write(const uint8_t* buf, const size_t buf_len, size_t& written)
{
  written = 0;

  if (state_ != SocketState::Connected)
  {
    URCL_LOG_ERROR("Attempt to write on a non-connected socket");
    return false;
  }

  size_t remaining = buf_len;

  // handle partial sends
  while (written < buf_len)
  {
    ssize_t sent =
        ::send(socket_fd_, reinterpret_cast<const char*>(buf + written), static_cast<socklen_t>(remaining), 0);

    if (sent <= 0)
    {
      URCL_LOG_ERROR("Sending data through socket failed.");
      return false;
    }

    written += static_cast<size_t>(sent);
    remaining -= sent;
  }

  return true;
}

void TCPSocket::setReceiveTimeout(const timeval& timeout)
{
  recv_timeout_.reset(new timeval(timeout));

  if (state_ == SocketState::Connected)
  {
    setupOptions();
  }
}

void TCPSocket::setReconnectionTime(const std::chrono::milliseconds reconnection_time)
{
  URCL_LOG_ERROR("Calling setReconnectionTime is deprecated. Reconnection timeout is passed to the setup method "
                 "directly.");
  reconnection_time_ = reconnection_time;
  reconnection_time_modified_deprecated_ = true;
}

}  // namespace comm
}  // namespace urcl
