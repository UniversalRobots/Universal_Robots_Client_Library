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
#include <stdexcept>
#include <thread>
#include "ur_client_library/comm/socket_t.h"

#ifndef _WIN32
#  include <arpa/inet.h>
#  include <fcntl.h>
#  include <netinet/tcp.h>
#  include <poll.h>
#endif

#include "ur_client_library/log.h"
#include "ur_client_library/comm/tcp_socket.h"

namespace urcl
{
namespace comm
{

const std::string& socketStateToString(SocketState state)
{
  switch (state)
  {
    case SocketState::Invalid:
    {
      static const std::string invalid_state = "Invalid";
      return invalid_state;
    }
    case SocketState::Connecting:
    {
      static const std::string connecting_state = "Connecting";
      return connecting_state;
    }
    case SocketState::Connected:
    {
      static const std::string connected_state = "Connected";
      return connected_state;
    }
    case SocketState::LostConnection:
    {
      static const std::string lost_connection_state = "LostConnection";
      return lost_connection_state;
    }
    case SocketState::Disconnecting:
    {
      static const std::string disconnecting_state = "Disconnecting";
      return disconnecting_state;
    }
    case SocketState::Closed:
    {
      static const std::string closed_state = "Closed";
      return closed_state;
    }
  }
  throw std::invalid_argument("Unknown socket state");
}

namespace
{
// Time slice used while waiting for a non-blocking connect to resolve. Kept short so a
// concurrent disconnect() aborts the wait promptly even if closing the socket does not
// itself wake the wait (as is the case on Windows).
constexpr int CONNECT_POLL_SLICE_MS = 100;

// Toggle the blocking mode of a socket. Returns true on success.
bool setSocketBlocking(socket_t socket_fd, bool blocking)
{
#ifdef _WIN32
  u_long mode = blocking ? 0 : 1;
  return ::ioctlsocket(socket_fd, FIONBIO, &mode) == 0;
#else
  int flags = ::fcntl(socket_fd, F_GETFL, 0);
  if (flags < 0)
  {
    return false;
  }
  flags = blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK);
  return ::fcntl(socket_fd, F_SETFL, flags) == 0;
#endif
}

// True if the last connect() call indicated that the connection is being established
// asynchronously (the expected result for a non-blocking socket).
bool connectInProgress()
{
#ifdef _WIN32
  return ::WSAGetLastError() == WSAEWOULDBLOCK;
#else
  return errno == EINPROGRESS;
#endif
}

// Waits up to timeout_ms for the socket to become writable (connect resolved).
// Returns >0 if the socket is ready/has an event, 0 on timeout, <0 on error.
//
// poll() is used on both platforms (WSAPoll() on Windows). It avoids select()'s FD_SETSIZE
// limitation entirely and keeps a single mental model. On Windows this relies on the WSAPoll
// connect-failure fix introduced in Windows 10 version 2004 / Windows Server 2019: a failed
// non-blocking connect is reported as (POLLHUP | POLLERR | POLLWRNORM). The caller treats any
// returned event as "connect resolved" and consults SO_ERROR for the actual outcome, so it does
// not depend on which particular revents flag is set.
int waitForSocketWritable(socket_t socket_fd, int timeout_ms)
{
#ifdef _WIN32
  WSAPOLLFD pfd;
  pfd.fd = socket_fd;
  pfd.events = POLLWRNORM;  // == POLLOUT
  pfd.revents = 0;
  return ::WSAPoll(&pfd, 1, timeout_ms);
#else
  struct pollfd pfd;
  pfd.fd = socket_fd;
  pfd.events = POLLOUT;
  pfd.revents = 0;
  return ::poll(&pfd, 1, timeout_ms);
#endif
}
}  // namespace
TCPSocket::TCPSocket()
  : socket_fd_(INVALID_SOCKET)
  , state_(SocketState::Invalid)
  , target_state_(SocketState::Invalid)
  , reconnection_time_(std::chrono::seconds(10))
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
  constexpr int flag = 1;
  setSocketOptionAndWarnOnError(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag), "TCP_NODELAY");

  // macOS does not have TCP_QUICKACK
#ifdef TCP_QUICKACK
  setSocketOptionAndWarnOnError(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(flag), "TCP_QUICKACK");
#endif

  if (recv_timeout_ != nullptr)
  {
#ifdef _WIN32
    DWORD value = recv_timeout_->tv_sec * 1000;
    value += recv_timeout_->tv_usec / 1000;
    setSocketOptionAndWarnOnError(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &value, sizeof(value), "SO_RCVTIMEO");
#else
    setSocketOptionAndWarnOnError(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, recv_timeout_.get(), sizeof(timeval),
                                  "SO_RCVTIMEO");
#endif
  }
}

bool TCPSocket::openInterruptible(socket_t socket_fd, struct sockaddr* address, size_t address_len)
{
  if (!setSocketBlocking(socket_fd, false))
  {
    return false;
  }

  int connect_res = ::connect(socket_fd, address, static_cast<socklen_t>(address_len));
  bool connected = false;
  if (connect_res == 0)
  {
    // Connected immediately (common for loopback).
    connected = true;
  }
  else if (connectInProgress())
  {
    // Poll in short slices until the connect resolves, the OS connect timeout expires,
    // or a concurrent disconnect() asks us to abort.
    while (true)
    {
      if (isStopRequested())
      {
        return false;
      }
      int ready = waitForSocketWritable(socket_fd, CONNECT_POLL_SLICE_MS);
      if (ready < 0)
      {
        // poll() error (e.g. the fd was closed by disconnect()).
        return false;
      }
      if (ready == 0)
      {
        // Timeout slice elapsed without the connect resolving: re-check stop and keep waiting.
        continue;
      }
      // The socket reported an event: query SO_ERROR to find out whether the connect succeeded.
      int so_error = 0;
      socklen_t len = sizeof(so_error);
      if (::getsockopt(socket_fd, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len) < 0)
      {
        return false;
      }
      connected = (so_error == 0);
      break;
    }
  }
  else
  {
    // Immediate, permanent failure (e.g. connection refused).
    connected = false;
  }

  if (connected && !setSocketBlocking(socket_fd, true))
  {
    // Could not restore blocking mode; treat the connection as failed.
    return false;
  }
  return connected;
}

bool TCPSocket::setupInternal(const std::string& host, const int port, const size_t max_num_tries,
                              const std::chrono::milliseconds reconnection_time)
{
  // This can be removed once we remove the setReconnectionTime() method
  auto reconnection_time_resolved = reconnection_time;
  if (reconnection_time_modified_deprecated_)
  {
    URCL_LOG_WARN("TCPSocket::setupInternal(): Reconnection time was modified using `setReconnectionTime()` which is "
                  "deprecated. Please change your code to set reconnection_time through the `(re)connect()` method "
                  "directly. The value passed to this function will be ignored.");
    reconnection_time_resolved = reconnection_time_;
  }

  if (state_ == SocketState::Connected)
    return false;

  state_ = SocketState::Connecting;

  URCL_LOG_DEBUG("Setting up connection: %s:%d", host.c_str(), port);

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
    if (isStopRequested())
      return false;

    if (getaddrinfo(host_name, service.c_str(), &hints, &result) != 0)
    {
      URCL_LOG_ERROR("Failed to get address for %s:%d", host.c_str(), port);
      return false;
    }
    // loop through the list of addresses until we find one that's connectable
    for (struct addrinfo* p = result; p != nullptr; p = p->ai_next)
    {
      socket_fd_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);

      if (socket_fd_ != -1 && openInterruptible(socket_fd_, p->ai_addr, p->ai_addrlen))
      {
        connected = true;
        break;
      }

      if (isStopRequested())
      {
        freeaddrinfo(result);
        freeFileDescriptor();
        return false;
      }
    }

    freeaddrinfo(result);

    if (!connected)
    {
      if (++connect_counter >= max_num_tries && max_num_tries > 0)
      {
        URCL_LOG_ERROR("Failed to establish connection for %s:%d after %d tries", host.c_str(), port, max_num_tries);
        freeFileDescriptor();
        return false;
      }
      else
      {
        std::stringstream ss;
        ss << "Failed to connect to robot on IP " << host_name << ":" << port
           << ". Please check that the robot is booted and reachable on " << host_name << ". Retrying in "
           << std::chrono::duration_cast<std::chrono::duration<float>>(reconnection_time_resolved).count()
           << " seconds.";
        URCL_LOG_ERROR("%s", ss.str().c_str());
        // Sleep in short slices so that a concurrent disconnect() (e.g. from ~RTDEClient or
        // ~PrimaryClient before joining the reconnect thread) can interrupt the back-off promptly.
        const auto sleep_slice = std::chrono::milliseconds(100);
        for (auto slept = std::chrono::milliseconds(0); slept < reconnection_time_resolved && !isStopRequested();
             slept += sleep_slice)
        {
          std::this_thread::sleep_for(sleep_slice);
        }
        if (isStopRequested())
        {
          freeFileDescriptor();
          return false;
        }
      }
    }
  }
  setupOptions();
  // Mark Connected only if no deliberate disconnect() slipped in while we were finishing up; a late
  // disconnect() must win so we do not advertise a usable socket that was just torn down.
  SocketState expected = SocketState::Connecting;
  if (!state_.compare_exchange_strong(expected, SocketState::Connected))
  {
    close();
    return false;
  }
  URCL_LOG_DEBUG("Connection established for %s:%d", host.c_str(), port);
  return connected;
}

bool TCPSocket::connect(const std::string& host, const int port, const size_t max_num_tries,
                        const std::chrono::milliseconds reconnection_time)
{
  if (state_ == SocketState::Connected)
  {
    URCL_LOG_ERROR("Connect called on a socket that is already connected");
    return false;
  }
  target_state_ = SocketState::Connected;
  if (!setupInternal(host, port, max_num_tries, reconnection_time))
  {
    disconnect();
    return false;
  }
  return true;
}

bool TCPSocket::reconnect(const std::string& host, const int port, const size_t max_num_tries,
                          const std::chrono::milliseconds reconnection_time)
{
  if (state_ != SocketState::LostConnection)
  {
    URCL_LOG_ERROR("Reconnect called on a socket that is not in LostConnection state");
    return false;
  }
  setTargetStateUnlessStopRequested(SocketState::Connected);
  if (!setupInternal(host, port, max_num_tries, reconnection_time))
  {
    // If we failed to reconnect, we need to set the target state back to LostConnection so that
    auto expected_state = SocketState::Connecting;
    state_.compare_exchange_strong(expected_state, SocketState::LostConnection);
    return false;
  }
  return true;
}

bool TCPSocket::setTargetStateUnlessStopRequested(SocketState desired)
{
  SocketState current_target = target_state_.load();
  if (current_target == desired)
  {
    return true;  // already tracking target
  }

  // Lock-free compare-and-swap: move state_ to `desired` unless a deliberate disconnect()
  // (Disconnecting/Disconnected) is in effect. This is not an unbounded spin: each iteration
  // either succeeds, or observes that another thread changed state_ and re-evaluates. We use
  // compare_exchange_strong so there are no spurious retries; the loop can only re-iterate when a
  // concurrent writer genuinely changed state_, and the only such writers (a racing disconnect(),
  // or a paired close()) make a bounded number of writes, so it terminates promptly.
  while (target_state_ != SocketState::Closed)
  {
    if (target_state_.compare_exchange_strong(current_target, desired))
    {
      return true;  // successfully moved to `desired`
    }
  }
  return false;  // a deliberate disconnect() is in effect; state_ left untouched
}

void TCPSocket::freeFileDescriptor()
{
  if (socket_fd_ >= 0)
  {
    ::ur_close(socket_fd_);
    socket_fd_ = INVALID_SOCKET;
  }
}

void TCPSocket::close()
{
  if (state_ != SocketState::Closed)
  {
    // closing overwrites everything, so we do not need to check for stopRequested() here
    target_state_ = SocketState::Closed;
    state_ = SocketState::Disconnecting;
  }
  freeFileDescriptor();
  state_ = SocketState::Closed;
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
    state_ = SocketState::LostConnection;
    return false;
  }
  else if (res < 0)
  {
    res = 0;
#ifdef _WIN32
    int code = ::WSAGetLastError();
    if (code != WSAETIMEDOUT && code != WSAEWOULDBLOCK)
    {
      state_ = SocketState::LostConnection;
    }
#else
    if (!(errno == EAGAIN || errno == EWOULDBLOCK))
    {
      // any permanent error should be detected early
      state_ = SocketState::LostConnection;
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
