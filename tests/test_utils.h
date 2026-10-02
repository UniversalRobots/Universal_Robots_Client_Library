// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright © 2025 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <ur_client_library/primary/primary_client.h>
#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/comm/tcp_socket.h"

// Loopback server with a full accept queue, so connection requests go unanswered like those to a switched-off robot.
// isUnresponsive() is false where the operating system refuses them instead (e.g. Windows).
class UnresponsiveServer
{
public:
  explicit UnresponsiveServer(const int port = 0)
  {
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ == INVALID_SOCKET)
    {
      ADD_FAILURE() << "UnresponsiveServer: could not create socket";
      return;
    }
#ifndef _WIN32
    const int reuse = 1;
    ur_setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#endif
    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(static_cast<uint16_t>(port));
    socklen_t address_len = sizeof(address);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), address_len) != 0 || ::listen(listen_fd_, 0) != 0 ||
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address), &address_len) != 0)
    {
      ADD_FAILURE() << "UnresponsiveServer: could not listen on port " << port << ", is it in use?";
      return;
    }
    port_ = ntohs(address.sin_port);

    // Fill the queue until a request stays unanswered. Aborting it closes its socket, so it cannot take a slot later.
    for (size_t i = 0; i < MAX_QUEUED_CONNECTIONS; ++i)
    {
      auto client = std::make_unique<urcl::comm::TCPSocket>();
      std::atomic<bool> done(false);
      bool connected = false;
      std::thread connect_thread([this, &client, &done, &connected]() {
        connected = client->connect("127.0.0.1", port_, 1);
        done = true;
      });
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (!done && std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      const bool answered = done;
      if (!answered)
      {
        client->disconnect();
      }
      connect_thread.join();
      if (answered && connected)
      {
        queued_connections_.push_back(std::move(client));
        continue;
      }
      unresponsive_ = !answered;
      break;
    }
  }

  ~UnresponsiveServer()
  {
    if (listen_fd_ != INVALID_SOCKET)
    {
      ur_close(listen_fd_);
    }
  }

  UnresponsiveServer(const UnresponsiveServer&) = delete;
  UnresponsiveServer& operator=(const UnresponsiveServer&) = delete;

  int getPort() const
  {
    return port_;
  }

  bool isUnresponsive() const
  {
    return unresponsive_;
  }

  // Accepts and closes all queued connections, returns their number.
  size_t acceptPending()
  {
    size_t count = 0;
    while (true)
    {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(listen_fd_, &read_fds);
      timeval no_wait{ 0, 0 };
      if (::select(static_cast<int>(listen_fd_) + 1, &read_fds, nullptr, nullptr, &no_wait) <= 0)
      {
        return count;
      }
      const socket_t fd = ::accept(listen_fd_, nullptr, nullptr);
      if (fd == INVALID_SOCKET)
      {
        return count;
      }
      ur_close(fd);
      ++count;
    }
  }

private:
  static constexpr size_t MAX_QUEUED_CONNECTIONS = 16;

  socket_t listen_fd_ = INVALID_SOCKET;
  int port_ = 0;
  bool unresponsive_ = false;
  std::vector<std::unique_ptr<urcl::comm::TCPSocket>> queued_connections_;
};

bool robotVersionLessThan(const std::string& robot_ip, const std::string& robot_version)
{
  urcl::comm::INotifier notifier;
  urcl::primary_interface::PrimaryClient primary_client(robot_ip, notifier);
  primary_client.start();
  auto version_information = primary_client.getRobotVersion();
  return *version_information < urcl::VersionInformation::fromString(robot_version);
}

class TestableTcpServer : public urcl::comm::TCPServer
{
public:
  TestableTcpServer(const int port, const bool register_callbacks = true) : TCPServer(port)
  {
    if (register_callbacks)
    {
      this->setConnectCallback(std::bind(&TestableTcpServer::connectionCallback, this, std::placeholders::_1));
      this->setMessageCallback(std::bind(&TestableTcpServer::messageCallback, this, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3));
      this->setDisconnectCallback(std::bind(&TestableTcpServer::disconnectionCallback, this, std::placeholders::_1));
    }
  }

  ~TestableTcpServer()
  {
    // unregister callbacks to avoid any callback being triggered after the server is destroyed,
    // which would cause the tests to fail due to accessing already destroyed objects.
    setConnectCallback([](const socket_t) {});
    setMessageCallback([](const socket_t, char*, int) {});
    setDisconnectCallback([](const socket_t) {});
  }

  void connectionCallback(const socket_t filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fds_.push_back(filedescriptor);
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

  void messageCallback([[maybe_unused]] const socket_t filedescriptor, char* buffer, int nbytesrecv)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    received_message_ = std::string(buffer);
    read_ = nbytesrecv;
    message_cv_.notify_one();
    message_callback_ = true;
  }

  void disconnectionCallback(const socket_t filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    for (size_t i = 0; i < client_fds_.size(); ++i)
    {
      if (client_fds_[i] == filedescriptor)
      {
        client_fds_.erase(client_fds_.begin() + i);
        break;
      }
    }
    disconnect_cv_.notify_one();
    disconnection_callback_ = true;
  }

  bool waitForConnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (connect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds),
                             [this]() { return connection_callback_ == true; }))
    {
      connection_callback_ = false;
      return true;
    }
    return false;
  }

  bool waitForMessageCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(message_mutex_);
    if (message_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds),
                             [this]() { return message_callback_ == true; }))
    {
      message_callback_ = false;
      return true;
    }
    return false;
  }

  bool waitForDisconnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (disconnect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds),
                                [this]() { return disconnection_callback_ == true; }))
    {
      disconnection_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool write(const uint8_t* buf, const size_t buf_len, size_t& written, const size_t client_index = 0)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (client_fds_.empty() || client_index >= client_fds_.size())
    {
      return false;
    }
    return TCPServer::write(client_fds_[client_index], buf, buf_len, written);
  }

  std::string getReceivedMessage()
  {
    size_t bytes_read;
    return getReceivedMessage(bytes_read);
  }

  std::string getReceivedMessage(size_t& bytes_read)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    bytes_read = read_;
    return received_message_;
  }

  std::vector<socket_t> getClientFDs()
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    return client_fds_;
  }

  size_t getPollSetSize()
  {
    return TCPServer::getPollSetSize();
  }

private:
  std::vector<socket_t> client_fds_;
  std::condition_variable connect_cv_;
  std::condition_variable message_cv_;
  std::condition_variable disconnect_cv_;
  std::mutex connect_mutex_;
  std::mutex message_mutex_;
  std::atomic<bool> connection_callback_ = false;
  std::atomic<bool> message_callback_ = false;
  std::atomic<bool> disconnection_callback_ = false;

  std::string received_message_;
  size_t read_ = 0;
};
