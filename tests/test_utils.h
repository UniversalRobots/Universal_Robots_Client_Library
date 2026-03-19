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

#include <ur_client_library/primary/primary_client.h>
#include "ur_client_library/comm/tcp_server.h"

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
