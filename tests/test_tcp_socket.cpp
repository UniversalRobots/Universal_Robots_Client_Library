// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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

#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <cstddef>

// This file adds a test for a deprecated function. To avoid a compiler warning in CI (where we want
// to treat warnings as errors) we suppress the warning inside this file.
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/comm/tcp_server.h>
#include "ur_client_library/types.h"

using namespace urcl;

class TCPSocketTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    server_.reset(new comm::TCPServer(60001));
    server_->setConnectCallback(std::bind(&TCPSocketTest::connectionCallback, this, std::placeholders::_1));
    server_->setMessageCallback(
        std::bind(&TCPSocketTest::messageCallback, this, std::placeholders::_1, std::placeholders::_2));
    server_->start();

    client_.reset(new Client(60001));
  }

  void TearDown()
  {
    server_.reset();
    client_.reset();
  }

  // callback functions for the tcp server
  void messageCallback(const int filedescriptor, char* buffer)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    received_message_ = std::string(buffer);
    message_cv_.notify_one();
    message_callback_ = true;
  }

  void connectionCallback(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fd_ = filedescriptor;
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

  bool waitForMessageCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(message_mutex_);
    if (message_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        message_callback_ == true)
    {
      message_callback_ = false;
      return true;
    }
    return false;
  }

  bool waitForConnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (connect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        connection_callback_ == true)
    {
      connection_callback_ = false;
      return true;
    }
    return false;
  }

  class Client : public comm::TCPSocket
  {
  public:
    Client(int port, const std::string& ip = "127.0.0.1")
    {
      port_ = port;
      ip_ = ip;
    }

    bool setup(const size_t max_num_tries = 0,
               const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10))
    {
      return TCPSocket::setup(ip_, port_, max_num_tries, reconnection_time);
    }

    void setupClientBeforeServer(const size_t max_num_tries = 0,
                                 const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10))
    {
      done_setting_up_client_ = false;
      client_setup_thread_ = std::thread(&Client::setupClient, this, port_, max_num_tries, reconnection_time);
    }

    bool waitForClientSetupThread()
    {
      unsigned int max_count = 50;
      unsigned int count = 0;
      while (count < max_count)
      {
        if (done_setting_up_client_)
        {
          client_setup_thread_.join();
          return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        count++;
      }
      client_setup_thread_.detach();
      return false;
    }

  private:
    std::thread client_setup_thread_;
    int port_;
    std::string ip_;
    bool done_setting_up_client_;

    void setupClient(int port, const size_t max_num_tries = 0,
                     std::chrono::milliseconds reconnection_time = std::chrono::seconds(10))
    {
      std::string ip = "127.0.0.1";
      TCPSocket::setup(ip, port, max_num_tries, reconnection_time);
      done_setting_up_client_ = true;
    }
  };

  std::string received_message_;
  int client_fd_;

  std::unique_ptr<comm::TCPServer> server_;
  std::unique_ptr<Client> client_;

private:
  std::condition_variable message_cv_;
  std::mutex message_mutex_;

  std::condition_variable connect_cv_;
  std::mutex connect_mutex_;

  bool connection_callback_ = false;
  bool message_callback_ = false;
};

TEST_F(TCPSocketTest, socket_state)
{
  // Client state should be invalid
  comm::SocketState expected_state = comm::SocketState::Invalid;
  comm::SocketState actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));

  // Client state should be connected after setup
  client_->setup();
  expected_state = comm::SocketState::Connected;
  actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));

  // Client state should be closed after close call
  client_->close();
  expected_state = comm::SocketState::Closed;
  actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));
}

TEST_F(TCPSocketTest, setup_client_before_server)
{
  // Make server unavailable
  server_.reset();

  client_->setupClientBeforeServer(0, std::chrono::seconds(1));

  // Make sure that the client has tried to connect to the server, before creating the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Client state should be invalid as long as the server is not available
  comm::SocketState expected_state = comm::SocketState::Invalid;
  comm::SocketState actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));

  server_.reset(new comm::TCPServer(60001));
  server_->start();

  // Test that client goes into connected state after the server has been started
  EXPECT_TRUE(client_->waitForClientSetupThread());
  expected_state = comm::SocketState::Connected;
  actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));
}

TEST_F(TCPSocketTest, get_ip)
{
  // Ip should be empty when the client is not connected to any server
  std::string expected_ip = "";
  std::string actual_ip = client_->getIP();

  EXPECT_EQ(expected_ip, actual_ip);

  client_->setup();
  expected_ip = "127.0.0.1";
  actual_ip = client_->getIP();

  EXPECT_EQ(expected_ip, actual_ip);
}

TEST_F(TCPSocketTest, write_on_non_connected_socket)
{
  std::string message = "test message";
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t len = message.size();
  size_t written;

  EXPECT_FALSE(client_->write(data, len, written));
}

TEST_F(TCPSocketTest, read_on_non_connected_socket)
{
  char character;
  size_t read_chars = 0;

  EXPECT_FALSE(client_->read((uint8_t*)&character, 1, read_chars));
}

TEST_F(TCPSocketTest, write_on_connected_socket)
{
  client_->setup();

  std::string message = "test message";
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t len = message.size();
  size_t written;
  client_->write(data, len, written);

  EXPECT_TRUE(waitForMessageCallback());
  EXPECT_EQ(message, received_message_);
}

TEST_F(TCPSocketTest, read_on_connected_socket)
{
  client_->setup();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(waitForConnectionCallback());

  std::string send_message = "test message";
  size_t len = send_message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(send_message.c_str());
  size_t written;
  server_->write(client_fd_, data, len, written);

  std::stringstream ss;
  char characters;
  size_t read_chars = 0;
  while (len > 0)
  {
    client_->read((uint8_t*)&characters, 1, read_chars);
    ss << characters;
    len -= 1;
  }

  std::string received_message = ss.str();

  EXPECT_EQ(send_message, received_message);
}

TEST_F(TCPSocketTest, get_socket_fd)
{
  // When the client is not connected to any socket the fd should be -1
  int expected_fd = -1;
  int actual_fd = client_->getSocketFD();

  EXPECT_EQ(expected_fd, actual_fd);

  client_->setup();
  actual_fd = client_->getSocketFD();

  // When the client has connected to the socket the file descriptor should be different from -1
  EXPECT_NE(expected_fd, actual_fd);

  client_->close();
  actual_fd = client_->getSocketFD();

  EXPECT_EQ(expected_fd, actual_fd);
}

TEST_F(TCPSocketTest, receive_timeout)
{
  client_->setup();

  timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  client_->setReceiveTimeout(tv);

  char character;
  size_t read_chars = 0;

  // Read should return false, when it times out
  EXPECT_FALSE(client_->read((uint8_t*)&character, 1, read_chars));
}

TEST_F(TCPSocketTest, setup_while_client_is_connected)
{
  client_->setup();

  EXPECT_FALSE(client_->setup());
}

TEST_F(TCPSocketTest, connect_non_running_robot)
{
  Client client(12321, "127.0.0.1");
  auto start = std::chrono::system_clock::now();
  EXPECT_FALSE(client.setup(2, std::chrono::milliseconds(500)));
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  // This is only a rough estimate, obviously
  EXPECT_LT(elapsed, std::chrono::milliseconds(1500));
}

TEST_F(TCPSocketTest, test_deprecated_reconnection_time_interface)
{
  client_->setReconnectionTime(std::chrono::milliseconds(100));
  EXPECT_TRUE(client_->setup(2));
}

TEST_F(TCPSocketTest, test_read_on_socket_abruptly_closed)
{
  client_->setup();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(waitForConnectionCallback());

  std::string send_message = "test message";
  size_t len = send_message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(send_message.c_str());
  size_t written;
  server_->write(client_fd_, data, len, written);

  // Simulate socket failure
  close(client_->getSocketFD());

  char characters;
  size_t read_chars = 0;
  EXPECT_FALSE(client_->read((uint8_t*)&characters, 1, read_chars));
  EXPECT_EQ(client_->getState(), comm::SocketState::Disconnected);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
