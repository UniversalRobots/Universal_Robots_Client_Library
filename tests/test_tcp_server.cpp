// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include <gtest/gtest.h>
#include <condition_variable>
#include <chrono>

#include <ur_client_library/comm/tcp_server.h>
#include <ur_client_library/comm/tcp_socket.h>

using namespace urcl;

class TCPServerTest : public ::testing::Test
{
protected:
  class Client : public comm::TCPSocket
  {
  public:
    Client(const int& port)
    {
      std::string host = "127.0.0.1";
      TCPSocket::setup(host, port);
    }

    void send(const std::string& text)
    {
      size_t len = text.size();
      const uint8_t* data = reinterpret_cast<const uint8_t*>(text.c_str());
      size_t written;
      TCPSocket::write(data, len, written);
    }

    std::string recv()
    {
      std::stringstream result;
      char character;
      size_t read_chars = 99;
      while (read_chars > 0)
      {
        TCPSocket::read((uint8_t*)&character, 1, read_chars);
        result << character;
        if (character == '\n')
        {
          break;
        }
      }
      return result.str();
    }

  protected:
    virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
    {
      return ::connect(socket_fd, address, address_len) == 0;
    }
  };

  // callback functions
  void connectionCallback(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fd_ = filedescriptor;
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

  void disconnectionCallback(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(disconnect_mutex_);
    client_fd_ = -1;
    disconnect_cv_.notify_one();
    disconnection_callback_ = true;
  }

  void messageCallback(const int filedescriptor, char* buffer)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    message_ = std::string(buffer);
    message_cv_.notify_one();
    message_callback_ = true;
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
    else
    {
      return false;
    }
  }

  bool waitForDisconnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(disconnect_mutex_);
    if (disconnect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        disconnection_callback_ == true)
    {
      disconnection_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
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
    else
    {
      return false;
    }
  }

  int port_ = 50001;
  std::string message_ = "";
  int client_fd_ = -1;

private:
  std::condition_variable connect_cv_;
  std::mutex connect_mutex_;

  std::condition_variable disconnect_cv_;
  std::mutex disconnect_mutex_;

  std::condition_variable message_cv_;
  std::mutex message_mutex_;

  bool connection_callback_ = false;
  bool disconnection_callback_ = false;
  bool message_callback_ = false;
};

TEST_F(TCPServerTest, socket_creation)
{
  comm::TCPServer server(port_);

  // Shouldn't be able to create antoher server on same port
  EXPECT_THROW(comm::TCPServer server2(port_), std::system_error);

  server.start();

  // We should be able to connect to the server even though the callbacks haven't been configured
  ASSERT_NO_THROW(Client client(port_));
  Client client(port_);

  // We should also be able to send message and disconnect. We wait to be absolutely sure no exception is thrown
  EXPECT_NO_THROW(client.send("message\n"));
  EXPECT_NO_THROW(waitForMessageCallback());

  EXPECT_NO_THROW(client.close());
  EXPECT_NO_THROW(waitForDisconnectionCallback());
}

TEST_F(TCPServerTest, callback_functions)
{
  comm::TCPServer server(port_);
  server.setMessageCallback(std::bind(&TCPServerTest_callback_functions_Test::messageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  server.setConnectCallback(
      std::bind(&TCPServerTest_callback_functions_Test::connectionCallback, this, std::placeholders::_1));
  server.setDisconnectCallback(
      std::bind(&TCPServerTest_callback_functions_Test::disconnectionCallback, this, std::placeholders::_1));
  server.start();

  // Check that the appropriate callback functions are called
  Client client(port_);
  EXPECT_TRUE(waitForConnectionCallback());

  client.send("message\n");
  EXPECT_TRUE(waitForMessageCallback());

  client.close();
  EXPECT_TRUE(waitForDisconnectionCallback());
}

TEST_F(TCPServerTest, unlimited_clients_allowed)
{
  comm::TCPServer server(port_);
  server.setMessageCallback(std::bind(&TCPServerTest_unlimited_clients_allowed_Test::messageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  server.setConnectCallback(
      std::bind(&TCPServerTest_unlimited_clients_allowed_Test::connectionCallback, this, std::placeholders::_1));
  server.setDisconnectCallback(
      std::bind(&TCPServerTest_unlimited_clients_allowed_Test::disconnectionCallback, this, std::placeholders::_1));
  server.start();

  // Test that a large number of clients can connect to the server
  std::vector<Client*> clients;
  Client* client;
  for (unsigned int i = 0; i < 100; ++i)
  {
    client = new Client(port_);
    ASSERT_TRUE(waitForConnectionCallback());
    clients.push_back(client);
  }
}

TEST_F(TCPServerTest, max_clients_allowed)
{
  comm::TCPServer server(port_);
  server.setMessageCallback(std::bind(&TCPServerTest_max_clients_allowed_Test::messageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  server.setConnectCallback(
      std::bind(&TCPServerTest_max_clients_allowed_Test::connectionCallback, this, std::placeholders::_1));
  server.setDisconnectCallback(
      std::bind(&TCPServerTest_max_clients_allowed_Test::disconnectionCallback, this, std::placeholders::_1));
  server.start();
  server.setMaxClientsAllowed(1);

  // Test that only one client can connect
  Client client1(port_);
  EXPECT_TRUE(waitForConnectionCallback());
  Client client2(port_);
  EXPECT_FALSE(waitForConnectionCallback());
}

TEST_F(TCPServerTest, message_transmission)
{
  comm::TCPServer server(port_);
  server.setMessageCallback(std::bind(&TCPServerTest_message_transmission_Test::messageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  server.setConnectCallback(
      std::bind(&TCPServerTest_message_transmission_Test::connectionCallback, this, std::placeholders::_1));
  server.setDisconnectCallback(
      std::bind(&TCPServerTest_message_transmission_Test::disconnectionCallback, this, std::placeholders::_1));
  server.start();

  Client client(port_);
  EXPECT_TRUE(waitForConnectionCallback());

  // Test that messages are transmitted corectly between client and server
  std::string message = "test message\n";
  client.send(message);

  EXPECT_TRUE(waitForMessageCallback());
  EXPECT_EQ(message, message_);

  size_t len = message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t written;

  ASSERT_TRUE(server.write(client_fd_, data, len, written));
  EXPECT_EQ(client.recv(), message);
}

TEST_F(TCPServerTest, client_connections)
{
  comm::TCPServer server(port_);
  server.setMessageCallback(std::bind(&TCPServerTest_client_connections_Test::messageCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
  server.setConnectCallback(
      std::bind(&TCPServerTest_client_connections_Test::connectionCallback, this, std::placeholders::_1));
  server.setDisconnectCallback(
      std::bind(&TCPServerTest_client_connections_Test::disconnectionCallback, this, std::placeholders::_1));
  server.start();

  std::string message = "text message\n";
  size_t len = message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t written;

  // Test that we can connect multiple clients
  Client client1(port_);
  EXPECT_TRUE(waitForConnectionCallback());
  int client1_fd = client_fd_;

  Client client2(port_);
  EXPECT_TRUE(waitForConnectionCallback());
  int client2_fd = client_fd_;

  Client client3(port_);
  EXPECT_TRUE(waitForConnectionCallback());
  int client3_fd = client_fd_;

  // Test that the correct clients are disconnected on the server side.
  client1.close();
  EXPECT_TRUE(waitForDisconnectionCallback());

  EXPECT_FALSE(server.write(client1_fd, data, len, written));
  EXPECT_TRUE(server.write(client2_fd, data, len, written));
  EXPECT_TRUE(server.write(client3_fd, data, len, written));

  client2.close();
  EXPECT_TRUE(waitForDisconnectionCallback());
  EXPECT_FALSE(server.write(client1_fd, data, len, written));
  EXPECT_FALSE(server.write(client2_fd, data, len, written));
  EXPECT_TRUE(server.write(client3_fd, data, len, written));

  client3.close();
  EXPECT_TRUE(waitForDisconnectionCallback());
  EXPECT_FALSE(server.write(client1_fd, data, len, written));
  EXPECT_FALSE(server.write(client2_fd, data, len, written));
  EXPECT_FALSE(server.write(client3_fd, data, len, written));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
