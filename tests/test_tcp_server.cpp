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
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#ifndef _WIN32
#  include <fcntl.h>
#  include <unistd.h>
#  include <sys/resource.h>
#endif
#include "test_utils.h"

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
        if (TCPSocket::read((uint8_t*)&character, 1, read_chars))
        {
          result << character;
          if (character == '\n')
          {
            break;
          }
        }
      }
      return result.str();
    }
  };

  int port_ = 50001;
};

TEST_F(TCPServerTest, socket_creation)
{
  TestableTcpServer server(port_, false);  // do not register callbacks

  // Shouldn't be able to create antoher server on same port
  EXPECT_THROW(comm::TCPServer server2(port_, 1, std::chrono::milliseconds(1)), std::system_error);

  server.start();

  // We should be able to connect to the server even though the callbacks haven't been configured
  ASSERT_NO_THROW(Client client(port_));
  Client client(port_);

  // We should also be able to send message and disconnect. We wait to be absolutely sure no exception is thrown
  EXPECT_NO_THROW(client.send("message\n"));
  EXPECT_NO_THROW(server.waitForMessageCallback());

  EXPECT_NO_THROW(client.close());
  EXPECT_NO_THROW(server.waitForDisconnectionCallback());
}

TEST_F(TCPServerTest, callback_functions)
{
  TestableTcpServer server(port_);
  server.start();

  // Check that the appropriate callback functions are called
  Client client(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  client.send("message\n");
  EXPECT_TRUE(server.waitForMessageCallback());

  client.close();
  EXPECT_TRUE(server.waitForDisconnectionCallback());
}

TEST_F(TCPServerTest, many_clients_allowed)
{
  TestableTcpServer server(port_);
  server.start();

#ifdef _WIN32
  // Windows has a maximum of 64 sockets per process, so we can only test with 63 clients since the server also uses one
  // socket.
  constexpr int num_clients = 63;
#else
  constexpr int num_clients = 100;
#endif

  // Test that a large number of clients can connect to the server
  std::vector<std::unique_ptr<Client>> clients;
  std::unique_ptr<Client> client;
  for (unsigned int i = 0; i < num_clients; ++i)
  {
    clients.push_back(std::make_unique<Client>(port_));
    ASSERT_TRUE(server.waitForConnectionCallback());
  }
}

TEST_F(TCPServerTest, max_clients_allowed)
{
  TestableTcpServer server(port_);
  server.start();
  server.setMaxClientsAllowed(1);

  // Test that only one client can connect
  Client client1(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());
  Client client2(port_);
  EXPECT_FALSE(server.waitForConnectionCallback());
}

TEST_F(TCPServerTest, message_transmission)
{
  TestableTcpServer server(port_);
  server.start();

  Client client(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  // Test that messages are transmitted corectly between client and server
  std::string message = "test message\n";
  client.send(message);

  EXPECT_TRUE(server.waitForMessageCallback());
  EXPECT_EQ(message, server.getReceivedMessage());

  size_t len = message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t written;

  ASSERT_TRUE(server.write(data, len, written));
  EXPECT_EQ(client.recv(), message);
}

TEST_F(TCPServerTest, client_connections)
{
  TestableTcpServer server(port_);
  server.start();

  std::string message = "text message\n";
  size_t len = message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t written;

  // Test that we can connect multiple clients
  Client client1(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  Client client2(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  Client client3(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  auto client_fds = server.getClientFDs();

  // Test that the correct clients are disconnected on the server side.
  client1.close();
  EXPECT_TRUE(server.waitForDisconnectionCallback());

  auto tcp_server = dynamic_cast<comm::TCPServer*>(&server);

  EXPECT_FALSE(tcp_server->write(client_fds[0], data, len, written));
  EXPECT_TRUE(tcp_server->write(client_fds[1], data, len, written));
  EXPECT_TRUE(tcp_server->write(client_fds[2], data, len, written));

  client2.close();
  EXPECT_TRUE(server.waitForDisconnectionCallback());
  EXPECT_FALSE(tcp_server->write(client_fds[0], data, len, written));
  EXPECT_FALSE(tcp_server->write(client_fds[1], data, len, written));
  EXPECT_TRUE(tcp_server->write(client_fds[2], data, len, written));

  client3.close();
  EXPECT_TRUE(server.waitForDisconnectionCallback());
  EXPECT_FALSE(tcp_server->write(client_fds[0], data, len, written));
  EXPECT_FALSE(tcp_server->write(client_fds[1], data, len, written));
  EXPECT_FALSE(tcp_server->write(client_fds[2], data, len, written));
}
TEST_F(TCPServerTest, check_address_already_in_use)
{
  comm::TCPServer blocking_server(12321);

  EXPECT_THROW(comm::TCPServer test_server(12321, 2, std::chrono::milliseconds(500)), std::system_error);
}

TEST_F(TCPServerTest, check_shutting_down_server_while_listening)
{
  auto server = std::make_unique<comm::TCPServer>(port_);
  server->start();

  // Use a client with a read timeout so we don't hang forever if the server doesn't shut down
  // properly.
  Client client(port_);
  timeval tv;
  tv.tv_sec = 10;
  tv.tv_usec = 0;  // 100 ms
  client.setReceiveTimeout(tv);

  // Start reading data with the client in a separate thread. This will have a blocking recv() call
  // that should be interrupted when the server is shut down. In that case the
  bool read_success = true;
  std::thread read_data_thread([&client, &read_success]() {
    while (read_success)
    {
      // As we aren't sending any data, this will block until the server is shut down and the
      // connection is closed. At that point, recv() should return an empty string, which we
      // interpret as a successful shutdown of the server.
      read_success = (client.recv() != "");
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  server.reset();

  if (read_data_thread.joinable())
  {
    read_data_thread.join();
  }
  EXPECT_FALSE(read_success);
  // If the read just would have timeouted, the client state would still be connected.
  EXPECT_EQ(client.getState(), comm::SocketState::Disconnected);
}

TEST_F(TCPServerTest, double_shutdown)
{
  TestableTcpServer server(port_);
  server.start();

  Client client(port_);
  EXPECT_TRUE(server.waitForConnectionCallback());

  EXPECT_NO_THROW(server.shutdown());
  EXPECT_NO_THROW(server.shutdown());
}

TEST_F(TCPServerTest, concurrent_writes_same_client)
{
  TestableTcpServer server(0);
  server.start();

  Client client(server.getPort());
  ASSERT_TRUE(server.waitForConnectionCallback());

  const std::string message = "test data\n";
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  const size_t len = message.size();

  constexpr int num_threads = 10;
  constexpr int writes_per_thread = 100;
  std::atomic<int> success_count{ 0 };
  std::vector<std::thread> writers;

  for (int i = 0; i < num_threads; ++i)
  {
    writers.emplace_back([&server, data, len, &success_count]() {
      for (int j = 0; j < writes_per_thread; ++j)
      {
        size_t written;
        if (server.write(data, len, written))
        {
          ++success_count;
        }
      }
    });
  }

  for (auto& t : writers)
  {
    t.join();
  }

  EXPECT_EQ(success_count.load(), num_threads * writes_per_thread);
}

TEST_F(TCPServerTest, write_during_client_disconnect)
{
  TestableTcpServer server(0);
  server.start();

  Client client(server.getPort());
  ASSERT_TRUE(server.waitForConnectionCallback());

  const std::string message = "test data\n";
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  const size_t len = message.size();

  std::atomic<bool> stop{ false };

  std::thread writer([&server, data, len, &stop]() {
    while (!stop.load())
    {
      size_t written;
      server.write(data, len, written);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  client.close();
  ASSERT_TRUE(server.waitForDisconnectionCallback());

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  stop.store(true);
  writer.join();
}

TEST_F(TCPServerTest, rapid_connect_disconnect_with_concurrent_writes)
{
  comm::TCPServer server(0);

  std::mutex fds_mutex;
  std::vector<socket_t> connected_fds;

  server.setConnectCallback([&](const socket_t fd) {
    std::lock_guard<std::mutex> lk(fds_mutex);
    connected_fds.push_back(fd);
  });
  server.setDisconnectCallback([&](const socket_t fd) {
    std::lock_guard<std::mutex> lk(fds_mutex);
    connected_fds.erase(std::remove(connected_fds.begin(), connected_fds.end(), fd), connected_fds.end());
  });
  server.start();

  const std::string message = "test data\n";
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  const size_t len = message.size();

  std::atomic<bool> stop{ false };

  std::thread writer([&]() {
    while (!stop.load())
    {
      std::vector<socket_t> snapshot;
      {
        std::lock_guard<std::mutex> lk(fds_mutex);
        snapshot = connected_fds;
      }
      for (const auto& fd : snapshot)
      {
        size_t written;
        server.write(fd, data, len, written);
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  constexpr int num_iterations = 50;
  for (int i = 0; i < num_iterations; ++i)
  {
    Client client(server.getPort());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  stop.store(true);
  writer.join();
}

TEST_F(TCPServerTest, concurrent_writes_multiple_clients)
{
  comm::TCPServer server(0);

  std::mutex fds_mutex;
  std::vector<socket_t> connected_fds;
  std::condition_variable all_connected_cv;
  constexpr size_t num_clients = 10;

  server.setConnectCallback([&](const socket_t fd) {
    std::lock_guard<std::mutex> lk(fds_mutex);
    connected_fds.push_back(fd);
    if (connected_fds.size() == num_clients)
    {
      all_connected_cv.notify_all();
    }
  });
  server.start();

  std::vector<std::unique_ptr<Client>> clients;
  for (size_t i = 0; i < num_clients; ++i)
  {
    clients.push_back(std::make_unique<Client>(server.getPort()));
  }

  {
    std::unique_lock<std::mutex> lk(fds_mutex);
    ASSERT_TRUE(
        all_connected_cv.wait_for(lk, std::chrono::seconds(5), [&]() { return connected_fds.size() == num_clients; }));
  }

  const std::string message = "test data\n";
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  const size_t len = message.size();

  constexpr int writes_per_thread = 100;
  std::atomic<int> total_successes{ 0 };

  std::vector<std::thread> writers;
  {
    std::lock_guard<std::mutex> lk(fds_mutex);
    for (const auto& fd : connected_fds)
    {
      writers.emplace_back([&server, fd, data, len, &total_successes]() {
        for (int j = 0; j < writes_per_thread; ++j)
        {
          size_t written;
          if (server.write(fd, data, len, written))
          {
            ++total_successes;
          }
        }
      });
    }
  }

  for (auto& t : writers)
  {
    t.join();
  }

  EXPECT_EQ(total_successes.load(), static_cast<int>(num_clients) * writes_per_thread);
}

TEST_F(TCPServerTest, shutdown_during_active_writes)
{
  TestableTcpServer server(0);
  server.start();

  Client client(server.getPort());
  ASSERT_TRUE(server.waitForConnectionCallback());

  const std::string message = "test data\n";
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  const size_t len = message.size();

  std::atomic<bool> stop{ false };

  std::thread writer([&server, data, len, &stop]() {
    while (!stop.load())
    {
      size_t written;
      server.write(data, len, written);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  server.shutdown();
  stop.store(true);
  writer.join();
}

// Verifies that the server receives data from many clients that all send simultaneously. This
// exercises the poll() revents loop across many client file descriptors and guards against
// missed read events when several sockets are readable at once.
TEST_F(TCPServerTest, receives_from_many_concurrent_clients)
{
  comm::TCPServer server(0);

  std::mutex mtx;
  std::condition_variable cv;
  std::atomic<int> message_count{ 0 };

  server.setMessageCallback([&](const socket_t, char*, int) {
    message_count.fetch_add(1);
    std::lock_guard<std::mutex> lk(mtx);
    cv.notify_all();
  });
  server.start();

#ifdef _WIN32
  // Windows allows a maximum of 64 sockets per process by default.
  constexpr int num_clients = 50;
#else
  constexpr int num_clients = 100;
#endif

  std::vector<std::unique_ptr<Client>> clients;
  for (int i = 0; i < num_clients; ++i)
  {
    clients.push_back(std::make_unique<Client>(server.getPort()));
  }

  // Every client sends a single message concurrently.
  std::vector<std::thread> senders;
  for (auto& client : clients)
  {
    senders.emplace_back([&client]() { client->send("ping\n"); });
  }
  for (auto& t : senders)
  {
    t.join();
  }

  // The server's poll() loop must observe activity on every client FD and deliver all messages.
  std::unique_lock<std::mutex> lk(mtx);
  EXPECT_TRUE(cv.wait_for(lk, std::chrono::seconds(5), [&]() { return message_count.load() >= num_clients; }));
  EXPECT_EQ(message_count.load(), num_clients);
}

#ifndef _WIN32
// Regression test for the FD_SETSIZE limitation of select(): a client whose accepted socket file
// descriptor number is >= FD_SETSIZE (1024) must still be serviced normally. This is the exact
// scenario that occurs when the hosting process (e.g. a JVM) holds many file descriptors. The old
// select()-based implementation rejected/crashed on such descriptors; poll() handles them.
TEST_F(TCPServerTest, services_client_with_high_fd_number)
{
  // Make sure we are allowed to open more than FD_SETSIZE descriptors; raise the soft limit if
  // needed and skip the test if the hard limit does not allow it.
  struct rlimit rl;
  ASSERT_EQ(getrlimit(RLIMIT_NOFILE, &rl), 0);
  const rlim_t needed = static_cast<rlim_t>(FD_SETSIZE) + 64;
  if (rl.rlim_cur < needed)
  {
    rl.rlim_cur = std::min<rlim_t>(needed, rl.rlim_max);
    if (setrlimit(RLIMIT_NOFILE, &rl) != 0 || rl.rlim_cur < needed)
    {
      GTEST_SKIP() << "Cannot raise RLIMIT_NOFILE above FD_SETSIZE; skipping high-fd test.";
    }
  }

  // Consume the low-numbered descriptors so that subsequently created sockets are assigned fd
  // numbers beyond FD_SETSIZE.
  std::vector<int> fd_hogs;
  while (true)
  {
    int fd = ::open("/dev/null", O_RDONLY);
    if (fd < 0)
    {
      break;
    }
    fd_hogs.push_back(fd);
    if (fd > static_cast<int>(FD_SETSIZE) + 8)
    {
      break;
    }
  }
  const bool pushed_past_limit = !fd_hogs.empty() && fd_hogs.back() > static_cast<int>(FD_SETSIZE);
  if (!pushed_past_limit)
  {
    for (int fd : fd_hogs)
    {
      ::close(fd);
    }
    GTEST_SKIP() << "Could not allocate descriptors beyond FD_SETSIZE; skipping.";
  }

  TestableTcpServer server(port_);
  server.start();

  Client client(port_);
  EXPECT_TRUE(server.waitForConnectionCallback(2000));

  // The server-side accepted client FD should exceed FD_SETSIZE -- the case that breaks select().
  auto client_fds = server.getClientFDs();
  ASSERT_FALSE(client_fds.empty());
  EXPECT_GT(client_fds.back(), static_cast<socket_t>(FD_SETSIZE));

  // Data must flow both ways on the high-numbered descriptor.
  const std::string message = "high fd message\n";
  client.send(message);
  EXPECT_TRUE(server.waitForMessageCallback(2000));
  EXPECT_EQ(server.getReceivedMessage(), message);

  size_t written;
  const auto* data = reinterpret_cast<const uint8_t*>(message.c_str());
  ASSERT_TRUE(server.write(data, message.size(), written));
  EXPECT_EQ(client.recv(), message);

  // Disconnect must also be detected on the high-numbered descriptor.
  client.close();
  EXPECT_TRUE(server.waitForDisconnectionCallback(2000));

  for (int fd : fd_hogs)
  {
    ::close(fd);
  }
}
#endif

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
