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
#include <cstddef>
#include <thread>
#include "test_utils.h"

#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/comm/tcp_server.h>
#include "ur_client_library/types.h"

#ifdef __linux__
#  include <dirent.h>
#endif

using namespace urcl;

#ifdef __linux__
namespace
{
// Counts the process' currently open file descriptors by listing /proc/self/fd. The transient
// descriptor opened for the directory itself is present in every measurement, so it cancels out
// when comparing a before/after count.
size_t countOpenFds()
{
  DIR* dir = ::opendir("/proc/self/fd");
  EXPECT_NE(dir, nullptr) << "Could not open /proc/self/fd";
  if (dir == nullptr)
  {
    return 0;
  }
  size_t count = 0;
  while (struct dirent* entry = ::readdir(dir))
  {
    if (std::string(entry->d_name) == "." || std::string(entry->d_name) == "..")
    {
      continue;
    }
    ++count;
  }
  ::closedir(dir);
  return count;
}
}  // namespace
#endif  // __linux__

class TCPSocketTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    server_.reset(new TestableTcpServer(60001));
    server_->start();

    client_.reset(new Client(60001));
  }

  void TearDown()
  {
    server_.reset();
    client_.reset();
  }

  class Client : public comm::TCPSocket
  {
  public:
    Client(int port, const std::string& ip = "127.0.0.1")
    {
      port_ = port;
      ip_ = ip;
    }

    bool connect(const size_t max_num_tries = 0,
                 const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10))
    {
      return TCPSocket::connect(ip_, port_, max_num_tries, reconnection_time);
    }

    bool reconnect(const size_t max_num_tries = 0,
                   const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10))
    {
      return TCPSocket::reconnect(ip_, port_, max_num_tries, reconnection_time);
    }

    bool setTargetStateUnlessStopRequested(comm::SocketState desired)
    {
      URCL_LOG_INFO("Setting target state to %s", socketStateToString(desired).c_str());
      return TCPSocket::setTargetStateUnlessStopRequested(desired);
    }

    bool isStopRequested() const
    {
      return TCPSocket::isStopRequested();
    }

    void printState()
    {
      comm::SocketState state = getState();
      std::string state_str = socketStateToString(state);
      std::cout << "Client state: " << state_str << ", stop_requested: " << std::boolalpha << isStopRequested()
                << std::endl;
    }

    void disconnect()
    {
      TCPSocket::close();
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
      TCPSocket::connect(ip, port, max_num_tries, reconnection_time);
      done_setting_up_client_ = true;
    }
  };

  std::unique_ptr<TestableTcpServer> server_;
  std::unique_ptr<Client> client_;
};

TEST_F(TCPSocketTest, socket_state)
{
  // Client state should be invalid
  comm::SocketState expected_state = comm::SocketState::Invalid;
  comm::SocketState actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));

  // Client state should be connected after setup
  client_->connect();
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

  // Client state should be Connecting while it keeps retrying and the server is not available
  comm::SocketState expected_state = comm::SocketState::Connecting;
  comm::SocketState actual_state = client_->getState();

  EXPECT_EQ(toUnderlying(expected_state), toUnderlying(actual_state));

  server_.reset(new TestableTcpServer(60001));
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

  client_->connect();
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
  client_->connect();

  std::string message = "test message";
  const uint8_t* data = reinterpret_cast<const uint8_t*>(message.c_str());
  size_t len = message.size();
  size_t written;
  client_->write(data, len, written);

  EXPECT_TRUE(server_->waitForMessageCallback());
  EXPECT_EQ(message, server_->getReceivedMessage());
}

TEST_F(TCPSocketTest, read_on_connected_socket)
{
  client_->connect();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(server_->waitForConnectionCallback());

  std::string send_message = "test message";
  size_t len = send_message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(send_message.c_str());
  size_t written;
  server_->write(data, len, written);

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
  socket_t expected_fd = INVALID_SOCKET;
  socket_t actual_fd = client_->getSocketFD();

  EXPECT_EQ(expected_fd, actual_fd);

  client_->connect();
  actual_fd = client_->getSocketFD();

  EXPECT_NE(expected_fd, actual_fd);

  client_->close();
  actual_fd = client_->getSocketFD();

  EXPECT_EQ(expected_fd, actual_fd);
}

TEST_F(TCPSocketTest, receive_timeout)
{
  client_->connect();

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
  client_->connect();

  EXPECT_FALSE(client_->connect());
}

TEST_F(TCPSocketTest, connect_non_running_robot)
{
  Client client(12321, "127.0.0.1");
  auto start = std::chrono::system_clock::now();
  EXPECT_FALSE(client.connect(2, std::chrono::milliseconds(500)));
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  // This is only a rough estimate, obviously
  EXPECT_LT(elapsed, std::chrono::milliseconds(7500));
}

// Regression test for the bug where TCPSocket::setup() could block the caller
// indefinitely when the wait between retry attempts was not interruptible.
//
// setup() is interrupted by disconnect(), which moves the socket into the deliberate-stop
// state (Disconnecting/Disconnected) and closes it. setup() never overwrites that state and
// aborts as soon as it observes it, so a teardown signal cannot be lost by setup()'s internal
// state updates. This is the path used by ~RTDEClient()/~PrimaryClient() before joining their
// reconnect threads.
TEST_F(TCPSocketTest, setup_interruptible_by_close)
{
  // Use a port with no listener so every connect attempt fails immediately,
  // sending setup() into the between-attempt wait.
  const int unused_port = 12322;
  const std::chrono::milliseconds large_reconnect_timeout(5000);

  Client client(unused_port, "127.0.0.1");

  // Run setup() with unlimited retries in a background thread.
  std::thread setup_thread([&client, &large_reconnect_timeout]() {
    // max_num_tries=0 (unlimited) → setup() waits large_reconnect_timeout after
    // every failed connect attempt and never exits on its own.
    client.connect(0, large_reconnect_timeout);
  });

  // Give the thread time to reach the between-attempt wait inside setup().
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // disconnect() moves the socket to the deliberate-stop state; the sliced wait in setup()
  // detects it within 100 ms and setup() returns, allowing the thread to finish.
  const auto t0 = std::chrono::steady_clock::now();
  client.disconnect();

  setup_thread.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  // Without the fix, elapsed would be >= large_reconnect_timeout (5 s).
  EXPECT_LT(elapsed, std::chrono::seconds(2)) << "TCPSocket::setup() was not interrupted by disconnect() within 2 s; "
                                                 "the between-attempt wait is not interruptible";
}

// Regression test for issue #368: a reconnect thread blocked inside connect() to a
// genuinely unreachable host (no SYN-ACK, no RST) must still be abortable. The
// previous fix only made the between-attempt *sleep* interruptible, not the connect
// itself, so this case could block for the full OS connect timeout. setup() now uses
// a non-blocking connect polled in short slices, so disconnect() aborts it promptly.
TEST_F(TCPSocketTest, setup_interruptible_during_blocking_connect)
{
  // 10.255.255.1 is in a private range and is (almost) never routable, so connect()
  // hangs in SYN retransmit rather than failing fast like a refused localhost port.
  const int unused_port = 12323;
  const std::chrono::milliseconds large_reconnect_timeout(5000);

  Client client(unused_port, "10.255.255.1");

  std::thread setup_thread([&client, &large_reconnect_timeout]() { client.connect(0, large_reconnect_timeout); });

  // Give the thread time to enter the (blocking) connect attempt.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto t0 = std::chrono::steady_clock::now();
  client.disconnect();

  setup_thread.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::seconds(2)) << "TCPSocket::setup() was not interrupted while blocked in connect() "
                                                 "within 2 s; "
                                                 "the connect attempt is not interruptible";
}

TEST_F(TCPSocketTest, test_deprecated_reconnection_time_interface)
{
  URCL_SILENCE_DEPRECATED_BEGIN
  client_->setReconnectionTime(std::chrono::milliseconds(100));
  URCL_SILENCE_DEPRECATED_END
  EXPECT_TRUE(client_->connect(2));
}

TEST_F(TCPSocketTest, test_read_on_socket_abruptly_closed)
{
  client_->connect();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(server_->waitForConnectionCallback());

  std::string send_message = "test message";
  size_t len = send_message.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(send_message.c_str());
  size_t written;
  server_->write(data, len, written);

  // Simulate socket failure
  ur_close(client_->getSocketFD());

  char characters;
  size_t read_chars = 0;
  EXPECT_FALSE(client_->read((uint8_t*)&characters, 1, read_chars));
  EXPECT_EQ(client_->getState(), comm::SocketState::LostConnection);
}

TEST_F(TCPSocketTest, test_socket_client_lifecycle)
{
  // After startup a client should be in state Invalid
  EXPECT_EQ(client_->getState(), comm::SocketState::Invalid);
  client_->printState();

  client_->connect();
  EXPECT_EQ(client_->getState(), comm::SocketState::Connected);
  client_->printState();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(server_->waitForConnectionCallback());

  std::atomic<int> read_count(0);
  auto read_fun = [this, &read_count]() {
    bool read_success = true;
    uint8_t buffer[64];
    size_t read_chars = 0;
    while (read_success)
    {
      std::cout << "Reading from socket..." << std::endl;
      read_success = client_->read(buffer, 64, read_chars);
      if (read_success)
      {
        std::cout << "Read " << read_chars << " characters." << std::endl;
        read_count.store(read_count.load() + 1);
      }
    }
  };

  // Scenario 1: Client is connected, server is shutdown, client should go into LostConnection
  // state. Afterwards we should be able to restart the server and reconnect the client.
  {
    std::thread read_thread(read_fun);

    std::string send_message = "test message";
    size_t len = send_message.size();
    const uint8_t* data = reinterpret_cast<const uint8_t*>(send_message.c_str());
    size_t written;
    server_->write(data, len, written);

    ASSERT_NO_THROW(waitFor([&read_count]() { return read_count.load() > 0; }, std::chrono::seconds(1)));
    std::cout << "Read count: " << read_count.load() << std::endl;

    EXPECT_EQ(client_->getState(), comm::SocketState::Connected);
    client_->printState();

    std::cout << "Shutting down server..." << std::endl;
    server_->shutdown();

    read_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::LostConnection);
    client_->printState();

    std::cout << "Restarting server..." << std::endl;
    server_.reset(new TestableTcpServer(60001));
    server_->start();

    std::cout << "Reconnecting client..." << std::endl;
    ASSERT_TRUE(client_->connect());
    client_->printState();
  }

  // Scenario 2: Deliberate disconnect() while a read is in progress should abort the read and put
  // the client into Disconnected state. We should be able to reconnect the client afterwards.
  {
    std::thread read_thread(read_fun);
    client_->disconnect();
    read_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::Closed);
    client_->printState();

    client_->connect();
    EXPECT_EQ(client_->getState(), comm::SocketState::Connected)
        << "Expected state" << comm::socketStateToString(comm::SocketState::Connected)
        << ". Got state : " << socketStateToString(client_->getState());
    client_->printState();
  }

  // Scenario 3: Server shuts down, client disconnects. Connecting the client will go into a
  // connection attempt loop. At some point we start the server again and the client should reconnect successfully.
  // This simulates a client application with a auto-reconnect on read-failure.
  {
    std::thread read_thread(read_fun);

    std::cout << "Shutting down server..." << std::endl;
    server_->shutdown();

    read_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::LostConnection);
    client_->printState();

    std::cout << "Starting connection attempt loop..." << std::endl;
    std::thread connect_thread([this]() {
      bool result = client_->reconnect(0, std::chrono::milliseconds(100));
      EXPECT_TRUE(result) << "Client failed to reconnect after server restart";
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    server_.reset(new TestableTcpServer(60001));
    server_->start();

    connect_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::Connected)
        << "Expected state" << comm::socketStateToString(comm::SocketState::Connected)
        << ". Got state: " << socketStateToString(client_->getState());
    client_->printState();
  }

  // Scenario 4: Same as scenario 3, but while reconnecting we do not start the server but
  // disconnect the client. The client should abort the connection attempt loop and go into Disconnected state.
  {
    std::thread read_thread(read_fun);

    std::cout << "Shutting down server..." << std::endl;
    server_->shutdown();

    read_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::LostConnection)
        << "Client state after reconnect: " << socketStateToString(client_->getState());
    client_->printState();

    std::cout << "Starting connection attempt loop..." << std::endl;
    std::thread connect_thread([this]() {
      bool result = client_->reconnect(0, std::chrono::milliseconds(100));
      EXPECT_FALSE(result) << "Client should have aborted the connection attempt loop";
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    client_->disconnect();

    connect_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::Closed)
        << "Client state after reconnect: " << socketStateToString(client_->getState());
    client_->printState();
  }

  // Scenario 5: We attempt to connect to a non-reachable server. Disconnecting the client should abort the connection
  // attempt loop and put the client into Disconnected state.
  {
    server_->shutdown();
    std::thread connect_thread([this]() {
      bool result = client_->connect(0, std::chrono::milliseconds(100));
      EXPECT_FALSE(result) << "Client should have aborted the connection attempt loop";
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    client_->disconnect();

    connect_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::Closed)
        << "Client state after reconnect: " << socketStateToString(client_->getState());
    client_->printState();
  }

  // Scenario 6: Server shuts down, connection lost. Then, client gets disconnected. Server starts
  // again, reconnect should fail, since it was deliberately disconnected. Connect should work.
  {
    server_.reset(new TestableTcpServer(60001));
    server_->start();
    client_->connect();
    std::thread read_thread(read_fun);

    std::cout << "Shutting down server..." << std::endl;
    server_->shutdown();

    read_thread.join();
    EXPECT_EQ(client_->getState(), comm::SocketState::LostConnection);
    client_->printState();

    std::cout << "Disconnecting client..." << std::endl;
    client_->disconnect();
    EXPECT_EQ(client_->getState(), comm::SocketState::Closed);
    client_->printState();

    std::cout << "Starting server..." << std::endl;
    server_.reset(new TestableTcpServer(60001));
    server_->start();

    std::cout << "Attempting to reconnect client..." << std::endl;
    bool result = client_->reconnect(0, std::chrono::milliseconds(100));
    EXPECT_FALSE(result) << "Client should not have reconnected after deliberate disconnect()";
    EXPECT_EQ(client_->getState(), comm::SocketState::Closed);
    client_->printState();

    std::cout << "Attempting to connect client..." << std::endl;
    result = client_->connect(0, std::chrono::milliseconds(100));
    EXPECT_TRUE(result) << "Client should have connected after deliberate disconnect()";
    EXPECT_EQ(client_->getState(), comm::SocketState::Connected);
    client_->printState();
  }
}

TEST_F(TCPSocketTest, test_set_target_state_unless_stop_requested)
{
  ASSERT_EQ(client_->getState(), comm::SocketState::Invalid);
  // Requesting to go to current state should return true
  ASSERT_TRUE(client_->setTargetStateUnlessStopRequested(comm::SocketState::Invalid));
  // We should be able to set the target state to Connected, since we are not in a deliberate
  // disconnect() state
  ASSERT_TRUE(client_->setTargetStateUnlessStopRequested(comm::SocketState::Connected));
  // This should not have modified the state of the client, since we are not connected yet
  ASSERT_EQ(client_->getState(), comm::SocketState::Invalid);
  client_->connect();

  // Make sure the client has connected to the server, before writing to the client
  EXPECT_TRUE(server_->waitForConnectionCallback());

  ASSERT_EQ(client_->getState(), comm::SocketState::Connected);
  // Requesting to go to current state should return true
  ASSERT_TRUE(client_->setTargetStateUnlessStopRequested(comm::SocketState::Connected));

  client_->disconnect();
  ASSERT_EQ(client_->getState(), comm::SocketState::Closed);
  ASSERT_TRUE(client_->isStopRequested());
  ASSERT_FALSE(client_->setTargetStateUnlessStopRequested(comm::SocketState::Connected));
  client_->printState();
}

// Regression test for a file-descriptor leak: each connection attempt creates a new socket
// descriptor, and a failed attempt must free it. Repeatedly failing to connect must therefore not
// grow the number of open descriptors.
TEST_F(TCPSocketTest, no_fd_leak_on_repeated_failed_connects)
{
#ifndef __linux__
  GTEST_SKIP() << "Counting open file descriptors via /proc/self/fd is Linux-only";
#else
  // Port with no listener so every attempt fails quickly.
  const int dead_port = 12321;
  // A little slack absorbs unrelated descriptors (e.g. logging, DNS caches) that may be opened
  // lazily on the first attempts; a real leak grows linearly with the iteration count and blows
  // well past this.
  const size_t slack = 4;

  // Warm up once so any one-off descriptor allocation happens before we take the baseline.
  {
    Client warmup(dead_port, "127.0.0.1");
    EXPECT_FALSE(warmup.connect(2, std::chrono::milliseconds(20)));
  }

  const size_t baseline = countOpenFds();
  for (int i = 0; i < 50; ++i)
  {
    Client client(dead_port, "127.0.0.1");
    EXPECT_FALSE(client.connect(2, std::chrono::milliseconds(20)));
  }
  const size_t after = countOpenFds();

  EXPECT_LE(after, baseline + slack) << "Open file descriptors grew from " << baseline << " to " << after
                                     << " over 50 failed connection attempts; a descriptor is leaking";
#endif  // __linux__
}

// Regression test targeting the retry loop specifically: with unlimited retries the socket keeps
// creating new descriptors between attempts. Letting it retry several times and then disconnecting
// must not leave leaked descriptors behind.
TEST_F(TCPSocketTest, no_fd_leak_in_retry_loop)
{
#ifndef __linux__
  GTEST_SKIP() << "Counting open file descriptors via /proc/self/fd is Linux-only";
#else
  const int dead_port = 12321;
  const size_t slack = 4;

  const size_t baseline = countOpenFds();

  Client client(dead_port, "127.0.0.1");
  // Unlimited retries with a short back-off, so the loop runs many iterations while we wait.
  std::thread connect_thread([&client]() { client.connect(0, std::chrono::milliseconds(50)); });

  // Give the retry loop time to run through a good number of failed attempts.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  client.disconnect();
  connect_thread.join();

  const size_t after = countOpenFds();
  EXPECT_LE(after, baseline + slack) << "Open file descriptors grew from " << baseline << " to " << after
                                     << " while retrying to connect; the retry loop is leaking descriptors";
#endif  // __linux__
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
