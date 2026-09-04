// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

// Losing the RTDE connection and getting it back is handled by RTDEClient::reconnect(), running on
// its own thread. Everything it does happens between the client and the RTDE server, so the fake
// server is enough to drive it and these tests need no robot.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include <ur_client_library/exceptions.h>
#include <ur_client_library/log.h>
#include <ur_client_library/rtde/rtde_client.h>

#include "fake_rtde_server.h"

using namespace urcl;

namespace
{
constexpr int g_FAKE_RTDE_PORT = 60007;
constexpr double g_RTDE_FREQUENCY = 100.0;

const std::vector<std::string> g_OUTPUT_RECIPE{ "timestamp", "actual_q", "target_speed_fraction", "runtime_state" };
const std::vector<std::string> g_INPUT_RECIPE{ "speed_slider_mask", "speed_slider_fraction" };

// How long to allow for a state transition that depends on the reconnect thread's retry timing.
constexpr std::chrono::seconds g_STATE_CHANGE_TIMEOUT{ 10 };
}  // namespace

class RTDEClientReconnectTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    client_.reset();
    server_.reset();
  }

  /*!
   * \brief Starts a fake server that the client's bootup check will accept straight away.
   *
   * RTDEClient::isRobotBooted() reads data for a second when the reported uptime is below 40
   * seconds, which would only slow these tests down.
   */
  void startServer()
  {
    server_ = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
    server_->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
  }

  void makeClient()
  {
    client_ = std::make_unique<rtde_interface::RTDEClient>("localhost", notifier_, g_OUTPUT_RECIPE, g_INPUT_RECIPE,
                                                           g_RTDE_FREQUENCY, false, g_FAKE_RTDE_PORT);
  }

  /*!
   * \brief Waits for the client to reach \p expected, so the tests don't depend on how long a
   * reconnect attempt happens to take.
   */
  bool waitForState(const rtde_interface::ClientState expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + g_STATE_CHANGE_TIMEOUT;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (client_->getClientState() == expected)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return client_->getClientState() == expected;
  }

  comm::INotifier notifier_;
  std::unique_ptr<RTDEServer> server_;
  std::unique_ptr<rtde_interface::RTDEClient> client_;
};

// Dropping the server has to take the client down, and bringing it back has to get the client all
// the way to RUNNING again without the application doing anything.
TEST_F(RTDEClientReconnectTest, reconnects_when_the_server_comes_back_during_background_read)
{
  startServer();
  makeClient();
  ASSERT_TRUE(client_->init(0, std::chrono::milliseconds(123), 3, std::chrono::milliseconds(100)));
  client_->start();

  // A reader in the background, because that is what arms the reconnect callback and what an
  // application would be doing when the connection drops.
  std::atomic<bool> keep_running{ true };
  std::thread data_consumer([this, &keep_running]() {
    rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
    while (keep_running)
    {
      if (!client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  server_.reset();
  EXPECT_TRUE(waitForState(rtde_interface::ClientState::UNINITIALIZED)) << "the client did not notice the lost server";

  startServer();
  EXPECT_TRUE(waitForState(rtde_interface::ClientState::RUNNING)) << "the client did not reconnect";

  keep_running = false;
  data_consumer.join();

  // Data has to actually flow again, not just the state having been restored
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  EXPECT_TRUE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)));
}

// The same recovery, but for a client reading synchronously. reconnect() restores whichever read
// mode was in use, so both need covering.
TEST_F(RTDEClientReconnectTest, reconnects_when_the_server_comes_back_during_blocking_read)
{
  startServer();
  makeClient();
  ASSERT_TRUE(client_->init(0, std::chrono::milliseconds(123), 3, std::chrono::milliseconds(100)));
  client_->start(false);

  std::atomic<bool> keep_running{ true };
  std::thread data_consumer([this, &keep_running]() {
    auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
    while (keep_running)
    {
      if (!client_->getDataPackageBlocking(data_pkg))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  server_.reset();
  EXPECT_TRUE(waitForState(rtde_interface::ClientState::UNINITIALIZED)) << "the client did not notice the lost server";

  startServer();
  EXPECT_TRUE(waitForState(rtde_interface::ClientState::RUNNING)) << "the client did not reconnect";

  keep_running = false;
  data_consumer.join();

  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  EXPECT_TRUE(client_->getDataPackageBlocking(data_pkg));
}

// A server that stays connected but stops talking leaves the read path waiting, which the
// destructor has to be able to tear down.
TEST_F(RTDEClientReconnectTest, destroying_the_client_while_the_server_is_silent)
{
  startServer();
  makeClient();
  ASSERT_TRUE(client_->init());
  client_->start();

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)));
  double timestamp = 0.0;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  server_->stopSendingDataPackages();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  client_.reset();
}

// Regression test for the bug where ~RTDEClient() could block indefinitely when the reconnect
// thread was stuck inside TCPSocket::setup(). Fixed by: (1) calling stream_.disconnect() (followed
// by RTDEClient::disconnect()) before joining reconnecting_thread_ in ~RTDEClient(), and (2) making
// TCPSocket::setup() abort on the deliberate-stop state, both during the (non-blocking) connect
// attempt and during the between-attempt wait.
//
// See also TCPSocketTest.setup_interruptible_by_close and
// TCPSocketTest.setup_interruptible_during_blocking_connect in test_tcp_socket.cpp for lower-level
// unit tests of the same fix.
TEST_F(RTDEClientReconnectTest, destructor_not_blocked_by_stuck_reconnect_thread)
{
  // Large enough that the blocking window is clearly observable if the fix is absent: the 5 s sleep
  // would exceed the 2 s assertion threshold below.
  const std::chrono::milliseconds large_reconnect_timeout(5000);

  startServer();

  // Retry the handshake a few times, so a fake server response arriving just after the socket read
  // timeout doesn't fail the test before it has tested anything.
  bool initialized = false;
  for (int attempt = 0; attempt < 10 && !initialized; ++attempt)
  {
    makeClient();
    try
    {
      // max_connection_attempts=0 (unlimited): TCPSocket::setup() sleeps large_reconnect_timeout
      // between every failed connect attempt once the server is gone. The short initialization
      // timeout keeps the retries here quick.
      client_->init(0, large_reconnect_timeout, 1, std::chrono::milliseconds(50));
      initialized = true;
    }
    catch (const UrException&)
    {
      // Fall through and start over from a clean client
    }
  }
  if (!initialized)
  {
    GTEST_SKIP() << "Could not initialize RTDEClient with the fake server after 10 attempts; "
                    "this test requires a reliably responding RTDE server. "
                    "The TCPSocket-level regression test (TCPSocketTest.setup_interruptible_by_close) "
                    "verifies the underlying fix without a robot.";
  }

  // start(true) arms the reconnect callback via the background read thread.
  client_->start(true);

  // Drop the server: the background read thread detects the connection loss and calls
  // reconnectCallback(), which launches reconnecting_thread_. That thread enters
  // setupCommunication() -> TCPSocket::setup() and begins sleeping large_reconnect_timeout between
  // retry attempts.
  server_.reset();

  // Give the reconnect thread time to reach the wait inside TCPSocket::setup().
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // The destructor must return quickly: disconnect() aborts setup()'s connect/wait, so the join
  // completes in well under 2 s. Without the fix this would block for >= large_reconnect_timeout
  // (5 s), or forever with unlimited attempts. Run it on a worker with a watchdog so a regression
  // fails fast with a clear message instead of hanging the test binary.
  std::packaged_task<void()> teardown([this]() { client_.reset(); });
  auto teardown_future = teardown.get_future();
  std::thread teardown_thread(std::move(teardown));

  const auto t0 = std::chrono::steady_clock::now();
  if (teardown_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout)
  {
    teardown_thread.detach();
    FAIL() << "~RTDEClient() did not return within 5 s — reconnect thread was not aborted by disconnect()";
  }
  teardown_thread.join();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::seconds(2))
      << "RTDEClient destructor blocked for " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
      << " ms — reconnect thread was not aborted by disconnect()";
}

// A server that accepts the TCP connection but never finishes the handshake must not be retried
// forever. After max_initialization_attempts_ the client gives up and stays uninitialized.
TEST_F(RTDEClientReconnectTest, reconnect_gives_up_when_the_handshake_keeps_failing)
{
  startServer();
  makeClient();
  ASSERT_TRUE(client_->init(0, std::chrono::milliseconds(50), 2, std::chrono::milliseconds(50)));
  client_->start();

  std::atomic<bool> keep_running{ true };
  std::thread data_consumer([this, &keep_running]() {
    rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
    while (keep_running)
    {
      if (!client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  server_.reset();
  EXPECT_TRUE(waitForState(rtde_interface::ClientState::UNINITIALIZED));

  // A listener is back, so reconnect counts each failed handshake instead of waiting on connect.
  startServer();
  server_->setHighestAcceptedProtocolVersion(0);

  // Two failed handshakes plus the short sleep between them, then give up.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  keep_running = false;
  data_consumer.join();

  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  // During a retry the client is UNINITIALIZED between attempts, so a later state check alone
  // cannot prove it stopped. Another protocol-version request would mean it is still trying.
  const auto requests_after_give_up = server_->requestedProtocolVersions().size();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  EXPECT_EQ(server_->requestedProtocolVersions().size(), requests_after_give_up) << "the client kept retrying after "
                                                                                    "exhausting its initialization "
                                                                                    "attempts";
}
