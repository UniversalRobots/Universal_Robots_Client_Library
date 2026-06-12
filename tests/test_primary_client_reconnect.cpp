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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include <ur_client_library/primary/primary_client.h>

#include "fake_primary_server.h"

using namespace urcl;

// Regression test for ~PrimaryClient() blocking indefinitely when the pipeline's
// producer thread is stuck in its reconnect loop at teardown time.
//
// This is the PrimaryClient counterpart of
// RTDEClientTest.destructor_not_blocked_by_stuck_reconnect_thread (test_rtde_client.cpp).
//
// Root cause: when the robot drops the primary connection, TCPSocket::read()
// returns false and leaves the socket in SocketState::Disconnected. URProducer's
// tryGetImpl() then enters its reconnect path: it sleeps an (exponentially
// growing) backoff and calls stream_.connect(), which retries with no upper
// bound (max_num_tries == 0), sleeping reconnection_time between attempts. If
// ~PrimaryClient() simply called pipeline_->stop() (which joins the producer
// thread) without first closing the stream, the join would block for the full
// reconnect duration — effectively forever for an unreachable robot.
//
// Fix (two parts):
//   1. ~PrimaryClient()/PrimaryClient::stop() close the stream BEFORE joining the
//      pipeline. stream_.close() sets SocketState::Closed.
//   2. URProducer's reconnect backoff sleeps in 100 ms slices and bails out as
//      soon as the stream is closed (or the producer is stopped), and
//      TCPSocket::setup()'s between-attempt sleep is likewise interruptible by
//      SocketState::Closed.
// Together these wake the producer within ~100 ms of the destructor closing the
// stream, so the join — and therefore the destructor — returns promptly.
//
// Unlike test_primary_client.cpp's robot-dependent fixtures, this test uses the
// in-process FakePrimaryServer, so it runs in the normal (non-INTEGRATION_TESTS)
// build and needs no robot.
TEST(PrimaryClientReconnectTest, destructor_not_blocked_by_stuck_reconnect_thread)
{
  comm::INotifier notifier;

  auto server = std::make_unique<FakePrimaryServer>(primary_interface::UR_PRIMARY_PORT);
  auto client = std::make_unique<primary_interface::PrimaryClient>("127.0.0.1", notifier);

  // Unlimited reconnect attempts with a large reconnection time: if the fix is
  // absent, the producer's reconnect path keeps the destructor blocked.
  const std::chrono::milliseconds large_reconnect_timeout(5000);
  ASSERT_NO_THROW(client->start(/*max_num_tries=*/0, large_reconnect_timeout));
  ASSERT_TRUE(server->waitForClient()) << "PrimaryClient never connected to the fake server";

  // Drop the server. The producer's read() fails, the socket transitions to
  // SocketState::Disconnected, and the producer enters its reconnect loop.
  server.reset();

  // Give the producer time to detect the drop and reach its reconnect sleep
  // (initial backoff is 1 s, after which it sleeps inside TCPSocket::setup()).
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  // The destructor must return quickly: it closes the stream
  // (SocketState::Closed), waking the producer, so the pipeline join completes in
  // well under 2 s. Without the fix this blocks for at least the reconnect
  // timeout (and indefinitely with unlimited retries against a dead port).
  const auto t0 = std::chrono::steady_clock::now();
  client.reset();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::seconds(2))
      << "~PrimaryClient() blocked for " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()
      << " ms — the producer reconnect thread was not woken by the stream close";
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
