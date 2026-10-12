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
#include <stdexcept>
#include <string>
#include <vector>

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/exceptions.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/dashboard_client_implementation_g5.h>
#include <ur_client_library/ur/ur_driver.h>

#include "test_utils.h"

using namespace urcl;

namespace
{
const std::string LOCALHOST = "127.0.0.1";
const std::chrono::milliseconds CONNECT_TIMEOUT(500);
const std::chrono::milliseconds RECONNECTION_TIME(100);

const std::chrono::seconds MAX_SETUP_DURATION(3);

const std::vector<std::string> OUTPUT_RECIPE = { "timestamp" };
const std::vector<std::string> INPUT_RECIPE = {};

const char* const SKIP_MESSAGE = "This operating system refuses connection requests to a full accept queue";
}  // namespace

TEST(ClientConnectTimeoutTest, rtde_client_connect_timeout_is_disabled_by_default)
{
  comm::INotifier notifier;
  rtde_interface::RTDEClient client(LOCALHOST, notifier, OUTPUT_RECIPE, INPUT_RECIPE);
  EXPECT_EQ(client.getConnectTimeout(), std::chrono::milliseconds::zero());
}

TEST(ClientConnectTimeoutTest, rtde_client_set_connect_timeout)
{
  comm::INotifier notifier;
  rtde_interface::RTDEClient client(LOCALHOST, notifier, OUTPUT_RECIPE, INPUT_RECIPE);
  client.setConnectTimeout(CONNECT_TIMEOUT);
  EXPECT_EQ(client.getConnectTimeout(), CONNECT_TIMEOUT);
  EXPECT_THROW(client.setConnectTimeout(std::chrono::milliseconds(-1)), std::invalid_argument);
  EXPECT_EQ(client.getConnectTimeout(), CONNECT_TIMEOUT);
}

TEST(ClientConnectTimeoutTest, rtde_client_init_honors_connect_timeout)
{
  UnresponsiveServer robot;
  if (!robot.isUnresponsive())
  {
    GTEST_SKIP() << SKIP_MESSAGE;
  }
  comm::INotifier notifier;
  rtde_interface::RTDEClient client(LOCALHOST, notifier, OUTPUT_RECIPE, INPUT_RECIPE, 0.0, false, robot.getPort());
  client.setConnectTimeout(CONNECT_TIMEOUT);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(client.init(1, RECONNECTION_TIME, 1), UrException);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, CONNECT_TIMEOUT);
  EXPECT_LT(elapsed, MAX_SETUP_DURATION);
}

TEST(ClientConnectTimeoutTest, primary_client_connect_timeout_is_disabled_by_default)
{
  comm::INotifier notifier;
  primary_interface::PrimaryClient client(LOCALHOST, notifier);
  EXPECT_EQ(client.getConnectTimeout(), std::chrono::milliseconds::zero());
}

TEST(ClientConnectTimeoutTest, primary_client_set_connect_timeout)
{
  comm::INotifier notifier;
  primary_interface::PrimaryClient client(LOCALHOST, notifier);
  client.setConnectTimeout(CONNECT_TIMEOUT);
  EXPECT_EQ(client.getConnectTimeout(), CONNECT_TIMEOUT);
  EXPECT_THROW(client.setConnectTimeout(std::chrono::milliseconds(-1)), std::invalid_argument);
  EXPECT_EQ(client.getConnectTimeout(), CONNECT_TIMEOUT);
}

TEST(ClientConnectTimeoutTest, primary_client_start_honors_connect_timeout)
{
  UnresponsiveServer robot;
  if (!robot.isUnresponsive())
  {
    GTEST_SKIP() << SKIP_MESSAGE;
  }
  comm::INotifier notifier;
  primary_interface::PrimaryClient client(LOCALHOST, notifier, robot.getPort());
  client.setConnectTimeout(CONNECT_TIMEOUT);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(client.start(1, RECONNECTION_TIME), UrException);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, CONNECT_TIMEOUT);
  EXPECT_LT(elapsed, MAX_SETUP_DURATION);
}

TEST(ClientConnectTimeoutTest, dashboard_client_g5_connect_timeout_is_disabled_by_default)
{
  DashboardClient client(LOCALHOST, DashboardClient::ClientPolicy::G5);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), std::chrono::milliseconds::zero());
}

TEST(ClientConnectTimeoutTest, dashboard_client_g5_set_connect_timeout)
{
  DashboardClient client(LOCALHOST, DashboardClient::ClientPolicy::G5);
  client.setConnectTimeout(CONNECT_TIMEOUT);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), CONNECT_TIMEOUT);
  EXPECT_THROW(client.setConnectTimeout(std::chrono::milliseconds(-1)), std::invalid_argument);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), CONNECT_TIMEOUT);
}

TEST(ClientConnectTimeoutTest, dashboard_client_g5_connect_honors_connect_timeout)
{
  UnresponsiveServer robot(DashboardClientImplG5::DASHBOARD_SERVER_PORT);
  if (!robot.isUnresponsive())
  {
    GTEST_SKIP() << SKIP_MESSAGE;
  }
  DashboardClient client(LOCALHOST, DashboardClient::ClientPolicy::G5);
  client.setConnectTimeout(CONNECT_TIMEOUT);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.connect(1, RECONNECTION_TIME));
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, CONNECT_TIMEOUT);
  EXPECT_LT(elapsed, MAX_SETUP_DURATION);
}

TEST(ClientConnectTimeoutTest, dashboard_client_x_connect_timeout_defaults_to_five_seconds)
{
  DashboardClient client(LOCALHOST, DashboardClient::ClientPolicy::POLYSCOPE_X);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), std::chrono::seconds(5));
}

TEST(ClientConnectTimeoutTest, dashboard_client_x_set_connect_timeout)
{
  DashboardClient client(LOCALHOST, DashboardClient::ClientPolicy::POLYSCOPE_X);
  client.setConnectTimeout(CONNECT_TIMEOUT);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), CONNECT_TIMEOUT);
  EXPECT_THROW(client.setConnectTimeout(std::chrono::milliseconds(-1)), std::invalid_argument);
  EXPECT_EQ(client.getConfiguredConnectTimeout(), CONNECT_TIMEOUT);
  client.setConnectTimeout(std::chrono::milliseconds::zero());
  EXPECT_EQ(client.getConfiguredConnectTimeout(), std::chrono::milliseconds::zero());
}

TEST(ClientConnectTimeoutTest, dashboard_client_x_connect_honors_connect_timeout)
{
  UnresponsiveServer robot;
  if (!robot.isUnresponsive())
  {
    GTEST_SKIP() << SKIP_MESSAGE;
  }
  DashboardClient client(LOCALHOST + ":" + std::to_string(robot.getPort()), DashboardClient::ClientPolicy::POLYSCOPE_X);
  client.setConnectTimeout(CONNECT_TIMEOUT);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client.connect());
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, CONNECT_TIMEOUT);
  EXPECT_LT(elapsed, MAX_SETUP_DURATION);
}

TEST(ClientConnectTimeoutTest, ur_driver_honors_socket_connect_timeout)
{
  UnresponsiveServer robot(UR_RTDE_PORT);
  if (!robot.isUnresponsive())
  {
    GTEST_SKIP() << SKIP_MESSAGE;
  }
  UrDriverConfiguration config;
  config.robot_ip = LOCALHOST;
  config.output_recipe = OUTPUT_RECIPE;
  config.input_recipe = INPUT_RECIPE;
  config.headless_mode = true;
  config.socket_reconnect_attempts = 1;
  config.socket_reconnection_timeout = RECONNECTION_TIME;
  config.socket_connect_timeout = CONNECT_TIMEOUT;

  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(UrDriver driver(config), UrException);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_GE(elapsed, CONNECT_TIMEOUT);
  EXPECT_LT(elapsed, MAX_SETUP_DURATION);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
