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
#include <ur_client_library/exceptions.h>
#include <chrono>
#include <thread>
#include "ur_client_library/comm/tcp_socket.h"
#include "ur_client_library/ur/version_information.h"
#define private public
#include <ur_client_library/ur/dashboard_client.h>

using namespace urcl;

std::string ROBOT_IP = "192.168.56.101";

class DashboardClientTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    dashboard_client_.reset(new DashboardClient(ROBOT_IP));
  }

  void TearDown()
  {
    dashboard_client_.reset();
  }

  std::unique_ptr<DashboardClient> dashboard_client_;
};

TEST_F(DashboardClientTest, connect)
{
  EXPECT_TRUE(dashboard_client_->connect());
  dashboard_client_->commandCloseSafetyPopup();
}

TEST_F(DashboardClientTest, run_program)
{
  EXPECT_TRUE(dashboard_client_->connect());
  EXPECT_TRUE(dashboard_client_->commandLoadProgram("wait_program.urp"));
  EXPECT_TRUE(dashboard_client_->commandPowerOff());
  dashboard_client_->commandClosePopup();  // Necessary for CB3 test
  EXPECT_TRUE(dashboard_client_->commandPowerOn());
  EXPECT_TRUE(dashboard_client_->commandBrakeRelease());
  EXPECT_TRUE(dashboard_client_->commandPlay());
  EXPECT_TRUE(dashboard_client_->commandPause());
  EXPECT_TRUE(dashboard_client_->commandPlay());
  EXPECT_TRUE(dashboard_client_->commandStop());
  EXPECT_TRUE(dashboard_client_->commandPowerOff());
  dashboard_client_->commandClosePopup();  // Necessary for CB3 test
}

TEST_F(DashboardClientTest, load_installation)
{
  EXPECT_TRUE(dashboard_client_->connect());
  EXPECT_TRUE(dashboard_client_->commandLoadInstallation("default.installation"));
}

TEST_F(DashboardClientTest, not_connected)
{
  EXPECT_THROW(dashboard_client_->commandPowerOff(), UrException);
}

TEST_F(DashboardClientTest, popup)
{
  EXPECT_TRUE(dashboard_client_->connect());
  EXPECT_TRUE(dashboard_client_->commandPopup("Test Popup"));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Give time for popup to pop up
  EXPECT_TRUE(dashboard_client_->commandClosePopup());
}

TEST_F(DashboardClientTest, log_and_getters)
{
  std::string msg;
  EXPECT_TRUE(dashboard_client_->connect());
  EXPECT_TRUE(dashboard_client_->commandAddToLog("Testing Log:"));
  EXPECT_TRUE(dashboard_client_->commandPolyscopeVersion(msg));
  EXPECT_TRUE(dashboard_client_->commandAddToLog("Polyscope Version: " + msg));
  EXPECT_TRUE(dashboard_client_->commandRobotMode(msg));
  EXPECT_TRUE(dashboard_client_->commandAddToLog("Robot mode: " + msg));
  EXPECT_TRUE(dashboard_client_->commandGetLoadedProgram(msg));
  EXPECT_TRUE(dashboard_client_->commandAddToLog("Loaded program: " + msg));
  EXPECT_TRUE(dashboard_client_->commandProgramState(msg));
  EXPECT_TRUE(dashboard_client_->commandAddToLog("Program state: " + msg));
}

TEST_F(DashboardClientTest, flight_report_and_support_file)
{
  EXPECT_TRUE(dashboard_client_->connect());
  bool correct_polyscope_version = true;
  try
  {
    dashboard_client_->assertVersion("5.6.0", "3.13", "test_function");
  }
  catch (const UrException& e)
  {
    correct_polyscope_version = false;
  }

  if (correct_polyscope_version)
  {
    EXPECT_TRUE(dashboard_client_->commandGenerateFlightReport(""));
    EXPECT_TRUE(dashboard_client_->commandGenerateSupportFile("."));
  }
  else
  {
    EXPECT_THROW(dashboard_client_->commandGenerateFlightReport(""), UrException);
    EXPECT_THROW(dashboard_client_->commandGenerateSupportFile("."), UrException);
  }
}

// Only runs this test if robot is e-series
TEST_F(DashboardClientTest, e_series_version)
{
  std::string msg;
  EXPECT_TRUE(dashboard_client_->connect());
  if (!dashboard_client_->polyscope_version_.isESeries())
    GTEST_SKIP();
  dashboard_client_->polyscope_version_ = VersionInformation::fromString("5.0.0");
  EXPECT_THROW(dashboard_client_->commandSafetyStatus(msg), UrException);
  dashboard_client_->polyscope_version_ = VersionInformation::fromString("5.5.0");
  EXPECT_TRUE(dashboard_client_->commandSafetyStatus(msg));
  EXPECT_THROW(dashboard_client_->commandSetUserRole("none"), UrException);
}

// Only runs this test if robot is CB3
TEST_F(DashboardClientTest, cb3_version)
{
  std::string msg;
  EXPECT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->polyscope_version_.isESeries())
    GTEST_SKIP();
  dashboard_client_->polyscope_version_ = VersionInformation::fromString("1.6.0");
  EXPECT_THROW(dashboard_client_->commandIsProgramSaved(), UrException);
  dashboard_client_->polyscope_version_ = VersionInformation::fromString("1.8.0");
  EXPECT_TRUE(dashboard_client_->commandIsProgramSaved());
  EXPECT_THROW(dashboard_client_->commandIsInRemoteControl(), UrException);
}

TEST_F(DashboardClientTest, set_receive_timeout)
{
  timeval expected_tv;
  expected_tv.tv_sec = 30;
  expected_tv.tv_usec = 0.0;
  dashboard_client_->setReceiveTimeout(expected_tv);
  EXPECT_TRUE(dashboard_client_->connect());

  // Ensure that the receive timeout hasn't been overwritten
  timeval actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
  EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
  EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);

  bool correct_polyscope_version = true;
  try
  {
    dashboard_client_->assertVersion("5.6.0", "3.13", "test_function");
  }
  catch (const UrException& e)
  {
    correct_polyscope_version = false;
  }
  if (correct_polyscope_version)
  {
    EXPECT_TRUE(dashboard_client_->commandGenerateFlightReport(""));
    // Ensure that the receive timeout hasn't been overwritten
    actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
    EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
    EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);

    EXPECT_TRUE(dashboard_client_->commandGenerateSupportFile("."));
    // Ensure that the receive timeout hasn't been overwritten
    actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
    EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
    EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);
  }
}

TEST_F(DashboardClientTest, connect_non_running_robot)
{
  std::unique_ptr<DashboardClient> dashboard_client;
  // We use an IP address on the integration_test's subnet
  dashboard_client.reset(new DashboardClient("192.168.56.123"));
  auto start = std::chrono::system_clock::now();
  EXPECT_FALSE(dashboard_client->connect(2, std::chrono::milliseconds(500)));
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  // This is only a rough estimate, obviously.
  // Since this isn't done on the loopback device, trying to open a socket on a non-existing address
  // takes considerably longer.
  EXPECT_LT(elapsed, 2 * comm::TCPSocket::DEFAULT_RECONNECTION_TIME);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      ROBOT_IP = argv[i + 1];
      break;
    }
  }

  return RUN_ALL_TESTS();
}
