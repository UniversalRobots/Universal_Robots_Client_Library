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
#include "gtest/gtest.h"
#include "test_utils.h"
#include "ur_client_library/comm/tcp_socket.h"
#include "ur_client_library/ur/dashboard_client_implementation_x.h"
#include "ur_client_library/ur/version_information.h"
#include <ur_client_library/ur/dashboard_client.h>

using namespace urcl;
using namespace std::chrono_literals;

std::string g_ROBOT_IP = "192.168.56.101";

class TestableDashboardClient : public DashboardClientImplX
{
public:
  TestableDashboardClient(const std::string& host) : DashboardClientImplX(host)
  {
  }

  void setPolyscopeVersion(const std::string& version)
  {
    polyscope_version_ = VersionInformation::fromString(version);
  }
};

class DashboardClientTestX : public ::testing::Test
{
protected:
  void SetUp()
  {
    if (robotVersionLessThan(g_ROBOT_IP, "10.11.0"))
    {
      GTEST_SKIP_("Running DashboardClient tests only supported from version 10.11.0 on.");
    }

    dashboard_client_.reset(new TestableDashboardClient(g_ROBOT_IP));
    urcl::comm::INotifier notifier;
    primary_client_.reset(new urcl::primary_interface::PrimaryClient(g_ROBOT_IP, notifier));
    primary_client_->start();
  }

  void TearDown()
  {
    dashboard_client_.reset();
  }

  void waitForRobotMode(const RobotMode& robot_mode)
  {
    waitFor([&]() { return robot_mode == primary_client_->getRobotMode(); }, 10s, std::chrono::milliseconds(200));
    URCL_LOG_INFO("Robot has reached state %s", robotModeString(robot_mode).c_str());
  }

  std::unique_ptr<TestableDashboardClient> dashboard_client_;
  std::unique_ptr<urcl::primary_interface::PrimaryClient> primary_client_;
};

TEST_F(DashboardClientTestX, connect)
{
  EXPECT_TRUE(dashboard_client_->connect());

  auto dashboard_client = std::make_shared<TestableDashboardClient>("192.168.56.123");
  EXPECT_FALSE(dashboard_client->connect(2, std::chrono::milliseconds(500)));
}

TEST_F(DashboardClientTestX, power_cycle)
{
  ASSERT_TRUE(dashboard_client_->connect());
  dashboard_client_->commandPowerOff();
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::POWER_OFF));
  DashboardResponse response;
  response = dashboard_client_->commandPowerOn();
  ASSERT_TRUE(response.ok);
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::IDLE));
  response = dashboard_client_->commandBrakeRelease();
  ASSERT_TRUE(response.ok);
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::RUNNING));
  response = dashboard_client_->commandPowerOff();
  ASSERT_TRUE(response.ok);
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::POWER_OFF));
}

TEST_F(DashboardClientTestX, unlock_protective_stop)
{
  ASSERT_TRUE(dashboard_client_->connect());
  dashboard_client_->commandPowerOn();
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::IDLE));
  DashboardResponse response;
  response = dashboard_client_->commandBrakeRelease();
  ASSERT_TRUE(response.ok);
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::RUNNING));
  primary_client_->sendScript("protective_stop()");
  waitFor(
      [this]() {
        try
        {
          return primary_client_->isRobotProtectiveStopped();
        }
        catch (const UrException& e)
        {
          return false;
        }
      },
      std::chrono::milliseconds(1000));
  response = dashboard_client_->commandUnlockProtectiveStop();
  ASSERT_TRUE(response.ok);
  EXPECT_NO_THROW(waitFor([this]() { return primary_client_->isRobotProtectiveStopped() == false; },
                          std::chrono::milliseconds(1000)));
}

// This currently cannot be tested, as I don't know a way to provoke a state that requires
// restarting safety.
// TEST_F(DashboardClientTestX, restart_safety)
//{
// ASSERT_TRUE(dashboard_client_->connect());
// dashboard_client_->commandPowerOff();
// ASSERT_TRUE(dashboard_client_->commandRestartSafety());
// ASSERT_TRUE(dashboard_client_->commandPowerOn());
//}

TEST_F(DashboardClientTestX, program_interaction)
{
  ASSERT_TRUE(dashboard_client_->connect());
  dashboard_client_->commandPowerOff();
  DashboardResponse response;
  response = dashboard_client_->commandLoadProgram("wait_program");
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPowerOn();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandBrakeRelease();
  ASSERT_TRUE(response.ok);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  response = dashboard_client_->commandPlay();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PLAYING");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  response = dashboard_client_->commandPause();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PAUSED");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  response = dashboard_client_->commandResume();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PLAYING");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  response = dashboard_client_->commandStop();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "STOPPED");
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      g_ROBOT_IP = argv[i + 1];
      break;
    }
  }

  return RUN_ALL_TESTS();
}
