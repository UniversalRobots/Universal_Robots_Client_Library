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
#include <algorithm>
#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "test_utils.h"
#include "ur_client_library/comm/tcp_socket.h"
#include "ur_client_library/ur/dashboard_client_implementation_x.h"
#include "ur_client_library/ur/version_information.h"
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/dashboard_client_implementation.h>

using namespace urcl;
using namespace std::chrono_literals;

std::string g_ROBOT_IP = "192.168.56.101";

class TestableDashboardClientImplX : public DashboardClientImplX
{
public:
  TestableDashboardClientImplX(const std::string& host) : DashboardClientImplX(host)
  {
  }

  const VersionInformation& getRobotApiVersion() const
  {
    return robot_api_version_;
  }
};

class DashboardClientTestX : public ::testing::Test
{
protected:
  void SetUp()
  {
#ifdef POLYSCOPE_X_TESTS_WITH_REMOTE_CONTROL
#  if POLYSCOPE_X_TESTS_WITH_REMOTE_CONTROL == 1
    skip_remote_control_tests = false;
#  endif
#endif
    urcl::comm::INotifier notifier;
    primary_client_.reset(new urcl::primary_interface::PrimaryClient(g_ROBOT_IP, notifier));
    primary_client_->start();
    polyscope_version_ = primary_client_->getRobotVersion();
    if (*polyscope_version_ < urcl::VersionInformation::fromString("10.11.0"))
    {
      GTEST_SKIP_("Running DashboardClient tests only supported from version 10.11.0 on.");
    }

    dashboard_client_.reset(new TestableDashboardClientImplX(g_ROBOT_IP));
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

  std::unique_ptr<TestableDashboardClientImplX> dashboard_client_;
  std::unique_ptr<urcl::primary_interface::PrimaryClient> primary_client_;
  std::shared_ptr<VersionInformation> polyscope_version_;
  bool skip_remote_control_tests = true;
  int error_code_exists = 400;
};

TEST_F(DashboardClientTestX, connect)
{
  EXPECT_TRUE(dashboard_client_->connect());

  auto dashboard_client = std::make_shared<DashboardClientImplX>("192.168.56.123");
  EXPECT_FALSE(dashboard_client->connect(2, std::chrono::milliseconds(500)));
}

TEST_F(DashboardClientTestX, get_loaded_program)
{
  ASSERT_TRUE(dashboard_client_->connect());

  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandGetLoadedProgram(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandGetLoadedProgram();
    ASSERT_TRUE(response.ok);
    ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "Default program");
  }
}

TEST_F(DashboardClientTestX, power_cycle)
{
  if (skip_remote_control_tests)
  {
    GTEST_SKIP_("Skipping test that would require remote control to be enabled on robot");
  }
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
  if (skip_remote_control_tests)
  {
    GTEST_SKIP_("Skipping test that would require remote control to be enabled on robot");
  }
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
  if (skip_remote_control_tests)
  {
    GTEST_SKIP_("Skipping test that would require remote control to be enabled on robot");
  }
  ASSERT_TRUE(dashboard_client_->connect());
  dashboard_client_->commandPowerOff();
  DashboardResponse response;
  response = dashboard_client_->commandLoadProgram("wait_program");
  ASSERT_TRUE(response.ok);
  if (dashboard_client_->getRobotApiVersion() >= VersionInformation::fromString("3.1.4"))
  {
    response = dashboard_client_->commandGetLoadedProgram();
    ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program");
  }
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

TEST_F(DashboardClientTestX, get_control_mode)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandIsInRemoteControl(), NotImplementedException);
  }
  else
  {
    DashboardResponse response = dashboard_client_->commandIsInRemoteControl();
    ASSERT_TRUE(response.ok);
    if (skip_remote_control_tests)
    {
      ASSERT_FALSE(std::get<bool>(response.data["remote_control"]));
    }
    else
    {
      ASSERT_TRUE(std::get<bool>(response.data["remote_control"]));
    }
  }
}

TEST_F(DashboardClientTestX, get_operational_mode)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandGetOperationalMode(), NotImplementedException);
  }
  else
  {
    DashboardResponse response = dashboard_client_->commandGetOperationalMode();
    ASSERT_TRUE(response.ok);
    EXPECT_PRED3([](auto str, auto s1, auto s2) { return (str == s1 || str == s2); },
                 std::get<std::string>(response.data["operational_mode"]), "AUTOMATIC", "MANUAL");
  }
}

TEST_F(DashboardClientTestX, get_safety_mode)
{
  ASSERT_TRUE(dashboard_client_->connect());
  const std::vector<std::string> valid_states = { "NORMAL", "REDUCED", "FAULT", "PROTECTIVE_STOP", "EMERGENCY_STOP" };
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandSafetyMode(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandSafetyMode();
    ASSERT_TRUE(response.ok);
    const std::string safety_mode = std::get<std::string>(response.data["safety_mode"]);
    ASSERT_TRUE(std::any_of(valid_states.begin(), valid_states.end(),
                            [&safety_mode](const std::string& val) { return val == safety_mode; }));

    if (!skip_remote_control_tests)
    {
      DashboardResponse response;
      dashboard_client_->commandPowerOff();
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::POWER_OFF));
      response = dashboard_client_->commandPowerOn();
      ASSERT_TRUE(response.ok);
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::IDLE));
      response = dashboard_client_->commandBrakeRelease();
      ASSERT_TRUE(response.ok);
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::RUNNING));
      primary_client_->sendScript("protective_stop()");
      std::this_thread::sleep_for(1000ms);
      response = dashboard_client_->commandSafetyMode();
      ASSERT_TRUE(response.ok);
      ASSERT_EQ(std::get<std::string>(response.data["safety_mode"]), "PROTECTIVE_STOP");
      response = dashboard_client_->commandUnlockProtectiveStop();
      ASSERT_TRUE(response.ok);
      response = dashboard_client_->commandSafetyMode();
      ASSERT_TRUE(response.ok);
      ASSERT_EQ(std::get<std::string>(response.data["safety_mode"]), "NORMAL");
    }
  }
}

TEST_F(DashboardClientTestX, get_robot_mode)
{
  const std::vector<std::string> valid_states = { "NO_CONTROLLER", "DISCONNECTED", "CONFIRM_SAFETY", "BOOTING",
                                                  "POWER_OFF",     "POWER_ON",     "IDLE",           "BACKDRIVE",
                                                  "RUNNING",       "UPDATING" };
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandRobotMode(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandRobotMode();
    ASSERT_TRUE(response.ok);
    const std::string robot_mode = std::get<std::string>(response.data["robot_mode"]);
    ASSERT_TRUE(std::any_of(valid_states.begin(), valid_states.end(),
                            [&robot_mode](const std::string& val) { return val == robot_mode; }));

    if (!skip_remote_control_tests)
    {
      DashboardResponse response;
      dashboard_client_->commandPowerOff();
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::POWER_OFF));
      response = dashboard_client_->commandRobotMode();
      ASSERT_TRUE(response.ok);
      ASSERT_EQ(std::get<std::string>(response.data["robot_mode"]), "POWER_OFF");
      response = dashboard_client_->commandPowerOn();
      ASSERT_TRUE(response.ok);
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::IDLE));
      response = dashboard_client_->commandRobotMode();
      ASSERT_TRUE(response.ok);
      ASSERT_EQ(std::get<std::string>(response.data["robot_mode"]), "IDLE");
      response = dashboard_client_->commandBrakeRelease();
      ASSERT_TRUE(response.ok);
      ASSERT_NO_THROW(waitForRobotMode(RobotMode::RUNNING));
      response = dashboard_client_->commandRobotMode();
      ASSERT_TRUE(response.ok);
      ASSERT_EQ(std::get<std::string>(response.data["robot_mode"]), "RUNNING");
    }
  }
}

TEST_F(DashboardClientTestX, get_program_list)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandGetProgramList(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandGetProgramList();
    ASSERT_TRUE(response.ok);
  }
}

TEST_F(DashboardClientTestX, upload_program_from_file)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandUploadProgram("resources/upload_prog.urpx"), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandUploadProgram("resources/upload_prog.urpx");

    // Either the upload succeeded, or it failed because the program already exists. Both cases are
    // ok, as we just want to verify that the upload functionality works in principle, and we don't
    // cannot clean up the uploaded program after the test.
    if (!response.ok)
    {
      URCL_LOG_INFO("status code: %d", std::get<int>(response.data["status_code"]));
      ASSERT_EQ(std::get<int>(response.data["status_code"]), error_code_exists);
    }

    response = dashboard_client_->commandUploadProgram("non_existent_file.urpx");
    ASSERT_FALSE(response.ok);
    ASSERT_EQ(response.message, "URPX File not found: non_existent_file.urpx");
  }
}

TEST_F(DashboardClientTestX, upload_and_update_program_from_file)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandUploadProgram("resources/update_prog.urpx"), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandUploadProgram("resources/update_prog.urpx");
    if (!response.ok)
    {
      ASSERT_EQ(std::get<int>(response.data["status_code"]), error_code_exists);
    }

    response = dashboard_client_->commandUpdateProgram("resources/update_prog.urpx");
    ASSERT_TRUE(response.ok);
  }
}

TEST_F(DashboardClientTestX, download_program)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("3.1.4"))
  {
    ASSERT_THROW(dashboard_client_->commandDownloadProgram("test upload", "/tmp/downloaded.urpx"),
                 NotImplementedException);
  }
  else
  {
    // Make sure the target program exists. This call might fail, that's ok.
    auto response = dashboard_client_->commandUploadProgram("resources/upload_prog.urpx");
    response = dashboard_client_->commandDownloadProgram("test upload", "/tmp/downloaded.urpx");
    ASSERT_TRUE(response.ok);

    // TODO: The following doesn't work, as the uploaded program might get another ID and will get another creation
    // date. We would need to parse the json data for relevant parts in order to compare them.

    /*
    std::ifstream orig_file("resources/upload_prog.urpx");
    std::stringstream orig_content;
    orig_content << orig_file.rdbuf();
    std::ifstream downloaded_file("/tmp/downloaded.urpx");
    std::stringstream downloaded_content;
    downloaded_content << downloaded_file.rdbuf();
    ASSERT_EQ(orig_content.str(), downloaded_content.str());
    */

    response = dashboard_client_->commandDownloadProgram("non_existent_program", "/tmp/downloaded.urpx");
    ASSERT_FALSE(response.ok);

    response = dashboard_client_->commandDownloadProgram("test upload", "/non_existent_dir/downloaded.urpx");
    ASSERT_FALSE(response.ok);
  }
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
