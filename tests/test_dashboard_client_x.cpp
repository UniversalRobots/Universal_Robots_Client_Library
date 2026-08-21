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
#include <cstdlib>
#include <filesystem>
#include <thread>
#include <vector>
#ifndef _WIN32
#  include <fcntl.h>
#  include <sys/wait.h>
#  include <unistd.h>
#endif
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
    if (std::getenv("POLYSCOPE_X_TESTS_WITH_REMOTE_CONTROL") != nullptr)
    {
      std::string env_var = std::getenv("POLYSCOPE_X_TESTS_WITH_REMOTE_CONTROL");
      if (env_var != "" && parseBoolean(env_var))
      {
        skip_remote_control_tests = false;
      }
    }
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
  static constexpr int ERROR_CODE_EXISTS = 400;
  static constexpr int ERROR_CODE_CONFLICT = 409;
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
    if (!skip_remote_control_tests)
    {
      dashboard_client_->commandLoadProgram("Default program");
    }
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
  dashboard_client_->commandPowerOff();
  ASSERT_NO_THROW(waitForRobotMode(RobotMode::POWER_OFF));
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
    waitFor(
        [&]() {
          auto resp = dashboard_client_->commandGetLoadedProgram();
          return std::get<std::string>(resp.data["program_name"]) == "wait_program";
        },
        std::chrono::milliseconds(5000));
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
      bool is_exists_error = std::get<int>(response.data["status_code"]) == ERROR_CODE_EXISTS;
      bool is_conflict_error = std::get<int>(response.data["status_code"]) == ERROR_CODE_CONFLICT;
      ASSERT_TRUE(is_exists_error || is_conflict_error);
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
      bool is_exists_error = std::get<int>(response.data["status_code"]) == ERROR_CODE_EXISTS;
      bool is_conflict_error = std::get<int>(response.data["status_code"]) == ERROR_CODE_CONFLICT;
      ASSERT_TRUE(is_exists_error || is_conflict_error);
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

    response = dashboard_client_->commandDownloadProgram("", "/tmp/downloaded.urpx");
    ASSERT_FALSE(response.ok);

    response = dashboard_client_->commandDownloadProgram("test upload", "");
    ASSERT_FALSE(response.ok);
  }
}

TEST_F(DashboardClientTestX, set_receive_timeout)
{
  timeval expected_tv;
  expected_tv.tv_sec = 30;
  expected_tv.tv_usec = 0;
  dashboard_client_->setReceiveTimeout(expected_tv);
  EXPECT_TRUE(dashboard_client_->connect());

  timeval actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
  EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
  EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);
}

TEST_F(DashboardClientTestX, set_send_timeout)
{
  timeval expected_tv;
  expected_tv.tv_sec = 30;
  expected_tv.tv_usec = 0;
  dashboard_client_->setSendTimeout(expected_tv);
  EXPECT_TRUE(dashboard_client_->connect());

  timeval actual_tv = dashboard_client_->getConfiguredSendTimeout();
  EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
  EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);
}

TEST_F(DashboardClientTestX, timeouts_round_trip_subsecond)
{
  timeval expected_tv;
  expected_tv.tv_sec = 7;
  expected_tv.tv_usec = 250000;
  dashboard_client_->setReceiveTimeout(expected_tv);
  dashboard_client_->setSendTimeout(expected_tv);

  timeval recv_tv = dashboard_client_->getConfiguredReceiveTimeout();
  EXPECT_EQ(expected_tv.tv_sec, recv_tv.tv_sec);
  EXPECT_EQ(expected_tv.tv_usec, recv_tv.tv_usec);

  timeval send_tv = dashboard_client_->getConfiguredSendTimeout();
  EXPECT_EQ(expected_tv.tv_sec, send_tv.tv_sec);
  EXPECT_EQ(expected_tv.tv_usec, send_tv.tv_usec);
}

TEST_F(DashboardClientTestX, microsecond_receive_timeout_makes_connect_fail)
{
  timeval tiny_tv;
  tiny_tv.tv_sec = 0;
  tiny_tv.tv_usec = 1;
  dashboard_client_->setReceiveTimeout(tiny_tv);
  EXPECT_FALSE(dashboard_client_->connect());
}

TEST_F(DashboardClientTestX, open_and_close_popups)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandPopup(""), NotImplementedException);
    ASSERT_THROW(dashboard_client_->commandClosePopup(), NotImplementedException);
    ASSERT_THROW(dashboard_client_->commandCloseSafetyPopup(), NotImplementedException);
  }
  else
  {
    // Can only be tested in remote mode, so just check that we get the correct error message
    // Then we know the endpoint exists and the client is sending the correct message
    if (skip_remote_control_tests)
    {
      auto response = dashboard_client_->commandClosePopup();
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("Forbidden") != response.message.npos);
      response = dashboard_client_->commandPopup("Test popup", "Test");
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("Forbidden") != response.message.npos);
      response = dashboard_client_->commandClosePopup();
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("Forbidden") != response.message.npos);
      response = dashboard_client_->commandCloseSafetyPopup();
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("Forbidden") != response.message.npos);
    }
    else
    {
      auto response = dashboard_client_->commandClosePopup();
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("\"closed\":false") != response.message.npos);
      response = dashboard_client_->commandPopup("Test popup", "Test");
      ASSERT_TRUE(response.ok);
      ASSERT_TRUE(response.message.find("Popup opened successfully") != response.message.npos);
      response = dashboard_client_->commandClosePopup();
      ASSERT_TRUE(response.ok);
      response = dashboard_client_->commandCloseSafetyPopup();
      ASSERT_FALSE(response.ok);
      ASSERT_TRUE(response.message.find("\"closed\":false") != response.message.npos);
    }
  }
}

TEST_F(DashboardClientTestX, get_polyscope_version)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandPolyscopeVersion(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandPolyscopeVersion();
    ASSERT_TRUE(response.ok);
    std::string version_string = std::get<std::string>(response.data["polyscope_version"]);
    EXPECT_EQ(*polyscope_version_, VersionInformation::fromString(version_string));
  }
}

TEST_F(DashboardClientTestX, get_robot_model)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandGetRobotModel(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandGetRobotModel();
    ASSERT_TRUE(response.ok);
    const std::string model_string = std::get<std::string>(response.data["robot_model"]);

    waitFor([this]() { return primary_client_->getRobotType() != urcl::RobotType::UNDEFINED; },
            std::chrono::milliseconds(1000));

    const std::string primary_client_version = robotTypeString(primary_client_->getRobotType());
    std::string expected_model_string = primary_client_version;

    // On the primary interface UR7 and UR12 show as UR5 and UR10, so we need to adjust the
    // expected value accordingly.
    if (model_string == "UR7")
    {
      expected_model_string = "UR5";
    }
    else if (model_string == "UR12")
    {
      expected_model_string = "UR10";
    }
    EXPECT_EQ(model_string, primary_client_version);
  }
}

TEST_F(DashboardClientTestX, get_serial_number)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandGetSerialNumber(), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandGetSerialNumber();
    ASSERT_TRUE(response.ok);
    const std::string serial_number = std::get<std::string>(response.data["serial_number"]);
    EXPECT_FALSE(serial_number.empty());  // Dont know what to check for here otherwise
  }
}

TEST_F(DashboardClientTestX, shutdown_robot)
{
  // On URSim this will not really shutdown the docker container, so we can test it in CI. When
  // tests are run on a real robot, this test should be skipped, as it will shutdown the robot and
  // the test will fail.
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandShutdown(), NotImplementedException);
  }
  else
  {
    if (skip_remote_control_tests)
    {
      auto response = dashboard_client_->commandShutdown();
      ASSERT_FALSE(response.ok);
      EXPECT_TRUE(response.message.find("Forbidden") != response.message.npos);
    }
    else
    {
      auto response = dashboard_client_->commandShutdown();
      ASSERT_TRUE(response.ok);
    }
  }
}

TEST_F(DashboardClientTestX, add_to_log)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandAddToLog(""), NotImplementedException);
  }
  else
  {
    if (skip_remote_control_tests)
    {
      auto response = dashboard_client_->commandAddToLog("Test log");
      ASSERT_FALSE(response.ok);
      EXPECT_TRUE(response.message.find("Forbidden") != response.message.npos);
    }
    else
    {
      auto response = dashboard_client_->commandAddToLog("Test log");
      ASSERT_TRUE(response.ok);
      EXPECT_TRUE(response.message.find("Log entry added") != response.message.npos);
    }
  }
}

TEST_F(DashboardClientTestX, generate_flight_report)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandGenerateFlightReport(), NotImplementedException);
  }
  else
  {
    if (skip_remote_control_tests)
    {
      auto response = dashboard_client_->commandGenerateFlightReport();
      ASSERT_FALSE(response.ok);
      EXPECT_TRUE(response.message.find("Forbidden") != response.message.npos);
    }
    else
    {
      auto response = dashboard_client_->commandGenerateFlightReport();
      // On URSim this can't be used
      // ASSERT_TRUE(response.ok);
      // EXPECT_TRUE(response.message.find("Flight report generated") != response.message.npos);
    }
  }
}

TEST_F(DashboardClientTestX, download_support_files)
{
  ASSERT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getRobotApiVersion() < VersionInformation::fromString("5.0.107"))
  {
    ASSERT_THROW(dashboard_client_->commandDownloadSupportFiles("/tmp/support_files.zip"), NotImplementedException);
  }
  else
  {
    auto response = dashboard_client_->commandDownloadSupportFiles("/tmp/support_files.zip");
    // On URSim this can't be used
    // ASSERT_TRUE(response.ok);
    //// Check that the file was created and has some content
    // std::ifstream file("/tmp/support_files.zip", std::ios::binary | std::ios::ate);
    // ASSERT_TRUE(file.is_open());
    // std::streamsize size = file.tellg();
    // ASSERT_GT(size, 0);
  }
}

class PolyScopeScreenshotListener : public ::testing::EmptyTestEventListener
{
public:
  void OnTestEnd(const ::testing::TestInfo& test_info) override
  {
    if (!test_info.result()->Failed())
    {
      return;
    }

    const char* dir_env = std::getenv("POLYSCOPE_X_SCREENSHOT_DIR");
    std::filesystem::path screenshot_dir = dir_env ? dir_env : "test_artifacts/screenshots";
    std::filesystem::create_directories(screenshot_dir);

    std::string filename =
        (screenshot_dir / (std::string(test_info.test_suite_name()) + "." + test_info.name() + ".png")).string();
    std::string url = "http://" + g_ROBOT_IP;
    std::string script = "../tests/resources/polyscopex_screenshot.py";
    std::string delay = "5000";

#ifndef _WIN32
    // Build argv as a proper array — no shell involved, so spaces and metacharacters
    // in url, filename, or script path are passed through safely.
    std::vector<char*> args = { const_cast<char*>("python3"),     const_cast<char*>(script.c_str()),
                                const_cast<char*>(url.c_str()),   const_cast<char*>(filename.c_str()),
                                const_cast<char*>(delay.c_str()), nullptr };

    pid_t pid = fork();
    if (pid == 0)
    {
      int devnull = open("/dev/null", O_WRONLY);
      if (devnull >= 0)
      {
        dup2(devnull, STDERR_FILENO);
        close(devnull);
      }
      execvp("python3", args.data());
      _exit(1);
    }
    else if (pid > 0)
    {
      waitpid(pid, nullptr, 0);
    }
#endif
  }
};

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      g_ROBOT_IP = argv[i + 1];
      ++i;
    }
  }

  ::testing::UnitTest::GetInstance()->listeners().Append(new PolyScopeScreenshotListener());

  return RUN_ALL_TESTS();
}
