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
#include "ur_client_library/ur/dashboard_client_implementation_g5.h"
#include "ur_client_library/ur/version_information.h"
#include <ur_client_library/ur/dashboard_client.h>

using namespace urcl;

std::string g_ROBOT_IP = "192.168.56.101";

class TestableDashboardClient : public DashboardClientImplG5
{
public:
  TestableDashboardClient(const std::string& host) : DashboardClientImplG5(host)
  {
  }

  void setPolyscopeVersion(const std::string& version)
  {
    polyscope_version_ = VersionInformation::fromString(version);
  }
};

class DashboardClientTestG5 : public ::testing::Test
{
protected:
  void SetUp()
  {
    if (!robotVersionLessThan(g_ROBOT_IP, "10.0.0"))
    {
      GTEST_SKIP_("G5 DashboardClient tests are only applicable for robots with a G5 dashboard server.");
    }

    dashboard_client_.reset(new TestableDashboardClient(g_ROBOT_IP));
    // In CI we the dashboard client times out for no obvious reason. Hence we increase the timeout
    // here.
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    dashboard_client_->setReceiveTimeout(tv);
  }

  void TearDown()
  {
    dashboard_client_.reset();
  }

  std::unique_ptr<TestableDashboardClient> dashboard_client_;
};

TEST_F(DashboardClientTestG5, connect)
{
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandCloseSafetyPopup();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, power_cycle)
{
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response;

  // Cycle from POWER_OFF to IDLE to RUNNING
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandPowerOn(std::chrono::seconds(5));
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandBrakeRelease();
  EXPECT_TRUE(response.ok);

  // Calling power_on on a brake-released robot should succeed
  response = dashboard_client_->commandPowerOn(std::chrono::seconds(5));
  EXPECT_TRUE(response.ok);

  // Power off from brake-released state (RUNNING)
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);

  // Power off from powered_on state (IDLE)
  dashboard_client_->commandPowerOn();
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);

  // Power off from powered_on state (IDLE)
  dashboard_client_->commandPowerOff();
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);

  // Brake release from POWER_OFF should succeed
  dashboard_client_->commandPowerOff();
  response = dashboard_client_->commandBrakeRelease();
  EXPECT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, run_program)
{
  EXPECT_TRUE(dashboard_client_->connect());

  DashboardResponse response;
  response = dashboard_client_->commandLoadProgram("wait_program.urp");
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);
  dashboard_client_->commandClosePopup();  // Necessary for CB3 test
  response = dashboard_client_->commandPowerOn();
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandBrakeRelease();
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandPlay();
  EXPECT_TRUE(response.ok);

  response = dashboard_client_->commandPause();
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandPlay();
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandRunning();
  EXPECT_TRUE(response.ok);
  EXPECT_TRUE(std::get<bool>(response.data["running"]));

  response = dashboard_client_->commandStop();
  EXPECT_TRUE(response.ok);
  response = response = dashboard_client_->commandRunning();
  EXPECT_TRUE(response.ok);
  EXPECT_FALSE(std::get<bool>(response.data["running"]));
  response = dashboard_client_->commandPowerOff();
  EXPECT_TRUE(response.ok);
  dashboard_client_->commandClosePopup();  // Necessary for CB3 test
}

TEST_F(DashboardClientTestG5, load_installation)
{
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandLoadInstallation("default.installation");
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["installation_name"]), "default.installation");
}

TEST_F(DashboardClientTestG5, load_program)
{
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response = dashboard_client_->commandLoadProgram("wait_program.urp");
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
}

TEST_F(DashboardClientTestG5, not_connected)
{
  EXPECT_THROW(dashboard_client_->commandPowerOff(), UrException);
}

TEST_F(DashboardClientTestG5, popup)
{
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandPopup("Test Popup");
  ASSERT_TRUE(response.ok);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Give time for popup to pop up
  response = dashboard_client_->commandClosePopup();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, log_and_getters)
{
  std::string msg = "My message";
  EXPECT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandAddToLog("Testing Log:");
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPolyscopeVersion();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandAddToLog("Polyscope Version: " + msg);
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandRobotMode();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandAddToLog("Robot mode: " + msg);
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandGetLoadedProgram();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandAddToLog("Loaded program: " + msg);
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandAddToLog("Program state: " + msg);
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, flight_report_and_support_file)
{
  EXPECT_TRUE(dashboard_client_->connect());
  bool correct_polyscope_version = false;
  try
  {
    auto version = dashboard_client_->getPolyscopeVersion();
    if (version.major == 5)
    {
      correct_polyscope_version = version >= VersionInformation::fromString("5.6.0");
    }
    else if (version.major == 3)
    {
      correct_polyscope_version = version >= VersionInformation::fromString("3.13");
    }
  }
  catch (const UrException& e)
  {
    correct_polyscope_version = false;
  }

  DashboardResponse response;
  if (correct_polyscope_version)
  {
    response = dashboard_client_->commandGenerateFlightReport("");
    ASSERT_TRUE(response.ok);
    response = dashboard_client_->commandGenerateSupportFile(".");
    ASSERT_TRUE(response.ok);
  }
  else
  {
    EXPECT_THROW(dashboard_client_->commandGenerateFlightReport(""), UrException);
    EXPECT_THROW(dashboard_client_->commandGenerateSupportFile("."), UrException);
  }
}

// Only runs this test if robot is e-series
TEST_F(DashboardClientTestG5, e_series_version)
{
  EXPECT_TRUE(dashboard_client_->connect());
  if (!dashboard_client_->getPolyscopeVersion().isESeries())
    GTEST_SKIP();
  dashboard_client_->setPolyscopeVersion("5.0.0");
  EXPECT_THROW(dashboard_client_->commandSafetyStatus(), UrException);
  dashboard_client_->setPolyscopeVersion("5.5.0");
  DashboardResponse response;
  response = dashboard_client_->commandSafetyStatus();
  ASSERT_TRUE(response.ok);
  EXPECT_THROW(dashboard_client_->commandSetUserRole("none"), UrException);
  EXPECT_THROW(dashboard_client_->commandGetUserRole(), UrException);
}

// Only runs this test if robot is CB3
TEST_F(DashboardClientTestG5, cb3_version)
{
  EXPECT_TRUE(dashboard_client_->connect());
  if (dashboard_client_->getPolyscopeVersion().isESeries())
    GTEST_SKIP();

  DashboardResponse response;
  // Get a defined starting state
  dashboard_client_->commandSetUserRole("NONE");
  response = dashboard_client_->commandSetUserRole("PROGRAMMER");
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandGetUserRole();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["user_role"]), "PROGRAMMER");
  response = dashboard_client_->commandSetUserRole("OPERATOR");
  ASSERT_TRUE(response.ok);
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandGetUserRole();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["user_role"]), "OPERATOR");
  response = dashboard_client_->commandSetUserRole("NONE");
  ASSERT_TRUE(response.ok);
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandGetUserRole();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["user_role"]), "NONE");
  response = dashboard_client_->commandSetUserRole("LOCKED");
  ASSERT_TRUE(response.ok);
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandGetUserRole();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["user_role"]), "LOCKED");
  response = dashboard_client_->commandSetUserRole("RESTRICTED");
  ASSERT_TRUE(response.ok);
  EXPECT_TRUE(response.ok);
  response = dashboard_client_->commandGetUserRole();
  EXPECT_TRUE(response.ok);
  EXPECT_EQ(std::get<std::string>(response.data["user_role"]), "RESTRICTED");

  EXPECT_THROW(dashboard_client_->commandSafetyStatus(), UrException);

  dashboard_client_->setPolyscopeVersion("1.6.0");
  EXPECT_THROW(dashboard_client_->commandIsProgramSaved(), UrException);
  dashboard_client_->setPolyscopeVersion("1.8.0");
  EXPECT_TRUE(dashboard_client_->commandLoadProgram("wait_program.urp").ok);
  response = dashboard_client_->commandIsProgramSaved();
  ASSERT_TRUE(response.ok);
  EXPECT_THROW(dashboard_client_->commandIsInRemoteControl(), UrException);
}

TEST_F(DashboardClientTestG5, set_receive_timeout)
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

  bool correct_polyscope_version = false;
  try
  {
    auto version = dashboard_client_->getPolyscopeVersion();
    if (version.major == 5)
    {
      correct_polyscope_version = version >= VersionInformation::fromString("5.6.0");
    }
    else if (version.major == 3)
    {
      correct_polyscope_version = version >= VersionInformation::fromString("3.13");
    }
  }
  catch (const UrException& e)
  {
    correct_polyscope_version = false;
  }
  if (correct_polyscope_version)
  {
    DashboardResponse response;
    response = dashboard_client_->commandGenerateFlightReport("");
    ASSERT_TRUE(response.ok);
    // Ensure that the receive timeout hasn't been overwritten
    actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
    EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
    EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);

    response = dashboard_client_->commandGenerateSupportFile(".");
    ASSERT_TRUE(response.ok);
    // Ensure that the receive timeout hasn't been overwritten
    actual_tv = dashboard_client_->getConfiguredReceiveTimeout();
    EXPECT_EQ(expected_tv.tv_sec, actual_tv.tv_sec);
    EXPECT_EQ(expected_tv.tv_usec, actual_tv.tv_usec);
  }
}

TEST_F(DashboardClientTestG5, connect_non_running_robot)
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

TEST_F(DashboardClientTestG5, non_expected_result_returns_correctly)
{
  ASSERT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandPowerOff();
  ASSERT_TRUE(response.ok);

  // We will not get this answer
  EXPECT_FALSE(dashboard_client_->sendRequest("brake release", "non-existing-response"));

  // A non-matching answer should throw an exception for this call
  EXPECT_THROW(dashboard_client_->sendRequestString("brake release", "non-existing-response"), UrException);

  // Waiting for a non-matching answer should return false
  // Internally we wait 100ms between each attempt, hence the 300ms wait time
  EXPECT_FALSE(
      dashboard_client_->waitForReply("brake_release", "non-existing-response", std::chrono::milliseconds(300)));
}

TEST_F(DashboardClientTestG5, connecting_twice_returns_false)
{
  ASSERT_TRUE(dashboard_client_->connect());
  EXPECT_FALSE(dashboard_client_->connect());
}

TEST_F(DashboardClientTestG5, load_program_in_subdir_works)
{
  ASSERT_TRUE(dashboard_client_->connect());

  DashboardResponse response;
  response = dashboard_client_->commandLoadProgram("/ursim/programs/wait_program.urp");
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "/ursim/programs/wait_program.urp");
}

TEST_F(DashboardClientTestG5, clear_protective_stop)
{
  ASSERT_TRUE(dashboard_client_->connect());

  auto notifier = comm::INotifier();
  auto primary_client = primary_interface::PrimaryClient(g_ROBOT_IP, notifier);
  primary_client.start();
  primary_client.sendScript("protective_stop()");
  if (dashboard_client_->getPolyscopeVersion().isESeries())
  {
    EXPECT_NO_THROW(waitFor(
        [this]() {
          DashboardResponse response;
          response = dashboard_client_->commandSafetyStatus();
          return std::get<std::string>(response.data["safety_status"]) == "PROTECTIVE_STOP";
        },
        std::chrono::milliseconds(1000)));
  }
  std::string safety_mode;
  EXPECT_NO_THROW(waitFor(
      [this]() {
        DashboardResponse response;
        response = dashboard_client_->commandSafetyMode();
        return std::get<std::string>(response.data["safety_mode"]) == "PROTECTIVE_STOP";
      },
      std::chrono::milliseconds(5000)));

  sleep(5);  // Wait for the protective stop to be triggered

  // Clear the protective stop
  DashboardResponse response;
  response = dashboard_client_->commandUnlockProtectiveStop();
  ASSERT_TRUE(response.ok);

  // Check that the protective stop is cleared
  if (dashboard_client_->getPolyscopeVersion().isESeries())
  {
    EXPECT_NO_THROW(waitFor(
        [this]() {
          DashboardResponse response;
          response = dashboard_client_->commandSafetyStatus();
          return std::get<std::string>(response.data["safety_status"]) == "NORMAL";
        },
        std::chrono::milliseconds(5000)));
  }
  EXPECT_NO_THROW(waitFor(
      [this]() {
        DashboardResponse response;
        response = dashboard_client_->commandSafetyMode();
        return std::get<std::string>(response.data["safety_mode"]) == "NORMAL";
      },
      std::chrono::milliseconds(5000)));
  response = dashboard_client_->commandSafetyMode();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandCloseSafetyPopup();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, restart_safety)
{
  ASSERT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandPowerOff();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandRestartSafety();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPowerOn();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, program_interaction)
{
  DashboardResponse response;
  ASSERT_TRUE(dashboard_client_->connect());
  response = dashboard_client_->commandLoadProgram("wait_program.urp");
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPowerOff();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPowerOn();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandBrakeRelease();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandPlay();
  ASSERT_TRUE(response.ok);
  EXPECT_NO_THROW(waitFor(
      [this]() {
        DashboardResponse response = dashboard_client_->commandRunning();
        return response.ok && std::get<bool>(response.data["running"]);
      },
      std::chrono::milliseconds(500)));
  response = dashboard_client_->commandProgramState();
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PLAYING");
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
  response = dashboard_client_->commandPause();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PAUSED");
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
  response = dashboard_client_->commandPlay();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "PLAYING");
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
  response = dashboard_client_->commandStop();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandProgramState();
  ASSERT_TRUE(response.ok);
  ASSERT_EQ(std::get<std::string>(response.data["program_state"]), "STOPPED");
  ASSERT_EQ(std::get<std::string>(response.data["program_name"]), "wait_program.urp");
  response = dashboard_client_->commandRunning();
  ASSERT_TRUE(response.ok);
  ASSERT_FALSE(std::get<bool>(response.data["running"]));
  response = dashboard_client_->commandIsProgramSaved();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, test_save_log)
{
  ASSERT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  response = dashboard_client_->commandAddToLog("Test log message");
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandSaveLog();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, operational_mode)
{
  ASSERT_TRUE(dashboard_client_->connect());
  DashboardResponse response;
  if (dashboard_client_->getPolyscopeVersion().isESeries())
  {
    response = dashboard_client_->commandSetOperationalMode("AUTOMATIC");
    ASSERT_TRUE(response.ok);
    EXPECT_NO_THROW(waitFor(
        [this]() {
          DashboardResponse response;
          response = dashboard_client_->commandGetOperationalMode();
          return std::get<std::string>(response.data["operational_mode"]) == "AUTOMATIC";
        },
        std::chrono::milliseconds(500)));
    response = dashboard_client_->commandClearOperationalMode();
    ASSERT_TRUE(response.ok);
    // response = dashboard_client_->commandGetOperationalMode(operational_mode);
    ASSERT_TRUE(response.ok);
    response = dashboard_client_->commandSetOperationalMode("MANUAL");
    ASSERT_TRUE(response.ok);
    EXPECT_NO_THROW(waitFor(
        [this]() {
          DashboardResponse response;
          response = dashboard_client_->commandGetOperationalMode();
          return std::get<std::string>(response.data["operational_mode"]) == "MANUAL";
        },
        std::chrono::milliseconds(500)));
    response = dashboard_client_->commandClearOperationalMode();
    ASSERT_TRUE(response.ok);
    EXPECT_NO_THROW(waitFor(
        [this]() {
          DashboardResponse response;
          response = dashboard_client_->commandGetOperationalMode();
          return std::get<std::string>(response.data["operational_mode"]) == "NONE";
        },
        std::chrono::milliseconds(500)));
    response = dashboard_client_->commandIsInRemoteControl();
    ASSERT_TRUE(response.ok);
    ASSERT_FALSE(std::get<bool>(response.data["remote_control"]));
  }
  else
  {
    EXPECT_THROW(response = dashboard_client_->commandGetOperationalMode(), UrException);
    EXPECT_THROW(response = dashboard_client_->commandSetOperationalMode("NONE"), UrException);
    EXPECT_THROW(response = dashboard_client_->commandIsInRemoteControl(), UrException);
  }
}

TEST_F(DashboardClientTestG5, test_disconnect)
{
  ASSERT_TRUE(dashboard_client_->connect());
  dashboard_client_->disconnect();
  DashboardResponse response;
  EXPECT_THROW(dashboard_client_->commandRobotMode(), UrException);
  ASSERT_TRUE(dashboard_client_->connect());
  response = dashboard_client_->commandRobotMode();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandQuit();
  ASSERT_TRUE(response.ok);
  EXPECT_THROW(dashboard_client_->commandRobotMode(), UrException);
}

TEST_F(DashboardClientTestG5, get_robot_info)
{
  ASSERT_TRUE(dashboard_client_->connect());
  std::string robot_model;
  std::string serial_number;
  DashboardResponse response;
  response = dashboard_client_->commandGetRobotModel();
  ASSERT_TRUE(response.ok);
  response = dashboard_client_->commandGetSerialNumber();
  ASSERT_TRUE(response.ok);
}

TEST_F(DashboardClientTestG5, unknown_command_throws)
{
  ASSERT_TRUE(dashboard_client_->connect());
  EXPECT_THROW(dashboard_client_->sendRequest("unknown_command"), UrException);
  EXPECT_THROW(dashboard_client_->sendRequestString("unknown_command"), UrException);
}

TEST_F(DashboardClientTestG5, run_commands_through_client)
{
  auto dashboard_client = std::make_shared<DashboardClient>(g_ROBOT_IP);

  ASSERT_TRUE(dashboard_client_->connect());

  ASSERT_TRUE(dashboard_client_->sendRequest("power off"));
  ASSERT_TRUE(dashboard_client_->waitForReply("robotmode", "Robotmode: POWER_OFF"));  // Necessary for CB3 test
  ASSERT_TRUE(dashboard_client_->retryCommand("power on", "Powering on", "robotmode", "Robotmode: IDLE",
                                              std::chrono::seconds(5)));
  EXPECT_THROW(dashboard_client_->sendRequestString("brake release", "non-existing-response"), UrException);

  dashboard_client_->disconnect();
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
