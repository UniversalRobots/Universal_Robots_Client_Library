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

#include "ur_client_library/exceptions.h"
#include "ur_client_library/log.h"

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/dashboard_client_implementation_g5.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

using namespace urcl;

// This test mocks all the calls from the G5 DashboardClient implementation. It's main purpose is to validate the
// correct connection of all commands between the DashboardClient and its implementation.

const DashboardResponse SUCCESS_RESPONSE{ true, "Success", {} };

class MockDashboardClientImpl : public DashboardClientImplG5
{
public:
  MockDashboardClientImpl(const std::string& host) : DashboardClientImplG5(host)
  {
  }

  bool connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time) override
  {
    return true;  // Simulate a successful connection
  }

  MOCK_METHOD(DashboardResponse, commandAddToLog, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandBrakeRelease, (), (override));
  MOCK_METHOD(DashboardResponse, commandClearOperationalMode, (), (override));
  MOCK_METHOD(DashboardResponse, commandClosePopup, (), (override));
  MOCK_METHOD(DashboardResponse, commandCloseSafetyPopup, (), (override));
  MOCK_METHOD(DashboardResponse, commandGenerateFlightReport, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandGenerateSupportFile, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandGetLoadedProgram, (), (override));
  MOCK_METHOD(DashboardResponse, commandGetOperationalMode, (), (override));
  MOCK_METHOD(DashboardResponse, commandGetRobotModel, (), (override));
  MOCK_METHOD(DashboardResponse, commandGetSerialNumber, (), (override));
  MOCK_METHOD(DashboardResponse, commandGetUserRole, (), (override));
  MOCK_METHOD(DashboardResponse, commandIsInRemoteControl, (), (override));
  MOCK_METHOD(DashboardResponse, commandIsProgramSaved, (), (override));
  MOCK_METHOD(DashboardResponse, commandLoadInstallation, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandLoadProgram, (const std::string& program_name), (override));
  MOCK_METHOD(DashboardResponse, commandPause, (), (override));
  MOCK_METHOD(DashboardResponse, commandPlay, (), (override));
  MOCK_METHOD(DashboardResponse, commandPolyscopeVersion, (), (override));
  MOCK_METHOD(DashboardResponse, commandPopup, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandPowerOff, (), (override));
  MOCK_METHOD(DashboardResponse, commandPowerOn, (const std::chrono::duration<double> timeout), (override));
  MOCK_METHOD(DashboardResponse, commandProgramState, (), (override));
  MOCK_METHOD(DashboardResponse, commandQuit, (), (override));
  MOCK_METHOD(DashboardResponse, commandRestartSafety, (), (override));
  MOCK_METHOD(DashboardResponse, commandRobotMode, (), (override));
  MOCK_METHOD(DashboardResponse, commandRunning, (), (override));
  MOCK_METHOD(DashboardResponse, commandSafetyMode, (), (override));
  MOCK_METHOD(DashboardResponse, commandSafetyStatus, (), (override));
  MOCK_METHOD(DashboardResponse, commandSaveLog, (), (override));
  MOCK_METHOD(DashboardResponse, commandSetOperationalMode, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandSetUserRole, (const std::string&), (override));
  MOCK_METHOD(DashboardResponse, commandShutdown, (), (override));
  MOCK_METHOD(DashboardResponse, commandStop, (), (override));
  MOCK_METHOD(DashboardResponse, commandUnlockProtectiveStop, (), (override));

  void setPolyscopeVersion(const std::string& version)
  {
    polyscope_version_ = VersionInformation::fromString(version);
  }
};

class TestableDashboardClient : public DashboardClient
{
public:
  TestableDashboardClient(const std::string& host) : DashboardClient(host, DashboardClient::ClientPolicy::G5)
  {
    impl_ = std::make_shared<MockDashboardClientImpl>(host);
  }

  std::shared_ptr<MockDashboardClientImpl> getImplPtr()
  {
    return std::static_pointer_cast<MockDashboardClientImpl>(impl_);
  }
};

class DashboardClientTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    dashboard_client_.reset(new TestableDashboardClient("1.2.3.4"));
  }

  void TearDown()
  {
    dashboard_client_.reset();
  }

  std::unique_ptr<TestableDashboardClient> dashboard_client_;
};

TEST_F(DashboardClientTest, connect)
{
  EXPECT_TRUE(dashboard_client_->connect());
  auto mine = dashboard_client_->getImplPtr();
  EXPECT_CALL(*mine, commandCloseSafetyPopup()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_TRUE(dashboard_client_->commandCloseSafetyPopup());
}

TEST_F(DashboardClientTest, run_program)
{
  EXPECT_TRUE(dashboard_client_->connect());

  const auto impl = dashboard_client_->getImplPtr();

  EXPECT_CALL(*impl, commandLoadProgram("wait_program.urp")).WillOnce([](const std::string& str) {
    return DashboardResponse{ true, "Program loaded: " + str, { { "program_name", str } } };
  });
  EXPECT_CALL(*impl, commandPowerOn(std::chrono::duration<double>(1))).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandPlay()).Times(2).WillRepeatedly(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandPowerOff()).Times(2).WillRepeatedly(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandBrakeRelease()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandPause()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandRunning())
      .Times(2)
      .WillOnce([]() {
        DashboardResponse response;
        response.ok = true;
        response.data["running"] = true;  // Simulate that the program is running
        return response;
      })
      .WillOnce([]() {
        DashboardResponse response;
        response.ok = true;
        response.data["running"] = false;  // Simulate that the program is not running
        return response;
      });
  EXPECT_CALL(*impl, commandStop()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandIsProgramSaved())
      .WillOnce(testing::Return(DashboardResponse{ true, "true", { { "saved", true } } }));
  EXPECT_CALL(*impl, commandQuit()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandShutdown()).WillOnce(testing::Return(SUCCESS_RESPONSE));

  EXPECT_TRUE(dashboard_client_->commandLoadProgram("wait_program.urp"));
  EXPECT_TRUE(dashboard_client_->commandPowerOff());
  EXPECT_TRUE(dashboard_client_->commandPowerOn(std::chrono::duration<double>(1)));
  EXPECT_TRUE(dashboard_client_->commandBrakeRelease());
  EXPECT_TRUE(dashboard_client_->commandPlay());
  EXPECT_TRUE(dashboard_client_->commandPause());
  EXPECT_TRUE(dashboard_client_->commandPlay());
  EXPECT_TRUE(dashboard_client_->commandRunning());
  EXPECT_TRUE(dashboard_client_->commandStop());
  EXPECT_FALSE(dashboard_client_->commandRunning());
  EXPECT_TRUE(dashboard_client_->commandPowerOff());
  EXPECT_TRUE(dashboard_client_->commandIsProgramSaved());
  EXPECT_TRUE(dashboard_client_->commandQuit());
  EXPECT_TRUE(dashboard_client_->commandShutdown());
}

TEST_F(DashboardClientTest, load_installation)
{
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandLoadInstallation("default.installation")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_TRUE(dashboard_client_->commandLoadInstallation("default.installation"));
}

TEST_F(DashboardClientTest, popup)
{
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandPopup("Test Popup")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandClosePopup()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_TRUE(dashboard_client_->commandPopup("Test Popup"));
  EXPECT_TRUE(dashboard_client_->commandClosePopup());
}

TEST_F(DashboardClientTest, log_and_getters)
{
  std::string msg;
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandAddToLog("Testing Log:")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandPolyscopeVersion()).WillOnce(testing::Return(DashboardResponse{ true, "5.6.0" }));
  EXPECT_CALL(*impl, commandRobotMode()).WillOnce(testing::Return(DashboardResponse{ true, "Robotmode: POWER_OFF" }));
  EXPECT_CALL(*impl, commandGetLoadedProgram())
      .WillOnce(testing::Return(DashboardResponse{ true, "Loaded program: wait_program.urp" }));
  EXPECT_CALL(*impl, commandProgramState())
      .WillOnce(testing::Return(DashboardResponse{ true, "STOPPED wait_program.urp" }));
  EXPECT_CALL(*impl, commandSaveLog()).WillOnce(testing::Return(SUCCESS_RESPONSE));

  EXPECT_TRUE(dashboard_client_->commandAddToLog("Testing Log:"));
  EXPECT_TRUE(dashboard_client_->commandPolyscopeVersion(msg));
  EXPECT_TRUE(dashboard_client_->commandRobotMode(msg));
  EXPECT_TRUE(dashboard_client_->commandGetLoadedProgram(msg));
  EXPECT_TRUE(dashboard_client_->commandProgramState(msg));
  EXPECT_TRUE(dashboard_client_->commandSaveLog());
}

TEST_F(DashboardClientTest, flight_report_and_support_file)
{
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandGenerateFlightReport("")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandGenerateSupportFile(".")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_TRUE(dashboard_client_->commandGenerateFlightReport(""));
  EXPECT_TRUE(dashboard_client_->commandGenerateSupportFile("."));
}

TEST_F(DashboardClientTest, version_specific_calls)
{
  // Since we mock everything, we can call all version-specific calls in this test.
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandSafetyStatus())
      .WillOnce(testing::Return(DashboardResponse{ true, "Safety status: NORMAL" }));
  EXPECT_CALL(*impl, commandSetUserRole("PROGRAMMER")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandGetUserRole()).WillOnce(testing::Return(DashboardResponse{ true, "PROGRAMMER" }));

  std::string msg, user_role;
  EXPECT_TRUE(dashboard_client_->commandSafetyStatus(msg));
  EXPECT_TRUE(dashboard_client_->commandSetUserRole("PROGRAMMER"));
  EXPECT_TRUE(dashboard_client_->commandGetUserRole(user_role));
}

TEST_F(DashboardClientTest, safety_mode_and_protective_stop)
{
  EXPECT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandSafetyMode())
      .WillOnce(testing::Return(DashboardResponse{ true, "Safetymode: NORMAL", { { "safety_mode", "NORMAL" } } }));
  EXPECT_CALL(*impl, commandUnlockProtectiveStop()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandCloseSafetyPopup()).WillOnce(testing::Return(SUCCESS_RESPONSE));

  std::string safety_mode;
  EXPECT_TRUE(dashboard_client_->commandSafetyMode(safety_mode));
  EXPECT_EQ(safety_mode, "NORMAL");
  EXPECT_TRUE(dashboard_client_->commandUnlockProtectiveStop());
  EXPECT_TRUE(dashboard_client_->commandCloseSafetyPopup());
}

TEST_F(DashboardClientTest, restart_safety)
{
  ASSERT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();
  EXPECT_CALL(*impl, commandRestartSafety()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  ASSERT_TRUE(dashboard_client_->commandRestartSafety());
}

TEST_F(DashboardClientTest, operational_mode)
{
  ASSERT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();

  EXPECT_CALL(*impl, commandSetOperationalMode("AUTOMATIC")).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandGetOperationalMode()).WillOnce(testing::Return(DashboardResponse{ true, "AUTOMATIC" }));
  EXPECT_CALL(*impl, commandClearOperationalMode()).WillOnce(testing::Return(SUCCESS_RESPONSE));
  EXPECT_CALL(*impl, commandIsInRemoteControl())
      .WillOnce(testing::Return(DashboardResponse{ true, "false", { { "remote_control", false } } }));

  std::string operational_mode;
  ASSERT_TRUE(dashboard_client_->commandSetOperationalMode("AUTOMATIC"));
  EXPECT_TRUE(dashboard_client_->commandGetOperationalMode(operational_mode));
  ASSERT_TRUE(dashboard_client_->commandClearOperationalMode());
  EXPECT_FALSE(dashboard_client_->commandIsInRemoteControl());
}

TEST_F(DashboardClientTest, get_robot_info)
{
  ASSERT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();

  EXPECT_CALL(*impl, commandGetRobotModel())
      .WillOnce(testing::Return(DashboardResponse{ true, "UR10e", { { "robot_model", "UR10e" } } }));
  EXPECT_CALL(*impl, commandGetSerialNumber())
      .WillOnce(testing::Return(DashboardResponse{ true, "20235212345", { { "serial_number", "20235212345" } } }));

  std::string robot_model;
  std::string serial_number;
  ASSERT_TRUE(dashboard_client_->commandGetRobotModel(robot_model));
  ASSERT_TRUE(dashboard_client_->commandGetSerialNumber(serial_number));
}

TEST_F(DashboardClientTest, assert_version)
{
  ASSERT_TRUE(dashboard_client_->connect());
  const auto impl = dashboard_client_->getImplPtr();

  // Set a valid version
  impl->setPolyscopeVersion("5.6.0");
  EXPECT_NO_THROW(dashboard_client_->assertVersion("5.2.0", "3.15.1", "foobar"));
  impl->setPolyscopeVersion("5.12.0");
  EXPECT_NO_THROW(dashboard_client_->assertVersion("5.2.0", "3.15.1", "foobar"));
  EXPECT_THROW(dashboard_client_->assertVersion("-", "3.15.1", "foobar"), UrException);
  impl->setPolyscopeVersion("5.1.0");
  EXPECT_THROW(dashboard_client_->assertVersion("5.2.0", "3.15.1", "foobar"), UrException);
  impl->setPolyscopeVersion("3.5.0");
  EXPECT_THROW(dashboard_client_->assertVersion("5.2.0", "3.15.1", "foobar"), UrException);
  EXPECT_THROW(dashboard_client_->assertVersion("5.2.0", "-", "foobar"), UrException);
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
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  urcl::setLogLevel(urcl::LogLevel::DEBUG);

  return RUN_ALL_TESTS();
}
