// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2023 Universal Robots A/S
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

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/example_robot_wrapper.h>
#include "test_utils.h"

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

class UrDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!(robotVersionLessThan(g_ROBOT_IP, "10.0.0") || g_HEADLESS))
    {
      GTEST_SKIP_("Running URCap tests for PolyScope X is currently not supported.");
    }
    // Setup driver
    g_my_robot = std::make_unique<ExampleRobotWrapper>(g_ROBOT_IP, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS,
                                                       "external_control.urp", SCRIPT_FILE);

    g_my_robot->startRTDECommununication(true);
  }

  void SetUp()
  {
    if (!g_my_robot->isHealthy())
    {
      ASSERT_TRUE(g_my_robot->resendRobotProgram());
      ASSERT_TRUE(g_my_robot->waitForProgramRunning(500));
    }
  }

  void TearDown()
  {
    g_my_robot->getUrDriver()->stopControl();
    g_my_robot->waitForProgramNotRunning(1000);
  }
};

TEST_F(UrDriverTest, read_non_existing_script_file)
{
  const std::string non_existing_script_file = "";
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_THROW(UrDriver::readScriptFile(non_existing_script_file), UrException);
#pragma GCC diagnostic pop
}

TEST_F(UrDriverTest, read_existing_script_file)
{
  char existing_script_file[] = "urscript.XXXXXX";
#ifdef _WIN32
#  define mkstemp _mktemp_s
#endif
  std::ignore = mkstemp(existing_script_file);

  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_NO_THROW(UrDriver::readScriptFile(existing_script_file));
#pragma GCC diagnostic pop

  // clean up
  ofs.close();
  std::remove(existing_script_file);
}

TEST_F(UrDriverTest, robot_receive_timeout)
{
  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));

  // Start robot program
  g_my_robot->resendRobotProgram();
  EXPECT_TRUE(g_my_robot->waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_my_robot->getUrDriver()->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP,
                                                          RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));

  // Start robot program
  g_my_robot->resendRobotProgram();
  EXPECT_TRUE(g_my_robot->waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                                           RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));

  // Start robot program
  g_my_robot->resendRobotProgram();
  EXPECT_TRUE(g_my_robot->waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_my_robot->getUrDriver()->writeKeepalive(RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));
}

TEST_F(UrDriverTest, robot_receive_timeout_off)
{
  // Program should keep running when setting receive timeout off
  g_my_robot->getUrDriver()->writeKeepalive(RobotReceiveTimeout::off());
  EXPECT_FALSE(g_my_robot->waitForProgramNotRunning(1000));

  // Program should keep running when setting receive timeout off
  g_my_robot->getUrDriver()->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP,
                                                          RobotReceiveTimeout::off());
  EXPECT_FALSE(g_my_robot->waitForProgramNotRunning(1000));

  // Program should keep running when setting receive timeout off
  g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                                           RobotReceiveTimeout::off());
  EXPECT_FALSE(g_my_robot->waitForProgramNotRunning(1000));

  // It shouldn't be possible to set robot receive timeout off, when dealing with realtime commands
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_SPEEDJ, RobotReceiveTimeout::off());
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));
}

TEST_F(UrDriverTest, stop_robot_control)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::off());

  // Make sure that we can stop the robot control, when robot receive timeout has been set off
  g_my_robot->getUrDriver()->stopControl();
  EXPECT_TRUE(g_my_robot->waitForProgramNotRunning(400));
}

TEST_F(UrDriverTest, target_outside_limits_servoj)
{
  g_my_robot->stopConsumingRTDEData();
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  g_my_robot->readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions_before;
  ASSERT_TRUE(data_pkg->getData("actual_q", joint_positions_before));

  // Create physically unfeasible target
  urcl::vector6d_t joint_target = joint_positions_before;
  joint_target[5] -= 2.5;

  // Send unfeasible targets to the robot
  g_my_robot->readDataPackage(data_pkg);
  g_my_robot->getUrDriver()->writeJointCommand(joint_target, comm::ControlMode::MODE_SERVOJ,
                                               RobotReceiveTimeout::millisec(200));

  // Ensure that the robot didn't move
  g_my_robot->readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions;
  ASSERT_TRUE(data_pkg->getData("actual_q", joint_positions));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_positions_before[i], joint_positions[i]);
  }

  // Make sure the program is stopped
  g_my_robot->startConsumingRTDEData();
  g_my_robot->getUrDriver()->stopControl();
  g_my_robot->waitForProgramNotRunning(1000);
}

TEST_F(UrDriverTest, target_outside_limits_pose)
{
  g_my_robot->stopConsumingRTDEData();
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  g_my_robot->readDataPackage(data_pkg);

  urcl::vector6d_t tcp_pose_before;
  ASSERT_TRUE(data_pkg->getData("actual_TCP_pose", tcp_pose_before));

  // Create physically unfeasible target
  urcl::vector6d_t tcp_target = tcp_pose_before;
  tcp_target[2] += 0.3;

  // Send unfeasible targets to the robot
  g_my_robot->readDataPackage(data_pkg);
  g_my_robot->getUrDriver()->writeJointCommand(tcp_target, comm::ControlMode::MODE_POSE,
                                               RobotReceiveTimeout::millisec(200));

  // Ensure that the robot didn't move
  g_my_robot->readDataPackage(data_pkg);
  urcl::vector6d_t tcp_pose;
  ASSERT_TRUE(data_pkg->getData("actual_TCP_pose", tcp_pose));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(tcp_pose_before[i], tcp_pose[i]);
  }

  // Make sure the program is stopped
  g_my_robot->startConsumingRTDEData();
  g_my_robot->getUrDriver()->stopControl();
  g_my_robot->waitForProgramNotRunning(1000);
}

TEST_F(UrDriverTest, send_robot_program_retry_on_failure)
{
  // Check that sendRobotProgram is robust to the primary stream being disconnected. This is what happens when
  // switching from Remote to Local and back to Remote mode for example.

  // To be able to re-send the robot program we'll have to make sure it isn't running
  g_my_robot->getUrDriver()->stopControl();
  g_my_robot->waitForProgramNotRunning();

  g_my_robot->getUrDriver()->stopPrimaryClientCommunication();

  EXPECT_TRUE(g_my_robot->resendRobotProgram());

  EXPECT_TRUE(g_my_robot->waitForProgramRunning(1000));
}

TEST_F(UrDriverTest, reset_rtde_client)
{
  g_my_robot->stopConsumingRTDEData();
  double target_frequency = 50;
  g_my_robot->getUrDriver()->resetRTDEClient(OUTPUT_RECIPE, INPUT_RECIPE, target_frequency);
  ASSERT_EQ(g_my_robot->getUrDriver()->getControlFrequency(), target_frequency);
}

TEST_F(UrDriverTest, read_error_code)
{
  // Wait until we actually received a package
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::stringstream cmd;
  cmd << "sec setup():" << std::endl << " protective_stop()" << std::endl << "end";
  EXPECT_TRUE(g_my_robot->getUrDriver()->sendScript(cmd.str()));

  auto error_codes = g_my_robot->getUrDriver()->getErrorCodes();
  while (error_codes.size() == 0)
  {
    error_codes = g_my_robot->getUrDriver()->getErrorCodes();
  }

  ASSERT_EQ(error_codes.size(), 1);
  // Check whether it is a "A protective stop was triggered"
  // https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/prod-err-codes/topics/CODE_209.html
  ASSERT_EQ(error_codes.at(0).message_code, 209);
  ASSERT_EQ(error_codes.at(0).message_argument, 0);

  // Wait for after PSTOP before clearing it
  std::this_thread::sleep_for(std::chrono::seconds(6));

  if (g_my_robot->getDashboardClient() != nullptr)
  {
    EXPECT_TRUE(g_my_robot->getDashboardClient()->commandCloseSafetyPopup());
  }
  EXPECT_NO_THROW(g_my_robot->getPrimaryClient()->commandUnlockProtectiveStop());
}

TEST(UrDriverInitTest, setting_connection_limits_works_correctly)
{
  UrDriverConfiguration config;
  config.socket_reconnect_attempts = 1;
  config.socket_reconnection_timeout = std::chrono::milliseconds(200);
  config.robot_ip = "192.168.56.100";  // That IP address should not exist on the test network
  config.input_recipe_file = INPUT_RECIPE;
  config.output_recipe_file = OUTPUT_RECIPE;
  config.headless_mode = g_HEADLESS;

  EXPECT_THROW(UrDriver ur_driver(config), UrException);
}

// TODO we should add more tests for the UrDriver class.

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
    if (std::string(argv[i]) == "--headless" && i + 1 < argc)
    {
      std::string headless = argv[i + 1];
      g_HEADLESS = headless == "true" || headless == "1" || headless == "True" || headless == "TRUE";
      break;
    }
  }

  return RUN_ALL_TESTS();
}