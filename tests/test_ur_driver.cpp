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
#define private public
#include <ur_client_library/ur/ur_driver.h>

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string ROBOT_IP = "192.168.56.101";

std::unique_ptr<UrDriver> g_ur_driver_;
std::unique_ptr<DashboardClient> g_dashboard_client_;

bool g_program_running;
std::condition_variable g_program_not_running_cv_;
std::mutex g_program_not_running_mutex_;
std::condition_variable g_program_running_cv_;
std::mutex g_program_running_mutex_;

// Helper functions for the driver
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
  if (program_running)
  {
    std::lock_guard<std::mutex> lk(g_program_running_mutex_);
    g_program_running = program_running;
    g_program_running_cv_.notify_one();
  }
  else
  {
    std::lock_guard<std::mutex> lk(g_program_not_running_mutex_);
    g_program_running = program_running;
    g_program_not_running_cv_.notify_one();
  }
}

class UrDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    g_dashboard_client_.reset(new DashboardClient(ROBOT_IP));
    ASSERT_TRUE(g_dashboard_client_->connect());

    // Make robot ready for test
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    g_dashboard_client_->setReceiveTimeout(tv);

    // Stop running program if there is one
    ASSERT_TRUE(g_dashboard_client_->commandStop());

    // if the robot is not powered on and ready
    ASSERT_TRUE(g_dashboard_client_->commandBrakeRelease());

    // Setup driver
    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    const bool HEADLESS = true;
    try
    {
      g_ur_driver_.reset(new UrDriver(ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                      HEADLESS, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
    }
    catch (UrException& exp)
    {
      std::cout << "caught exception " << exp.what() << " while launch driver, retrying once in 10 seconds"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(10));
      g_ur_driver_.reset(new UrDriver(ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                      HEADLESS, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
    }
  }

  void setUp()
  {
    g_ur_driver_->stopControl();
    waitForProgramNotRunning(1000);
  }

  static void TearDownTestSuite()
  {
    g_dashboard_client_->disconnect();
  }

  void readDataPackage(std::unique_ptr<rtde_interface::DataPackage>& data_pkg)
  {
    data_pkg = g_ur_driver_->getDataPackage();
    if (data_pkg == nullptr)
    {
      std::cout << "Failed to get data package from robot" << std::endl;
      GTEST_FAIL();
    }
  }

  bool waitForProgramRunning(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(g_program_running_mutex_);
    if (g_program_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        g_program_running == true)
    {
      return true;
    }
    return false;
  }

  bool waitForProgramNotRunning(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(g_program_not_running_mutex_);
    if (g_program_not_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        g_program_running == false)
    {
      return true;
    }
    return false;
  }
};

TEST_F(UrDriverTest, read_non_existing_script_file)
{
  const std::string non_existing_script_file = "";
  EXPECT_THROW(UrDriver::readScriptFile(non_existing_script_file), UrException);
}

TEST_F(UrDriverTest, read_existing_script_file)
{
  char existing_script_file[] = "urscript.XXXXXX";
  int fd = mkstemp(existing_script_file);
  if (fd == -1)
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  EXPECT_NO_THROW(UrDriver::readScriptFile(existing_script_file));

  // clean up
  close(fd);
  unlink(existing_script_file);
}

TEST_F(UrDriverTest, robot_receive_timeout)
{
  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_ur_driver_->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(waitForProgramNotRunning(400));

  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_ur_driver_->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP,
                                             RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(waitForProgramNotRunning(400));

  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_ur_driver_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(waitForProgramNotRunning(400));

  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  // Robot program should time out after the robot receive timeout, whether it takes exactly 200 ms is not so important
  g_ur_driver_->writeKeepalive(RobotReceiveTimeout::millisec(200));
  EXPECT_TRUE(waitForProgramNotRunning(400));
}

TEST_F(UrDriverTest, robot_receive_timeout_off)
{
  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  // Program should keep running when setting receive timeout off
  g_ur_driver_->writeKeepalive(RobotReceiveTimeout::off());
  EXPECT_FALSE(waitForProgramNotRunning(1000));

  // Program should keep running when setting receive timeout off
  g_ur_driver_->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP,
                                             RobotReceiveTimeout::off());
  EXPECT_FALSE(waitForProgramNotRunning(1000));

  // Program should keep running when setting receive timeout off
  g_ur_driver_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::off());
  EXPECT_FALSE(waitForProgramNotRunning(1000));

  // It shouldn't be possible to set robot receive timeout off, when dealing with realtime commands
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_ur_driver_->writeJointCommand(zeros, comm::ControlMode::MODE_SPEEDJ, RobotReceiveTimeout::off());
  EXPECT_TRUE(waitForProgramNotRunning(400));
}

TEST_F(UrDriverTest, stop_robot_control)
{
  // Start robot program
  g_ur_driver_->sendRobotProgram();
  EXPECT_TRUE(waitForProgramRunning(1000));

  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_ur_driver_->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::off());

  // Make sure that we can stop the robot control, when robot receive timeout has been set off
  g_ur_driver_->stopControl();
  EXPECT_TRUE(waitForProgramNotRunning(400));
}

// TODO we should add more tests for the UrDriver class.

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
