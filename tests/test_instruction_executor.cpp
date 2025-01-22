// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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
#include <thread>
#include "ur_client_library/ur/dashboard_client.h"
#include "ur_client_library/ur/instruction_executor.h"
#include "ur_client_library/control/motion_primitives.h"

using namespace urcl;
using namespace urcl::control;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string ROBOT_IP = "192.168.56.101";

bool g_program_running;
std::condition_variable g_program_not_running_cv;
std::mutex g_program_not_running_mutex;
std::condition_variable g_program_running_cv;
std::mutex g_program_running_mutex;

std::unique_ptr<DashboardClient> g_dashboard_client;
std::shared_ptr<UrDriver> g_ur_driver;

// Helper functions for the driver
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
  if (program_running)
  {
    std::lock_guard<std::mutex> lk(g_program_running_mutex);
    g_program_running = program_running;
    g_program_running_cv.notify_one();
  }
  else
  {
    std::lock_guard<std::mutex> lk(g_program_not_running_mutex);
    g_program_running = program_running;
    g_program_not_running_cv.notify_one();
  }
}

bool waitForProgramNotRunning(int milliseconds = 100)
{
  std::unique_lock<std::mutex> lk(g_program_not_running_mutex);
  if (g_program_not_running_cv.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
      g_program_running == false)
  {
    return true;
  }
  return false;
}

bool waitForProgramRunning(int milliseconds = 100)
{
  std::unique_lock<std::mutex> lk(g_program_running_mutex);
  if (g_program_running_cv.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
      g_program_running == true)
  {
    return true;
  }
  return false;
}

class InstructionExecutorTest : public ::testing::Test
{
protected:
  std::unique_ptr<InstructionExecutor> executor_;

  static void SetUpTestSuite()
  {
    g_dashboard_client.reset(new DashboardClient(ROBOT_IP));
    ASSERT_TRUE(g_dashboard_client->connect());

    // Make robot ready for test
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    g_dashboard_client->setReceiveTimeout(tv);

    // Stop running program if there is one
    ASSERT_TRUE(g_dashboard_client->commandStop());

    // if the robot is not powered on and ready
    ASSERT_TRUE(g_dashboard_client->commandBrakeRelease());

    // Setup driver
    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    const bool headless = true;
    try
    {
      g_ur_driver.reset(new UrDriver(ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                     headless, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
    }
    catch (UrException& exp)
    {
      std::cout << "caught exception " << exp.what() << " while launch driver, retrying once in 10 seconds"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(10));
      g_ur_driver.reset(new UrDriver(ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                     headless, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
    }
  }
  void SetUp() override
  {
    executor_ = std::make_unique<InstructionExecutor>(g_ur_driver);
    std::string safety_status;
    g_dashboard_client->commandSafetyStatus(safety_status);
    bool is_protective_stopped = safety_status.find("PROTECTIVE_STOP") != std::string::npos;
    if (is_protective_stopped)
    {
      // We forced a protective stop above. Some versions require waiting 5 seconds before releasing
      // the protective stop.
      std::this_thread::sleep_for(std::chrono::seconds(5));
      g_dashboard_client->commandCloseSafetyPopup();
      ASSERT_TRUE(g_dashboard_client->commandUnlockProtectiveStop());
    }
    // Make sure script is running on the robot
    if (g_program_running == false)
    {
      g_ur_driver->sendRobotProgram();
      ASSERT_TRUE(waitForProgramRunning(1000));
    }
  }
};

TEST_F(InstructionExecutorTest, execute_motion_sequence_success)
{
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1,
                                                    std::chrono::seconds(5)),
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1,
                                                    std::chrono::seconds(5)),

    std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.1,
                                                    std::chrono::seconds(2)),
    std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1,
                                                    std::chrono::seconds(2)),
  };
  ASSERT_TRUE(executor_->executeMotion(motion_sequence));
}

TEST_F(InstructionExecutorTest, execute_movej_success)
{
  // default parametrization
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }));
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  // acceleration & velocity parametrization
  auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }, 3.4, 3.1, 0));
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // With this parametrization execution should take about 1.4 seconds plus some overhead. We test
  // time parametrization below with a motion time of 2.5 seconds, so this is the upper bound to
  // distinguish between the two. This large range is necessary, as the actual overhead is not
  // known.
  ASSERT_GT(duration.count(), 1400);
  ASSERT_LT(duration.count(), 2500);

  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 3.4, 3.1, 0));
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GT(duration.count(), 1400);
  ASSERT_LT(duration.count(), 2500);

  // time parametrization
  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }, 3.4, 3.1, 3));  // 3 seconds
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GE(duration.count(), 3000);

  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 3.4, 3.1, 2.5));  // 2.5 seconds
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GE(duration.count(), 2500);
}

TEST_F(InstructionExecutorTest, execute_movel_success)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }));
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }));

  // acceleration & velocity parametrization
  auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 2.5, 2.5));
  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // With this parametrization execution should take about 0.6 seconds plus some overhead. We test
  // time parametrization below with a motion time of 2.0 seconds, so this is the upper bound to
  // distinguish between the two. This large range is necessary, as the actual overhead is not
  // known.
  ASSERT_GT(duration.count(), 500);
  ASSERT_LT(duration.count(), 2000);

  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 2.5, 2.5));
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GT(duration.count(), 500);
  ASSERT_LT(duration.count(), 2000);

  // time parametrization
  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 2.5, 2.5, 2.0));
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GT(duration.count(), 2000);

  start = std::chrono::steady_clock::now();
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 2.5, 1.5, 2.2));
  end = std::chrono::steady_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  ASSERT_GT(duration.count(), 2200);
}

TEST_F(InstructionExecutorTest, sending_commands_without_reverse_interface_connected_fails)
{
  g_dashboard_client->commandStop();
  ASSERT_TRUE(waitForProgramNotRunning(1000));

  ASSERT_FALSE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));
  ASSERT_FALSE(executor_->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }));
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1,
                                                    std::chrono::seconds(5)),
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1,
                                                    std::chrono::seconds(5)),

    std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.1,
                                                    std::chrono::seconds(2)),
    std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose{ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1,
                                                    std::chrono::seconds(2)),
  };
  ASSERT_FALSE(executor_->executeMotion(motion_sequence));

  // Disconnect mid-motion
  g_ur_driver->sendRobotProgram();
  ASSERT_TRUE(waitForProgramRunning(1000));

  // move to first pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }));
  // move to second pose asynchronoysly
  auto motion_thread = std::thread([&]() { ASSERT_FALSE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 })); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  g_dashboard_client->commandStop();
  ASSERT_TRUE(waitForProgramNotRunning(1000));
  motion_thread.join();
}

TEST_F(InstructionExecutorTest, sending_illegal_motion_type_fails)
{
  auto primitive = std::make_shared<urcl::control::MotionPrimitive>();
  primitive->type = urcl::control::MotionType::UNKNOWN;
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{ primitive };
  ASSERT_FALSE(executor_->executeMotion(motion_sequence));
}

TEST_F(InstructionExecutorTest, unfeasible_movej_target_results_in_failure)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  // move to an unfeasible pose
  ASSERT_FALSE(executor_->moveJ({ -123, 0, 0, 0, 0, 0 }));
}

TEST_F(InstructionExecutorTest, unfeasible_movel_target_results_in_failure)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  // move to an unfeasible pose
  ASSERT_FALSE(executor_->moveL({ -10.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 1.4, 1.04, 0.1));
}

TEST_F(InstructionExecutorTest, unfeasible_sequence_targets_results_in_failure)
{
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1),
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -157, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1),

  };
  ASSERT_FALSE(executor_->executeMotion(motion_sequence));
}

TEST_F(InstructionExecutorTest, unfeasible_times_succeeds)
{
  // Unfeasible time constraints should simply result in speed scaling slowing down the robot.

  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));
  // Request going to another pose unfeasibly fast
  ASSERT_TRUE(executor_->moveJ({ 1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 1.4, 1.4, 0.1));

  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));
  // Request going to another pose unfeasibly fast
  ASSERT_TRUE(executor_->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 1.4, 1.04, 0.1));
}

TEST_F(InstructionExecutorTest, empty_sequence_succeeds)
{
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{};
  ASSERT_TRUE(executor_->executeMotion(motion_sequence));
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
