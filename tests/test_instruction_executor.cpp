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
#include <algorithm>
#include <iostream>
#include <thread>
#include "ur_client_library/ur/instruction_executor.h"
#include "ur_client_library/control/motion_primitives.h"

#include "test_utils.h"

#include <ur_client_library/example_robot_wrapper.h>

using namespace urcl;
using namespace urcl::control;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

class InstructionExecutorTest : public ::testing::Test
{
protected:
  std::unique_ptr<InstructionExecutor> executor_;

  static void SetUpTestSuite()
  {
    if (!(robotVersionLessThan(g_ROBOT_IP, "10.0.0") || g_HEADLESS))
    {
      GTEST_SKIP_("Running URCap tests for PolyScope X is currently not supported.");
    }
    // Setup driver
    g_my_robot = std::make_unique<ExampleRobotWrapper>(g_ROBOT_IP, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS,
                                                       "external_control.urp", SCRIPT_FILE);
  }
  void SetUp() override
  {
    executor_ = std::make_unique<InstructionExecutor>(g_my_robot->getUrDriver());
    g_my_robot->clearProtectiveStop();
    // Make sure script is running on the robot
    if (!g_my_robot->waitForProgramRunning())
    {
      g_my_robot->resendRobotProgram();
      ASSERT_TRUE(g_my_robot->waitForProgramRunning());
    }
  }
  void TearDown() override
  {
    g_my_robot->getUrDriver()->stopControl();
    g_my_robot->waitForProgramNotRunning(1000);
    while (g_my_robot->getUrDriver()->isTrajectoryInterfaceConnected())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    URCL_LOG_INFO("Stopped robot control.");
  }

  static void TearDownTestSuite()
  {
    g_my_robot.reset();
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
    std::make_shared<urcl::control::OptimoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.1, 0.4,
                                                        0.7),
    std::make_shared<urcl::control::OptimoveLPrimitive>(urcl::Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076), 0.1,
                                                        0.4, 0.7),
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
  g_my_robot->getPrimaryClient()->commandStop();
  ASSERT_TRUE(g_my_robot->waitForProgramNotRunning(1000));

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
  g_my_robot->resendRobotProgram();
  ASSERT_TRUE(g_my_robot->waitForProgramRunning(1000));

  // move to first pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }));
  // move to second pose asynchronoysly
  auto motion_thread = std::thread([&]() { ASSERT_FALSE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 })); });

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  g_my_robot->getPrimaryClient()->commandStop();
  ASSERT_TRUE(g_my_robot->waitForProgramNotRunning(1000));
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

TEST_F(InstructionExecutorTest, canceling_without_running_trajectory_returns_false)
{
  ASSERT_FALSE(executor_->isTrajectoryRunning());
  ASSERT_FALSE(executor_->cancelMotion());
}

TEST(InstructionExecutorTestStandalone, canceling_without_receiving_answer_returns_false)
{
  if (!(robotVersionLessThan(g_ROBOT_IP, "10.0.0") || g_HEADLESS))
  {
    GTEST_SKIP_("Running URCap tests for PolyScope X is currently not supported.");
  }

  std::ifstream in_file(SCRIPT_FILE);
  std::ofstream out_file;
  const std::string test_script_file = "test_script.urscript";
  out_file.open(test_script_file);

  std::string line;
  std::string pattern = "socket_send_int(TRAJECTORY_RESULT_CANCELED, \"trajectory_socket\")";
  while (std::getline(in_file, line))
  {
    if (line.find(pattern) == std::string::npos)
    {
      out_file << line << std::endl;
    }
  }
  out_file.close();
  auto my_robot = std::make_unique<ExampleRobotWrapper>(g_ROBOT_IP, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS,
                                                        "external_control.urp", test_script_file);
  auto executor = std::make_unique<InstructionExecutor>(my_robot->getUrDriver());
  my_robot->clearProtectiveStop();
  // Make sure script is running on the robot
  if (!my_robot->waitForProgramRunning())
  {
    my_robot->resendRobotProgram();
    ASSERT_TRUE(my_robot->waitForProgramRunning());
  }
  ASSERT_TRUE(executor->moveJ({ -1.59, -1.72, -2.2, -0.8, 1.6, 0.2 }, 2.0, 2.0));
  std::thread move_thread([&executor]() { executor->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 2.7 }, 0.1, 0.1); });
  bool is_trajectory_running = false;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while (!is_trajectory_running || std::chrono::steady_clock::now() > start + std::chrono::seconds(5))
  {
    is_trajectory_running = executor->isTrajectoryRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(executor->isTrajectoryRunning());
  ASSERT_FALSE(executor->cancelMotion());
  move_thread.join();
  std::remove(test_script_file.c_str());
}

TEST_F(InstructionExecutorTest, canceling_with_running_trajectory_succeeds)
{
  ASSERT_TRUE(executor_->moveJ({ -1.59, -1.72, -2.2, -0.8, 1.6, 0.2 }, 2.0, 2.0));
  std::thread move_thread([this]() { executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 2.7 }); });
  bool is_trajectory_running = false;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while (!is_trajectory_running || std::chrono::steady_clock::now() > start + std::chrono::seconds(5))
  {
    is_trajectory_running = executor_->isTrajectoryRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(executor_->cancelMotion());
  move_thread.join();
}

TEST_F(InstructionExecutorTest, unfeasible_movel_target_results_in_failure)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.59, -1.72, -2.2, -0.8, 1.6, 0.2 }, 2.0, 2.0));

  // move to an unfeasible pose
  std::thread move_thread([this]() { executor_->moveL({ -10.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 1.5, 1.5); });

  std::string safety_status;
  bool is_protective_stopped = false;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while (!is_protective_stopped || std::chrono::steady_clock::now() > start + std::chrono::seconds(5))
  {
    is_protective_stopped = g_my_robot->getPrimaryClient()->isRobotProtectiveStopped();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(is_protective_stopped);
  ASSERT_TRUE(g_my_robot->clearProtectiveStop());
  ASSERT_NO_THROW(g_my_robot->getPrimaryClient()->commandStop());

  move_thread.join();
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

TEST_F(InstructionExecutorTest, movep_succeeds)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  ASSERT_TRUE(executor_->moveP({ -0.203, 0.363, 0.759, 0.68, -1.083, -2.076 }, 1.0, 1.0));
}

TEST_F(InstructionExecutorTest, movec_succeeds)
{
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  ASSERT_TRUE(executor_->moveP({ -0.209, 0.492, 0.5522, 0.928, -1.134, -2.168 }, 1.2, 0.25, 0.025));
  ASSERT_TRUE(executor_->moveC({ -0.209, 0.487, 0.671, 1.026, -0.891, -2.337 },
                               { -0.209, 0.425, 0.841, 1.16, -0.477, -2.553 }, 0.1, 0.25, 0.025, 0));
}

TEST_F(InstructionExecutorTest, optimovej_succeeds)
{
  if (robotVersionLessThan(g_ROBOT_IP, "5.21.0"))
  {
    GTEST_SKIP_("optimoveJ is not supported on robots with a version lower than 5.21.0.");
  }
  else if (!robotVersionLessThan(g_ROBOT_IP, "10.0.0") && robotVersionLessThan(g_ROBOT_IP, "10.8.0"))
  {
    GTEST_SKIP_("optimoveJ is not supported on PolyScope X with a version lower than 10.8.0.");
  }
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  ASSERT_TRUE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.4, 0.7, 0.1));
  ASSERT_TRUE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.1 }, 1.0, 1.0, 0.1));
}

TEST_F(InstructionExecutorTest, optimovel_succeeds)
{
  if (robotVersionLessThan(g_ROBOT_IP, "5.21.0"))
  {
    GTEST_SKIP_("optimoveL is not supported on robots with a version lower than 5.21.0.");
  }
  else if (!robotVersionLessThan(g_ROBOT_IP, "10.0.0") && robotVersionLessThan(g_ROBOT_IP, "10.8.0"))
  {
    GTEST_SKIP_("optimoveL is not supported on PolyScope X with a version lower than 10.8.0.");
  }
  // move to a feasible starting pose
  ASSERT_TRUE(executor_->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }));

  ASSERT_TRUE(executor_->optimoveL({ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.4, 0.7, 0.1));
  ASSERT_TRUE(executor_->optimoveL({ -0.203, 0.463, 0.459, 0.68, -1.083, -2.076 }, 1.0, 1.0, 0.1));
}

TEST_F(InstructionExecutorTest, optimovej_with_illegal_parameters_fails)
{
  // Negative acceleration
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, -0.4, 0.7));
  // Acceleration of 0
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.0, 0.7));
  // Acceleration > 1
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 1.2, 0.7));
  // negative velocity
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.4, -0.7));
  // Velocity of 0
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.4, 0.0));
  // Velocity > 1
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.4, 1.2));
  // Negative blend radius
  ASSERT_FALSE(executor_->optimoveJ({ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.4, 0.7, -0.1));
}

TEST_F(InstructionExecutorTest, optimovel_with_illegal_parameters_fails)
{
  // Negative acceleration
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, -0.4, 0.7));
  // Acceleration of 0
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.0, 0.7));
  // Acceleration > 1
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 1.2, 0.7));
  // negative velocity
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.4, -0.7));
  // Velocity of 0
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.4, 0.0));
  // Velocity > 1
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.4, 1.2));
  // Negative blend radius
  ASSERT_FALSE(executor_->optimoveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 0.4, 0.7, -0.1));
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
    if (std::string(argv[i]) == "--headless" && i + 1 < argc)
    {
      std::string headless = argv[i + 1];
      g_HEADLESS = headless == "true" || headless == "1" || headless == "True" || headless == "TRUE";
      break;
    }
  }

  return RUN_ALL_TESTS();
}