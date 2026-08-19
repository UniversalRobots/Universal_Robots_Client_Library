// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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
#include <iostream>
#include <thread>
#include "ur_client_library/ur/instruction_executor.h"

#include "test_utils.h"

#include <ur_client_library/example_robot_wrapper.h>

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

/*
    This test was moved to its own file, as the ExampleRobotWrapper initialisation in this test was clashing with the
    static one initialised in the InstructionExecutorTest test suite. This was causing flaky test behavior in the CI.
    Other Instruction Executor tests that require a separate robot wrapper should also be put here.
*/
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
  while (!is_trajectory_running && std::chrono::steady_clock::now() < start + std::chrono::seconds(5))
  {
    is_trajectory_running = executor->isTrajectoryRunning();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(executor->isTrajectoryRunning());
  EXPECT_FALSE(executor->cancelMotion());
  move_thread.join();
  std::remove(test_script_file.c_str());
}

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
    if (std::string(argv[i]) == "--headless" && i + 1 < argc)
    {
      std::string headless = argv[i + 1];
      g_HEADLESS = headless == "true" || headless == "1" || headless == "True" || headless == "TRUE";
      ++i;
    }
  }

  return RUN_ALL_TESTS();
}