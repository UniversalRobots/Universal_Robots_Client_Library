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
#include <memory>
#include "ur_client_library/ur/ur_driver.h"

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

void handleRobotProgramState(bool program_running)
{
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

TEST(UrDriverTestDeprecatedConstructor, sigA)
{
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  auto driver = std::make_shared<urcl::UrDriver>(g_ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE,
                                                 std::bind(&handleRobotProgramState, std::placeholders::_1), g_HEADLESS,
                                                 std::move(tool_comm_setup));
  driver->checkCalibration(CALIBRATION_CHECKSUM);
  auto version = driver->getVersion();
  ASSERT_TRUE(version.major > 0);
}

TEST(UrDriverTestDeprecatedConstructor, sigB)
{
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  auto driver = std::make_shared<urcl::UrDriver>(
      g_ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, std::bind(&handleRobotProgramState, std::placeholders::_1),
      g_HEADLESS, std::move(tool_comm_setup), 50001, 50002, 2000, 0.03, false, "", 50003, 50004, 0.025, 0.5);
  driver->checkCalibration(CALIBRATION_CHECKSUM);
  auto version = driver->getVersion();
  ASSERT_TRUE(version.major > 0);
}

TEST(UrDriverTestDeprecatedConstructor, sigC)
{
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  auto driver = std::make_shared<urcl::UrDriver>(g_ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE,
                                                 std::bind(&handleRobotProgramState, std::placeholders::_1), g_HEADLESS,
                                                 std::move(tool_comm_setup), CALIBRATION_CHECKSUM);
  auto version = driver->getVersion();
  ASSERT_TRUE(version.major > 0);
}

TEST(UrDriverTestDeprecatedConstructor, sigD)
{
  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  auto driver = std::make_shared<urcl::UrDriver>(g_ROBOT_IP, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE,
                                                 std::bind(&handleRobotProgramState, std::placeholders::_1), g_HEADLESS,
                                                 CALIBRATION_CHECKSUM);
  auto version = driver->getVersion();
  ASSERT_TRUE(version.major > 0);
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
