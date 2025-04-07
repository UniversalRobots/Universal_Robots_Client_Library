// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

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

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options

#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <iostream>
#include <memory>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;
std::atomic<bool> g_tool_contact_result_triggered;
control::ToolContactResult g_tool_contact_result;

void handleToolContactResult(control::ToolContactResult result)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mTool contact result: " << toUnderlying(result) << "\033[0m\n" << std::endl;
  g_tool_contact_result = result;
  g_tool_contact_result_triggered = true;
}

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);
  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Parse how many seconds to run
  auto second_to_run = std::chrono::seconds(0);
  if (argc > 2)
  {
    second_to_run = std::chrono::seconds(std::stoi(argv[2]));
  }

  bool headless_mode = true;
  g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                     "external_control.urp");
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }
  g_my_robot->getUrDriver()->registerToolContactResultCallback(&handleToolContactResult);
  g_my_robot->getUrDriver()->startToolContact();

  // This will move the robot downward in the z direction of the base until a tool contact is detected or seconds_to_run
  // is reached
  const vector6d_t tcp_speed = { 0.0, 0.0, -0.02, 0.0, 0.0, 0.0 };
  auto start_time = std::chrono::system_clock::now();
  while (second_to_run.count() < 0 || (std::chrono::system_clock::now() - start_time) < second_to_run)
  {
    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(tcp_speed, comm::ControlMode::MODE_SPEEDL,
                                                            RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
      return 1;
    }

    if (g_tool_contact_result_triggered)
    {
      URCL_LOG_INFO("Tool contact detected");
      break;
    }
  }
  URCL_LOG_INFO("Timed out before reaching tool contact.");
  g_my_robot->getUrDriver()->stopControl();
  return 0;
}
