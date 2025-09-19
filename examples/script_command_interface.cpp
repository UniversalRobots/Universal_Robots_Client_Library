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

#include <chrono>
#include <string>
#include "ur_client_library/ur/tool_communication.h"

#include <ur_client_library/log.h>
#include <ur_client_library/example_robot_wrapper.h>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;
bool g_HEADLESS = true;
bool g_running = false;

void sendScriptCommands()
{
  auto run_cmd = [](const std::string& log_output, std::function<void()> func) {
    const std::chrono::seconds timeout(3);
    if (g_running)
    {
      // We wait a fixed time so that not each command is run directly behind each other.
      // This is done for example purposes only, so users can follow the effect on the teach
      // pendant.
      std::this_thread::sleep_for(timeout);
      URCL_LOG_INFO(log_output.c_str());
      func();
    }
  };

  // Keep running all commands in a loop until g_running is set to false
  while (g_running)
  {
    run_cmd("Setting tool voltage to 24V",
            []() { g_my_robot->getUrDriver()->setToolVoltage(urcl::ToolVoltage::_24V); });
    run_cmd("Enabling tool contact mode", []() { g_my_robot->getUrDriver()->startToolContact(); });
    run_cmd("Setting friction_compensation variable to `false`",
            []() { g_my_robot->getUrDriver()->setFrictionCompensation(false); });
    run_cmd("Setting tool voltage to 0V", []() { g_my_robot->getUrDriver()->setToolVoltage(urcl::ToolVoltage::OFF); });
    run_cmd("Zeroing the force torque sensor", []() { g_my_robot->getUrDriver()->zeroFTSensor(); });
    run_cmd("Disabling tool contact mode", []() { g_my_robot->getUrDriver()->endToolContact(); });
    run_cmd("Setting friction_compensation variable to `true`",
            []() { g_my_robot->getUrDriver()->setFrictionCompensation(true); });
  }
  URCL_LOG_INFO("Script command thread finished.");
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

  // Parse whether to run in headless mode
  // When not using headless mode, the global variables can be watched on the teach pendant.
  if (argc > 3)
  {
    g_HEADLESS = std::string(argv[3]) == "true" || std::string(argv[3]) == "1" || std::string(argv[3]) == "True" ||
                 std::string(argv[3]) == "TRUE";
  }

  g_my_robot =
      std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS, "external_control.urp");

  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  // We will send script commands from a separate thread. That will stay active as long as
  // g_running is true.
  g_running = true;
  std::thread script_command_send_thread(sendScriptCommands);

  // We will need to keep the script running on the robot. As we use the "usual" external_control
  // urscript, we'll have to send keepalive signals as long as we want to keep it active.
  std::chrono::duration<double> time_done(0);
  std::chrono::duration<double> timeout(second_to_run);
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;
  while ((time_done < timeout || second_to_run.count() == 0) && g_my_robot->isHealthy())
  {
    g_my_robot->getUrDriver()->writeKeepalive();

    stopwatch_now = std::chrono::steady_clock::now();
    time_done += stopwatch_now - stopwatch_last;
    stopwatch_last = stopwatch_now;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1.0 / g_my_robot->getUrDriver()->getControlFrequency())));
  }

  URCL_LOG_INFO("Timeout reached.");
  g_my_robot->getUrDriver()->stopControl();

  // Stop the script command thread
  g_running = false;
  script_command_send_thread.join();

  return 0;
}
