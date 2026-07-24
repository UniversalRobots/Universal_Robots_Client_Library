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

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <ur_client_library/example_robot_wrapper.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include "ur_client_library/control/reverse_interface.h"

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

void runFreedrive(UrDriver& urdriver, std::chrono::seconds duration)
{
  URCL_LOG_INFO("Starting freedrive mode");

  urdriver.writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_START);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < duration || duration.count() == 0)
  {
    // Keeping the robot in freedrive by sending NOOP messages
    urdriver.writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  urdriver.writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP);
  URCL_LOG_INFO("Stopping freedrive mode");
}

void runConstrainedFreedrive(UrDriver& urdriver, std::array<int32_t, 6>& free_axes, std::array<double, 6>& feature_pose,
                             std::chrono::seconds duration)
{
  URCL_LOG_INFO("Starting constrained freedrive mode");

  urdriver.writeConstrainedFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_START, free_axes,
                                                   feature_pose);

  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < duration || duration.count() == 0)
  {
    // Keeping the robot in constrained freedrive by sending NOOP messages
    urdriver.writeConstrainedFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_NOOP, free_axes,
                                                     feature_pose);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  urdriver.writeConstrainedFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP, free_axes,
                                                   feature_pose);
  URCL_LOG_INFO("Stopping constrained freedrive mode");
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

  // Select the freedrive mode
  bool constrained = false;
  if (argc > 2)
  {
    std::string arg = argv[2];
    constrained = (arg == "true" || arg == "1");
  }

  // Parse how many seconds to run
  auto second_to_run = std::chrono::seconds(0);
  if (argc > 3)
  {
    second_to_run = std::chrono::seconds(std::stoi(argv[3]));
  }

  std::array<int32_t, 6> free_axes = { 1, 0, 1, 1, 0, 1 };
  std::array<double, 6> feature_pose = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Initialize robot connection
  bool headless_mode = true;
  g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                     "external_control.urp");

  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  // Execute selected freedrive mode
  if (constrained)
  {
    runConstrainedFreedrive(*g_my_robot->getUrDriver(), free_axes, feature_pose, second_to_run);
  }
  else
  {
    runFreedrive(*g_my_robot->getUrDriver(), second_to_run);
  }
}
