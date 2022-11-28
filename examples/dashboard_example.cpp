// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// -- END LICENSE BLOCK ------------------------------------------------

#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>

#include <iostream>
#include <memory>
#include <thread>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

// We need a callback function to register. See UrDriver's parameters for details.

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::DEBUG);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Making the robot ready for the program by:
  // Connect the the robot Dashboard
  std::unique_ptr<DashboardClient> my_dashboard;
  my_dashboard.reset(new DashboardClient(robot_ip));
  if (!my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  if (!my_dashboard->commandPowerOff())
  {
    URCL_LOG_ERROR("Could not send power off");
    return 1;
  }

  my_dashboard->commandCloseSafetyPopup();

  // Power it on
  if (!my_dashboard->commandPowerOn())
  {
    URCL_LOG_ERROR("Could not send Power on command");
    return 1;
  }

  // Release the brakes
  if (!my_dashboard->commandBrakeRelease())
  {
    URCL_LOG_ERROR("Could not send BrakeRelease command");
    return 1;
  }

  // Load existing program
  const std::string program_file_name_to_be_loaded("wait_program.urp");
  if (!my_dashboard->commandLoadProgram(program_file_name_to_be_loaded))
  {
    URCL_LOG_ERROR("Could not load %s program", program_file_name_to_be_loaded.c_str());
    return 1;
  }

  // Play loaded program
  if (!my_dashboard->commandPlay())
  {
    URCL_LOG_ERROR("Could not play program");
    return 1;
  }

  // Pause running program
  if (!my_dashboard->commandPause())
  {
    URCL_LOG_ERROR("Could not pause program");
    return 1;
  }

  // Play loaded program
  if (!my_dashboard->commandPlay())
  {
    URCL_LOG_ERROR("Could not play program");
    return 1;
  }

  // Stop program
  if (!my_dashboard->commandStop())
  {
    URCL_LOG_ERROR("Could not stop program");
    return 1;
  }

  // Power it off
  if (!my_dashboard->commandPowerOff())
  {
    URCL_LOG_ERROR("Could not send Power off command");
    return 1;
  }

  // Now the robot is ready to receive a program

  return 0;
}
