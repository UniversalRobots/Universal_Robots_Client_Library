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

#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/primary/primary_client.h"

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

// We need a callback function to register. See UrDriver's parameters for details.

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Query the robot information from the primary interface in order to select the correct
  // implementation policy.
  urcl::comm::INotifier notifier;
  urcl::primary_interface::PrimaryClient primary_client(robot_ip, notifier);
  primary_client.start();
  auto version_information = primary_client.getRobotVersion();
  DashboardClient::ClientPolicy policy = DashboardClient::ClientPolicy::POLYSCOPE_X;
  if (version_information->major < 10)
  {
    policy = DashboardClient::ClientPolicy::G5;
  }
  else if (version_information->minor < 11)
  {
    URCL_LOG_ERROR("DashboardClient examples require PolyScope version 10.11.0 or higher. Exiting now.");
    return 0;
  }

  // Connect to the robot Dashboard Server
  auto my_dashboard = std::make_unique<DashboardClient>(robot_ip, policy);
  if (!my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // Bring the robot to a defined state being powered off.
  if (version_information->major < 10)
  {
    if (!my_dashboard->commandPowerOff())
    {
      URCL_LOG_ERROR("Could not send power off");
      return 1;
    }
    // Get the PolyScope version
    std::string version;
    my_dashboard->commandPolyscopeVersion(version);
    URCL_LOG_INFO(version.c_str());

    my_dashboard->commandCloseSafetyPopup();
  }
  else
  {
    // We're ignoring errors here since
    // powering off an already powered off robot will return an error.
    my_dashboard->commandPowerOff();
  }

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
  std::string program_file_name_to_be_loaded("wait_program.urp");
  if (version_information->major >= 10)
  {
    // For PolyScope X, the program doesn't have an ending
    program_file_name_to_be_loaded = "wait_program";
  }
  if (!my_dashboard->commandLoadProgram(program_file_name_to_be_loaded))
  {
    URCL_LOG_ERROR("Could not load %s program", program_file_name_to_be_loaded.c_str());
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

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

  // Continue
  if (version_information->major >= 10)
  {
    // For PolyScope X, the command is called "resume"
    if (!my_dashboard->commandResume())
    {
      URCL_LOG_ERROR("Could not resume program");
      return 1;
    }
  }
  else
  {
    // For e-Series, the command is called "play"
    if (!my_dashboard->commandPlay())
    {
      URCL_LOG_ERROR("Could not resume program");
      return 1;
    }
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

  if (version_information->major < 10)
  {
    // Flush the log
    if (!my_dashboard->commandSaveLog())
    {
      URCL_LOG_ERROR("Could not send the save log command");
      return 1;
    }

    // Make a raw request and save the response
    std::string program_state = my_dashboard->sendAndReceive("programState");
    URCL_LOG_INFO("Program state: %s", program_state.c_str());

    // The response can be checked with a regular expression
    bool success = my_dashboard->sendRequest("power off", "Powering off");
    URCL_LOG_INFO("Power off command success: %d", success);
  }

  return 0;
}
