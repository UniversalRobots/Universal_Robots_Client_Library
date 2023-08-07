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

  // Flush the log
  if (!my_dashboard->commandSaveLog())
  {
    URCL_LOG_ERROR("Could not send the save log command");
    return 1;
  }

  // Now the robot is ready to receive a program

  return 0;
}
