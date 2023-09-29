// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-09-11
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/ur/dashboard_client.h>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
  // Set the loglevel to info get print out the DH parameters
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Parse how many seconds to run
  int second_to_run = -1;
  if (argc > 2)
  {
    second_to_run = std::stoi(argv[2]);
  }

  // The robot should be running in order to send script code to it
  // Connect the the robot Dashboard
  std::unique_ptr<DashboardClient> dashboard_client;
  dashboard_client.reset(new DashboardClient(robot_ip));
  if (!dashboard_client->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // Stop program, if there is one running
  if (!dashboard_client->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return 1;
  }

  // Release the brakes
  if (!dashboard_client->commandBrakeRelease())
  {
    URCL_LOG_ERROR("Could not send BrakeRelease command");
    return 1;
  }

  primary_interface::PrimaryClient primary_client(robot_ip);

  // Check that the calibration checksum matches the one provided from the robot
  const std::string calibration_check_sum = "";
  bool check_calibration_result = primary_client.checkCalibration(calibration_check_sum);
  std::string calibration_check_sum_matches = check_calibration_result ? "true" : "false";
  URCL_LOG_INFO("calibration check sum matches: %s", calibration_check_sum_matches.c_str());

  // Send a script program to the robot
  std::stringstream cmd;
  cmd.imbue(std::locale::classic());  // Make sure, decimal divider is actually '.'
  cmd << "def test():" << std::endl << "textmsg(\"Hello from script program\")" << std::endl << "end";

  if (primary_client.sendScript(cmd.str()))
  {
    URCL_LOG_INFO("Script program was successfully sent to the robot");
  }
  else
  {
    URCL_LOG_ERROR("Script program wasn't send successfully to the robot");
    return 1;
  }

  // Send a secondary script program to the robot
  cmd.str("");
  cmd << "sec setup():" << std::endl << "textmsg(\"Hello from secondary program\")" << std::endl << "end";
  if (primary_client.sendSecondaryScript(cmd.str()))
  {
    URCL_LOG_INFO("Secondary script program was successfully sent to the robot");
  }
  else
  {
    URCL_LOG_ERROR("Secondary script program wasn't send successfully to the robot");
    return 1;
  }

  // Package contents will be printed while not being interrupted
  do
  {
    std::this_thread::sleep_for(std::chrono::seconds(second_to_run));
  } while (second_to_run < 0);
  return 0;
}
