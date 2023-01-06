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

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/primary/primary_parser.h>
#include <ur_client_library/primary/primary_client.h>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // First of all, we need to create a primary client that connects to the robot
  primary_interface::PrimaryClient primary_client(robot_ip, CALIBRATION_CHECKSUM);

  // Give time to get the client to connect
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  for (int i = 0; i < 10; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // Create script program to send through client
    std::stringstream cmd;
    cmd.imbue(std::locale::classic());  // Make sure, decimal divider is actually '.'
    cmd << "sec setup():" << std::endl
        << " textmsg(\"Command through primary interface complete " << i++ << "\")" << std::endl
        << "end";
    std::string script_code = cmd.str();
    auto program_with_newline = script_code + '\n';
    // Send script
    primary_client.sendScript(program_with_newline);

    try
    {
      URCL_LOG_INFO("Cartesian Information:\n%s", primary_client.getCartesianInfo()->toString().c_str());
      URCL_LOG_INFO("Calibration Hash:\n%s", primary_client.getCalibrationChecker()->getData()->toHash().c_str());
      URCL_LOG_INFO("Build Date:\n%s", primary_client.getVersionMessage()->build_date_.c_str());
      std::cout << primary_client.getJointData()->toString() << std::endl;
      std::stringstream os;
      os << primary_client.getJointData()->q_actual_;
      URCL_LOG_INFO("Joint Angles:\n%s", os.str().c_str());
      // getGlobalVariablesSetupMessage() will throw an exception if a program on the robot has not been started
      URCL_LOG_INFO("Global Variables:\n%s", primary_client.getGlobalVariablesSetupMessage()->variable_names_.c_str());
    }
    catch (const UrException& e)
    {
      URCL_LOG_WARN(e.what());
    }
  }
  return 0;
}
