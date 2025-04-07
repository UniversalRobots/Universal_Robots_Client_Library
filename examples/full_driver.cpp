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
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-06
 *
 * Make sure to run this program from its source directory in order to find the respective files.
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <iostream>
#include <memory>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;
vector6d_t g_joint_positions;

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  bool headless_mode = true;
  g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                     "external_control.urp");
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }
  // --------------- INITIALIZATION END -------------------

  // Increment depends on robot version
  double increment_constant = 0.0005;
  if (g_my_robot->getUrDriver()->getVersion().major < 5)
  {
    increment_constant = 0.002;
  }
  double increment = increment_constant;

  bool first_pass = true;
  bool passed_negative_part = false;
  bool passed_positive_part = false;
  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t joint_target = { 0, 0, 0, 0, 0, 0 };

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();
  while (!(passed_positive_part && passed_negative_part))
  {
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
    if (!data_pkg)
    {
      URCL_LOG_WARN("Could not get fresh data package from robot");
      return 1;
    }
    // Read current joint positions from robot data
    if (!data_pkg->getData("actual_q", g_joint_positions))
    {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (first_pass)
    {
      joint_target = g_joint_positions;
      first_pass = false;
    }

    // Open loop control. The target is incremented with a constant each control loop
    if (passed_positive_part == false)
    {
      increment = increment_constant;
      if (g_joint_positions[5] >= 2)
      {
        passed_positive_part = true;
      }
    }
    else if (passed_negative_part == false)
    {
      increment = -increment_constant;
      if (g_joint_positions[5] <= 0)
      {
        passed_negative_part = true;
      }
    }
    joint_target[5] += increment;
    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(joint_target, comm::ControlMode::MODE_SERVOJ,
                                                            RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
      return 1;
    }
    URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg->toString().c_str());
  }
  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("Movement done");
  return 0;
}
