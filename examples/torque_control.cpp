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

#include <ur_client_library/example_robot_wrapper.h>
#include <cstddef>

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

const size_t JOINT_INDEX = 5;  // Joint index to control, in this case joint 6 (index 5)

std::unique_ptr<urcl::ExampleRobotWrapper> g_my_robot;
urcl::vector6d_t g_joint_positions;

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
  g_my_robot = std::make_unique<urcl::ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                           "external_control.urp");
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  // Torque control requires Software version 5.23 / 10.10 or higher. Error and exit on older
  // software versions.
  {
    auto robot_version = g_my_robot->getUrDriver()->getVersion();
    // ToDo: Increase to 5.23.0 once released
    if (robot_version < urcl::VersionInformation::fromString("5.22.0") ||
        (robot_version.major > 5 && robot_version < urcl::VersionInformation::fromString("10.10.0")))
    {
      URCL_LOG_ERROR("This example requires a robot with at least version 5.23.0 / 10.10.0. Your robot has version %s.",
                     robot_version.toString().c_str());
      return 0;
    }
  }
  // --------------- INITIALIZATION END -------------------

  const double torque_abs = 2.5;
  double cmd_torque = torque_abs;  // Target torque [Nm] for joint 6
  bool passed_negative_part = false;
  bool passed_positive_part = false;
  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t target_torques = { 0, 0, 0, 0, 0, 0 };

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();
  auto start_time = std::chrono::system_clock::now();
  while (!(passed_positive_part && passed_negative_part))
  {
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<urcl::rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
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

    // Open loop control. The target is incremented with a constant each control loop
    if (passed_positive_part == false)
    {
      if (g_joint_positions[JOINT_INDEX] >= 2)
      {
        passed_positive_part = true;
        cmd_torque = -torque_abs;
      }
    }
    else if (passed_negative_part == false)
    {
      if (g_joint_positions[JOINT_INDEX] <= 0)
      {
        cmd_torque = torque_abs;
        passed_negative_part = true;
      }
    }
    target_torques[JOINT_INDEX] = cmd_torque;

    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications. Having it
    // this high means that the robot will continue a motion for this given time if no new command
    // is sent / the connection is interrupted.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(target_torques, urcl::comm::ControlMode::MODE_TORQUE,
                                                            urcl::RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
      return 1;
    }
    URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg->toString().c_str());
    if (second_to_run.count() > 0 && (std::chrono::system_clock::now() - start_time) > second_to_run)
    {
      URCL_LOG_WARN("Time limit reached, stopping movement. This is expected on a simualted robot, as it doesn't move "
                    "to torque commands.");
      break;
    }
  }
  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("Movement done");
  return 0;
}
