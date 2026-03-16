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
#include "ur_client_library/ur/instruction_executor.h"
#include <cmath>
#include <cstddef>

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

const size_t JOINT_INDEX = 5;  // Joint index to control, in this case joint 6 (index 5)

// This example will apply a sinusoidal torque to JOINT_INDEX, while the other joints are kept at 0.0.
constexpr double frequency = 0.2;  // [Hz]
constexpr double amplitude = 2.5;  // [Nm]
constexpr double omega = 2 * M_PI * frequency;
constexpr double start_position = 0.0;  // [rad]

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

  // This example requires Software version 5.25.1 / 10.12.1 or higher. Skip gracefully on older
  // software versions so that CI (which runs all examples) does not fail.
  {
    auto robot_version = g_my_robot->getUrDriver()->getVersion();
    bool version_supported =
        ((robot_version.major == 5) && (robot_version >= urcl::VersionInformation::fromString("5.25.1"))) ||
        ((robot_version.major >= 10) && (robot_version >= urcl::VersionInformation::fromString("10.12.1")));
    if (!version_supported)
    {
      URCL_LOG_INFO("This direct_torque control example requires a robot with at least version 5.25.1 / 10.12.1. Your "
                    "robot has version %s. Skipping.",
                    robot_version.toString().c_str());
      return 0;
    }
  }
  // --------------- INITIALIZATION END -------------------

  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t target_torques = { 0, 0, 0, 0, 0, 0 };

  // Scale each individual joint's friction compensation. This is supported from PolyScope 5.25.1 / PolyScope X 10.12.1
  // and upwards.
  urcl::vector6d_t viscous_scale = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t coulomb_scale = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // As this example only applies to JOINT_INDEX, we can set the other joints to 0.0 to disable friction compensation to
  // make them more steady.
  viscous_scale[JOINT_INDEX] = 1.0;
  coulomb_scale[JOINT_INDEX] = 1.0;
  g_my_robot->getUrDriver()->setFrictionScales(viscous_scale, coulomb_scale);

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();

  urcl::rtde_interface::DataPackage data_pkg(g_my_robot->getUrDriver()->getRTDEOutputRecipe());
  if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
  {
    URCL_LOG_ERROR("Could not get fresh data package from robot");
    return 1;
  }
  if (!data_pkg.getData("actual_q", g_joint_positions))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
  // Use motion primitives to move JOINT_INDEX to start position
  g_joint_positions[JOINT_INDEX] = start_position;
  auto instruction_executor = std::make_shared<urcl::InstructionExecutor>(g_my_robot->getUrDriver());
  instruction_executor->moveJ(g_joint_positions, 1.4, 1.04, 0.1);

  auto start_time = std::chrono::system_clock::now();
  constexpr double timestep = 0.002;  // [s]
  double time = 0.0;

  // Run 10 periods of the sinusoidal
  while (time < 10 * 2 * M_PI / omega)
  {
    time += timestep;
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
    {
      URCL_LOG_WARN("Could not get fresh data package from robot");
      return 1;
    }
    // Read current joint positions from robot data
    if (!data_pkg.getData("actual_q", g_joint_positions))
    {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    // Open loop control. The target is incremented with a constant each control loop
    target_torques[JOINT_INDEX] = amplitude * std::sin(omega * time);

    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications. Having it
    // this high means that the robot will continue a motion for this given time if no new command
    // is sent / the connection is interrupted.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(target_torques, urcl::comm::ControlMode::MODE_TORQUE,
                                                            urcl::RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Make sure that the robot is in remote control mode and connected "
                     "with a network cable.");
      return 1;
    }
    URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg.toString().c_str());
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
