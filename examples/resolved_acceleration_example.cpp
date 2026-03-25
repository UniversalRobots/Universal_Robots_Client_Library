// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

// Required before <cmath> for M_PI on Windows (MSVC)
#define _USE_MATH_DEFINES
#include <cmath>

#include <ur_client_library/example_robot_wrapper.h>
#include "ur_client_library/ur/instruction_executor.h"

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

const size_t JOINT_INDEX = 5;

// Sinusoidal reference trajectory parameters
constexpr double frequency = 0.2;           // [Hz]
constexpr double position_amplitude = 0.5;  // [rad]
constexpr double omega = 2 * M_PI * frequency;

// PD controller gains for the controlled joint.
// The commanded acceleration is: qdd = Kp * (q_ref - q) + Kd * (qd_ref - qd)
// The urscript then computes the torque via inverse dynamics:
//   tau = M(q) * qdd + C(q, qd) - J^T * F_ext
constexpr double KP = 50.0;  // [1/s²]  proportional gain
constexpr double KD = 10.0;  // [1/s]   derivative gain

std::unique_ptr<urcl::ExampleRobotWrapper> g_my_robot;
urcl::vector6d_t g_joint_positions;
urcl::vector6d_t g_joint_velocities;

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

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

  // Resolved acceleration requires Software version 5.23.0 / 10.11.0 or higher.
  {
    auto robot_version = g_my_robot->getUrDriver()->getVersion();
    bool version_supported =
        ((robot_version.major == 5) && (robot_version >= urcl::VersionInformation::fromString("5.23.0"))) ||
        ((robot_version.major >= 10) && (robot_version >= urcl::VersionInformation::fromString("10.11.0")));
    if (!version_supported)
    {
      URCL_LOG_INFO("This resolved acceleration example requires a robot with at least version 5.23.0 / 10.11.0. "
                    "Your robot has version %s. Skipping.",
                    robot_version.toString().c_str());
      return 0;
    }
  }
  // --------------- INITIALIZATION END -------------------

  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t target_accelerations = { 0, 0, 0, 0, 0, 0 };

  g_my_robot->getUrDriver()->startRTDECommunication();

  urcl::rtde_interface::DataPackage data_pkg(g_my_robot->getUrDriver()->getRTDEOutputRecipe());
  if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
  {
    URCL_LOG_ERROR("Could not get fresh data package from robot");
    return 1;
  }
  if (!data_pkg.getData("actual_q", g_joint_positions))
  {
    std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }

  // Scale each individual joint's friction compensation. This is supported from PolyScope 5.25.1 / PolyScope X 10.12.1
  // and upwards.
  urcl::vector6d_t viscous_scale = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t coulomb_scale = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // As this example only applies to JOINT_INDEX, we can set the other joints to 0.0 to disable friction compensation to
  // make them more steady.
  viscous_scale[JOINT_INDEX] = 1.0;
  coulomb_scale[JOINT_INDEX] = 1.0;
  // g_my_robot->getUrDriver()->setFrictionScales(viscous_scale, coulomb_scale);

  // Record initial positions as the hold targets for all joints. JOINT_INDEX will get a sinusoidal
  // offset on top.
  urcl::vector6d_t q_ref_hold = g_joint_positions;

  auto start_time = std::chrono::system_clock::now();
  constexpr double timestep = 0.002;  // [s]
  double time = 0.0;

  // Run 10 full periods of the sinusoidal reference
  while (time < 10 * 2 * M_PI / omega)
  {
    time += timestep;
    if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
    {
      URCL_LOG_WARN("Could not get fresh data package from robot");
      return 1;
    }
    if (!data_pkg.getData("actual_q", g_joint_positions))
    {
      std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg.getData("actual_qd", g_joint_velocities))
    {
      std::string error_msg = "Did not find 'actual_qd' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    // PD control on all joints: hold them at their initial positions (qd_ref = 0, qdd_ff = 0)
    for (size_t i = 0; i < 6; ++i)
    {
      target_accelerations[i] = KP * (q_ref_hold[i] - g_joint_positions[i]) + KD * (0.0 - g_joint_velocities[i]);
    }

    // Apply sinusoidal reference with feedforward on JOINT_INDEX only
    double q_ref = q_ref_hold[JOINT_INDEX] + position_amplitude * std::sin(omega * time);
    double qd_ref = position_amplitude * omega * std::cos(omega * time);
    double qdd_feedforward = -position_amplitude * omega * omega * std::sin(omega * time);
    double position_error = q_ref - g_joint_positions[JOINT_INDEX];
    double velocity_error = qd_ref - g_joint_velocities[JOINT_INDEX];
    target_accelerations[JOINT_INDEX] = qdd_feedforward + KP * position_error + KD * velocity_error;

    // Send the commanded accelerations. The urscript resolvedAccelerationThread converts them to
    // torques using the robot's dynamics model: tau = M(q)*qdd + C(q,qd) - J^T * F_ext
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(target_accelerations,
                                                            urcl::comm::ControlMode::MODE_RESOLVED_ACCELERATION,
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
      URCL_LOG_WARN("Time limit reached, stopping movement. This is expected on a simulated robot, as it doesn't "
                    "respond to torque-based commands.");
      break;
    }
  }
  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("Movement done");
  return 0;
}
