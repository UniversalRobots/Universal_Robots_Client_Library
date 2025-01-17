// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2024 Universal Robots A/S
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

#include <chrono>
#include <string>
#include <thread>

#include "ur_client_library/types.h"
#include "ur_client_library/ur/ur_driver.h"
#include "ur_client_library/log.h"
#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/ur/dashboard_client.h"

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

std::unique_ptr<urcl::DashboardClient> g_my_dashboard;
std::unique_ptr<urcl::UrDriver> g_my_driver;

std::atomic<bool> g_trajectory_done = false;

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

void trajDoneCallback(const urcl::control::TrajectoryResult& result)
{
  URCL_LOG_INFO("Trajectory done with result %d", result);
  ;
  g_trajectory_done = true;
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

  // --------------- INITIALIZATION BEGIN -------------------
  // Making the robot ready for the program by:
  // Connect the robot Dashboard
  g_my_dashboard.reset(new urcl::DashboardClient(robot_ip));
  if (!g_my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // // Stop program, if there is one running
  if (!g_my_dashboard->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return 1;
  }

  // Power it off
  if (!g_my_dashboard->commandPowerOff())
  {
    URCL_LOG_ERROR("Could not send Power off command");
    return 1;
  }

  // Power it on
  if (!g_my_dashboard->commandPowerOn())
  {
    URCL_LOG_ERROR("Could not send Power on command");
    return 1;
  }

  // Release the brakes
  if (!g_my_dashboard->commandBrakeRelease())
  {
    URCL_LOG_ERROR("Could not send BrakeRelease command");
    return 1;
  }

  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  const bool headless = true;
  g_my_driver.reset(new urcl::UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                       headless, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
  // --------------- INITIALIZATION END -------------------

  g_my_driver->registerTrajectoryDoneCallback(&trajDoneCallback);

  URCL_LOG_INFO("Running MoveJ motion");
  // --------------- MOVEJ TRAJECTORY -------------------
  {
    g_trajectory_done = false;
    // Trajectory definition
    std::vector<urcl::vector6d_t> points{ { -1.57, -1.57, 0, 0, 0, 0 }, { -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 } };
    std::vector<double> motion_durations{ 5.0, 2.0 };
    std::vector<double> blend_radii{ 0.1, 0.1 };

    // Trajectory execution
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                               points.size() * 2);
    for (size_t i = 0; i < points.size(); i++)
    {
      g_my_driver->writeTrajectoryPoint(points[i], false, motion_durations[i], blend_radii[i]);
    }

    // Same motion, but parametrized with acceleration and velocity
    motion_durations = { 0.0, 0.0 };
    for (size_t i = 0; i < points.size(); i++)
    {
      g_my_driver->writeTrajectoryPoint(points[i], 2.0, 2.0, false, motion_durations[i], blend_radii[i]);
    }

    while (!g_trajectory_done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
  }
  // --------------- END MOVEJ TRAJECTORY -------------------

  URCL_LOG_INFO("Running MoveL motion");
  // --------------- MOVEL TRAJECTORY -------------------
  {
    g_trajectory_done = false;
    // Trajectory definition
    std::vector<urcl::vector6d_t> points{ { -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 },
                                          { -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 } };
    std::vector<double> motion_durations{ 5.0, 5.0 };
    std::vector<double> blend_radii{ 0.0, 0.0 };

    // Trajectory execution
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                               points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
      // setting the cartesian parameter makes it interpret the 6d vector as a pose and use movel
      g_my_driver->writeTrajectoryPoint(points[i], true, motion_durations[i], blend_radii[i]);
    }

    while (!g_trajectory_done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
  }
  // --------------- END MOVEL TRAJECTORY -------------------

  URCL_LOG_INFO("Running a spline motion");
  // --------------- SPLINE TRAJECTORY -------------------
  {
    g_trajectory_done = false;
    // Trajectory definition
    std::vector<urcl::vector6d_t> positions{ { -1.57, -1.57, 0, 0, 0, 0 },
                                             { -1.57, -1.57, -1.57, 0, 0, 0 },
                                             { -1.57, -1.57, 0, 0, 0, 0 },
                                             { -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 } };
    std::vector<urcl::vector6d_t> velocities{
      { 0, 0, 0.0, 0, 0, 0 }, { 0, 0, 0.0, 0, 0, 0 }, { 0, 0, 1.5, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0 }
    };
    std::vector<double> motion_durations{ 3.0, 3.0, 3.0, 3.0 };

    // Trajectory execution
    g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                               positions.size());
    for (size_t i = 0; i < positions.size(); i++)
    {
      g_my_driver->writeTrajectorySplinePoint(positions[i], velocities[i], motion_durations[i]);
    }

    while (!g_trajectory_done)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
  }
  // --------------- END MOVEJ TRAJECTORY -------------------

  g_my_driver->stopControl();
  return 0;
}
