// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2023 Universal Robots A/S
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

#include <ur_client_library/control/trajectory_point_interface.h>
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
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

std::unique_ptr<UrDriver> g_my_driver;
std::unique_ptr<DashboardClient> g_my_dashboard;
vector6d_t g_joint_positions;

void SendTrajectory(const std::vector<vector6d_t>& p_p, const std::vector<vector6d_t>& p_v,
                    const std::vector<vector6d_t>& p_a, const std::vector<double>& time, bool use_spline_interpolation_)
{
  assert(p_p.size() == time.size());

  URCL_LOG_INFO("Starting joint-based trajectory forward");
  g_my_driver->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START, p_p.size());

  for (size_t i = 0; i < p_p.size() && p_p.size() == time.size() && p_p[i].size() == 6; i++)
  {
    // MoveJ
    if (!use_spline_interpolation_)
    {
      g_my_driver->writeTrajectoryPoint(p_p[i], false, time[i]);
    }
    else  // Use spline interpolation
    {
      // QUINTIC
      if (p_v.size() == time.size() && p_a.size() == time.size() && p_v[i].size() == 6 && p_a[i].size() == 6)
      {
        g_my_driver->writeTrajectorySplinePoint(p_p[i], p_v[i], p_a[i], time[i]);
      }
      // CUBIC
      else if (p_v.size() == time.size() && p_v[i].size() == 6)
      {
        g_my_driver->writeTrajectorySplinePoint(p_p[i], p_v[i], time[i]);
      }
      else
      {
        g_my_driver->writeTrajectorySplinePoint(p_p[i], time[i]);
      }
    }
  }
  URCL_LOG_INFO("Finished Sending Trajectory");
}

// We need a callback function to register. See UrDriver's parameters for details.
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

// Callback function for trajectory execution.
bool g_trajectory_running(false);
void handleTrajectoryState(control::TrajectoryResult state)
{
  // trajectory_state = state;
  g_trajectory_running = false;
  std::string report = "?";
  switch (state)
  {
    case control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS:
      report = "success";
      break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED:
      report = "canceled";
      break;
    case control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE:
    default:
      report = "failure";
      break;
  }
  std::cout << "\033[1;32mTrajectory report: " << report << "\033[0m\n" << std::endl;
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

  // Making the robot ready for the program by:
  // Connect the the robot Dashboard
  g_my_dashboard.reset(new DashboardClient(robot_ip));
  if (!g_my_dashboard->connect())
  {
    URCL_LOG_ERROR("Could not connect to dashboard");
    return 1;
  }

  // Stop program, if there is one running
  if (!g_my_dashboard->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return 1;
  }

  // if the robot is not powered on and ready
  std::string robotModeRunning("RUNNING");
  while (!g_my_dashboard->commandRobotMode(robotModeRunning))
  {
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
  }

  // Release the brakes
  if (!g_my_dashboard->commandBrakeRelease())
  {
    URCL_LOG_ERROR("Could not send BrakeRelease command");
    return 1;
  }

  // Now the robot is ready to receive a program

  std::unique_ptr<ToolCommSetup> tool_comm_setup;
  const bool HEADLESS = true;
  g_my_driver.reset(new UrDriver(robot_ip, SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState, HEADLESS,
                                 std::move(tool_comm_setup), CALIBRATION_CHECKSUM));
  g_my_driver->setKeepaliveCount(5);  // This is for example purposes only. This will make the example running more
                                      // reliable on non-realtime systems. Do not use this in productive applications.

  g_my_driver->registerTrajectoryDoneCallback(&handleTrajectoryState);

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefore, do this directly before starting your main
  // loop.

  g_my_driver->startRTDECommunication();

  std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();

  if (data_pkg)
  {
    // Read current joint positions from robot data
    if (!data_pkg->getData("actual_q", g_joint_positions))
    {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
  }

  const unsigned number_of_points = 5;
  const double s_pos[number_of_points] = { 4.15364583e-03, 4.15364583e-03, -1.74088542e-02, 1.44817708e-02, 0.0 };
  const double s_vel[number_of_points] = { -2.01015625e-01, 4.82031250e-02, 1.72812500e-01, -3.49453125e-01,
                                           8.50000000e-02 };
  const double s_acc[number_of_points] = { 2.55885417e+00, -4.97395833e-01, 1.71276042e+00, -5.36458333e-02,
                                           -2.69817708e+00 };
  const double s_time[number_of_points] = { 1.0000000e+00, 4.00000000e+00, 8.00100000e+00, 1.25000000e+01,
                                            4.00000000e+00 };

  bool ret = false;
  URCL_LOG_INFO("Switch to Forward mode");
  ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);
  if (!ret)
  {
    std::stringstream lastq;
    lastq << g_joint_positions;
    URCL_LOG_ERROR("Could not send joint command. Last q received is %s\n Is the robot in remote control?",
                   lastq.str().c_str());
    return 1;
  }

  std::vector<vector6d_t> p, v, a;
  std::vector<double> time;

  unsigned int joint_to_control = 5;
  for (unsigned i = 0; i < number_of_points; ++i)
  {
    vector6d_t p_i = g_joint_positions;
    p_i[joint_to_control] = s_pos[i];
    p.push_back(p_i);

    vector6d_t v_i;
    v_i[joint_to_control] = s_vel[i];
    v.push_back(v_i);

    vector6d_t a_i;
    a_i[joint_to_control] = s_acc[i];
    a.push_back(a_i);

    time.push_back(s_time[i]);
  }

  // CUBIC
  SendTrajectory(p, v, std::vector<vector6d_t>(), time, true);

  g_trajectory_running = true;
  while (g_trajectory_running)
  {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
    if (data_pkg)
    {
      // Read current joint positions from robot data
      if (!data_pkg->getData("actual_q", g_joint_positions))
      {
        // This throwing should never happen unless misconfigured
        std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
        throw std::runtime_error(error_msg);
      }
      bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

      if (!ret)
      {
        std::stringstream lastq;
        lastq << g_joint_positions;
        URCL_LOG_ERROR("Could not send joint command. Last q received is %s\n Is the robot in remote control?",
                       lastq.str().c_str());
        return 1;
      }
    }
  }

  URCL_LOG_INFO("CUBIC Movement done");

  // QUINTIC
  SendTrajectory(p, v, a, time, true);
  ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

  g_trajectory_running = true;
  while (g_trajectory_running)
  {
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_driver->getDataPackage();
    if (data_pkg)
    {
      // Read current joint positions from robot data
      if (!data_pkg->getData("actual_q", g_joint_positions))
      {
        // This throwing should never happen unless misconfigured
        std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
        throw std::runtime_error(error_msg);
      }
      bool ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);

      if (!ret)
      {
        std::stringstream lastq;
        lastq << g_joint_positions;
        URCL_LOG_ERROR("Could not send joint command. Last q received is %s\n Is the robot in remote control?",
                       lastq.str().c_str());
        return 1;
      }
    }
  }

  URCL_LOG_INFO("QUINTIC Movement done");

  ret = g_my_driver->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD);
  if (!ret)
  {
    std::stringstream lastq;
    lastq << g_joint_positions;
    URCL_LOG_ERROR("Could not send joint command. Last q received is %s\n Is the robot in remote control?",
                   lastq.str().c_str());
    return 1;
  }
  return 0;
}
