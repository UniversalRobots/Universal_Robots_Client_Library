// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

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

#include "ur_client_library/example_robot_wrapper.h"
#include "ur_client_library/ur/dashboard_client.h"
#include "ur_client_library/ur/ur_driver.h"
#include "ur_client_library/types.h"
#include "ur_client_library/ur/instruction_executor.h"

#include <iostream>
#include <memory>
#include <cmath>
#include <signal.h>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

void signal_callback_handler(int signum)
{
  std::cout << "Caught signal " << signum << std::endl;
  if (g_my_robot != nullptr)
  {
    // Stop control of the robot
    g_my_robot->getUrDriver()->stopControl();
  }
  // Terminate program
  exit(signum);
}

struct DataStorage
{
  std::vector<urcl::vector6d_t> target;
  std::vector<urcl::vector6d_t> actual;
  std::vector<double> time;

  void write_to_file(const std::string& filename)
  {
    std::fstream output(filename, std::ios::out);
    output << "timestamp,target0,target1,target2,target3,target4,target5,actual0,actual1,actual2,"
              "actual3,actual4,actual5\n";
    for (size_t i = 0; i < time.size(); ++i)
    {
      output << time[i];
      for (auto& t : target[i])
      {
        output << "," << t;
      }
      for (auto& a : actual[i])
      {
        output << "," << a;
      }
      output << "\n";
    }
    output.close();
  }
};

bool pd_control_loop(DataStorage& data_storage, const std::string& actual_data_name,
                     const comm::ControlMode control_mode, const urcl::vector6d_t& amplitude)
{
  const int32_t running_time = 13;
  double time = 0.0;

  // Reserve space for expected amount of data
  data_storage.actual.reserve(running_time * 500);
  data_storage.target.reserve(running_time * 500);
  data_storage.time.reserve(running_time * 500);

  urcl::vector6d_t actual, target, start = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  bool first_pass = true;
  while (time < running_time)
  {
    const auto t_start = std::chrono::high_resolution_clock::now();
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
    if (!data_pkg)
    {
      URCL_LOG_WARN("Could not get fresh data package from robot");
      return false;
    }
    // Read current joint positions from robot data
    if (!data_pkg->getData(actual_data_name, actual))
    {
      // This throwing should never happen unless misconfigured
      std::string error_msg = "Did not find" + actual_data_name + "in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (first_pass)
    {
      target = actual;
      start = actual;
      for (size_t i = 0; i < start.size(); ++i)
      {
        start[i] = start[i] - amplitude[i];
      }
      first_pass = false;
    }

    for (size_t i = 0; i < target.size(); ++i)
    {
      target[i] = start[i] + amplitude[i] * cos(time);
    }

    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(target, control_mode, RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
      return false;
    }

    // Increment time and log data
    const auto t_end = std::chrono::high_resolution_clock::now();
    const double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count() / 1000;
    time = time + elapsed_time_ms;
    data_storage.actual.push_back(actual);
    data_storage.target.push_back(target);
    data_storage.time.push_back(time);
  }

  return true;
}

int main(int argc, char* argv[])
{
  // This will make sure that we stop controlling the robot if the user presses ctrl-c
  signal(SIGINT, signal_callback_handler);

  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  const bool headless_mode = true;
  g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode);

  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  auto instruction_executor = std::make_shared<urcl::InstructionExecutor>(g_my_robot->getUrDriver());

  URCL_LOG_INFO("Move the robot to initial position");
  instruction_executor->moveJ(urcl::vector6d_t{ 0, -1.67, -0.65, -1.59, 1.61, 5.09 }, 0.5, 0.2, 5);

  DataStorage joint_controller_storage;

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();
  URCL_LOG_INFO("Start controlling the robot using the PD controller");
  const bool completed_joint_control =
      pd_control_loop(joint_controller_storage, "actual_q", comm::ControlMode::MODE_PD_CONTROLLER_JOINT,
                      { 0.2, 0.2, 0.2, 0.2, 0.2, 0.2 });
  if (!completed_joint_control)
  {
    URCL_LOG_ERROR("Didn't complete pd control in joint space");
    g_my_robot->getUrDriver()->stopControl();
    return 1;
  }

  return 0;
}
