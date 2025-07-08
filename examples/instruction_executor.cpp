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
#include <ur_client_library/example_robot_wrapper.h>
#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/ur/dashboard_client.h"
#include "ur_client_library/ur/instruction_executor.h"

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

std::unique_ptr<urcl::ExampleRobotWrapper> g_my_robot;

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
  g_my_robot = std::make_unique<urcl::ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                           "external_control.urp");
  if (!g_my_robot->getUrDriver()->checkCalibration(CALIBRATION_CHECKSUM))
  {
    URCL_LOG_ERROR("Calibration checksum does not match actual robot.");
    URCL_LOG_ERROR("Use the ur_calibration tool to extract the correct calibration from the robot and pass that into "
                   "the description. See "
                   "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] "
                   "for details.");
  }
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }

  auto instruction_executor = std::make_shared<urcl::InstructionExecutor>(g_my_robot->getUrDriver());
  // --------------- INITIALIZATION END -------------------

  URCL_LOG_INFO("Running motion");
  // Trajectory definition
  std::vector<std::shared_ptr<urcl::control::MotionPrimitive>> motion_sequence{
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 0, 0, 0, 0 }, 0.1,
                                                    std::chrono::seconds(5)),
    // This point uses acceleration / velocity parametrization
    std::make_shared<urcl::control::MoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1,
                                                    std::chrono::seconds(0), 1.4, 2.0),

    std::make_shared<urcl::control::MoveLPrimitive>(urcl::Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076), 0.1,
                                                    std::chrono::seconds(2)),
    std::make_shared<urcl::control::MovePPrimitive>(urcl::Pose{ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1, 0.4,
                                                    0.4),
    std::make_shared<urcl::control::OptimoveJPrimitive>(urcl::vector6d_t{ -1.57, -1.57, 1.6, -0.5, 0.4, 0.3 }, 0.1, 0.4,
                                                        0.7),
    std::make_shared<urcl::control::OptimoveLPrimitive>(urcl::Pose(-0.203, 0.263, 0.559, 0.68, -1.083, -2.076), 0.1,
                                                        0.4, 0.7),
  };
  instruction_executor->executeMotion(motion_sequence);

  double goal_time_sec = 2.0;

  // acceleration / velocity parametrization
  instruction_executor->moveJ({ -1.57, -1.57, 0, 0, 0, 0 }, 2.0, 2.0);
  // goal time parametrization -- acceleration and velocity will be ignored
  instruction_executor->moveJ({ -1.57, -1.6, 1.6, -0.7, 0.7, 0.2 }, 0.1, 0.1, goal_time_sec);
  // acceleration / velocity parametrization
  instruction_executor->moveL({ -0.203, 0.263, 0.559, 0.68, -1.083, -2.076 }, 1.5, 1.5);
  // goal time parametrization -- acceleration and velocity will be ignored
  instruction_executor->moveL({ -0.203, 0.463, 0.559, 0.68, -1.083, -2.076 }, 0.1, 0.1, goal_time_sec);

  instruction_executor->moveP({ -0.203, 0.463, 0.759, 0.68, -1.083, -2.076 }, 1.5, 1.5);

  g_my_robot->getUrDriver()->stopControl();
  return 0;
}