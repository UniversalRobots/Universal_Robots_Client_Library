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

#include <gtest/gtest.h>

#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>

#include <iostream>
#include <memory>
#include <math.h>
#include <fstream>

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string SPLINE_SCRIPT_FILE = "spline_external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe_spline.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string ROBOT_IP = "192.168.56.101";

std::unique_ptr<UrDriver> g_ur_driver_;
std::unique_ptr<DashboardClient> g_dashboard_client_;
bool g_trajectory_running_;

// Helper functions for the driver
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
}

void handleTrajectoryState(control::TrajectoryResult state)
{
  g_trajectory_running_ = false;
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
}

class SplineInterpolationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    g_dashboard_client_.reset(new DashboardClient(ROBOT_IP));
    ASSERT_TRUE(g_dashboard_client_->connect());

    // Make robot ready for test
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    g_dashboard_client_->setReceiveTimeout(tv);

    // Stop running program if there is one
    ASSERT_TRUE(g_dashboard_client_->commandStop());

    // if the robot is not powered on and ready
    ASSERT_TRUE(g_dashboard_client_->commandBrakeRelease());

    // Add splineTimerTraveled to output registers to check for correct computation on test side
    std::ifstream in_file(SCRIPT_FILE);
    std::string prog((std::istreambuf_iterator<char>(in_file)), (std::istreambuf_iterator<char>()));

    std::string comment_replace = "# USED_IN_TEST_SPLINE_INTERPOLATION write_output_float_register(1, "
                                  "splineTimerTraveled)";
    std::string replacement = "write_output_float_register(1, splineTimerTraveled)";
    while (prog.find(comment_replace) != std::string::npos)
    {
      prog.replace(prog.find(comment_replace), comment_replace.length(), replacement);
    }

    std::ofstream out_file;
    out_file.open(SPLINE_SCRIPT_FILE);
    out_file << prog;
    out_file.close();

    // Setup driver
    std::unique_ptr<ToolCommSetup> tool_comm_setup;
    const bool HEADLESS = true;
    g_ur_driver_.reset(new UrDriver(ROBOT_IP, SPLINE_SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE, &handleRobotProgramState,
                                    HEADLESS, std::move(tool_comm_setup), CALIBRATION_CHECKSUM));

    g_ur_driver_->registerTrajectoryDoneCallback(&handleTrajectoryState);
    g_ur_driver_->setKeepaliveCount(10);

    g_ur_driver_->startRTDECommunication();
  }

  static void TearDownTestSuite()
  {
    g_dashboard_client_->disconnect();
    // Remove temporary file again
    std::remove(SPLINE_SCRIPT_FILE.c_str());
  }

  void SetUp()
  {
    step_time_ = 0.002;
    if (g_ur_driver_->getVersion().major < 5)
    {
      step_time_ = 0.008;
    }
  }

  void sendTrajectory(const std::vector<urcl::vector6d_t>& s_pos, const std::vector<urcl::vector6d_t>& s_vel,
                      const std::vector<urcl::vector6d_t>& s_acc, const std::vector<double>& s_time)
  {
    ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));

    // Send trajectory to robot for execution
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                            s_pos.size()));

    for (size_t i = 0; i < s_pos.size(); i++)
    {
      // QUINTIC
      if (s_pos.size() == s_acc.size())
      {
        g_ur_driver_->writeTrajectorySplinePoint(s_pos[i], s_vel[i], s_acc[i], s_time[i]);
      }
      // CUBIC
      else
      {
        g_ur_driver_->writeTrajectorySplinePoint(s_pos[i], s_vel[i], s_time[i]);
      }
    }
  }

  void interpolate(const double& time, urcl::vector6d_t& positions, const std::vector<urcl::vector6d_t>& coefficients)
  {
    for (unsigned int i = 0; i < 6; ++i)
    {
      positions[i] = ((((coefficients[i][5] * time + coefficients[i][4]) * time + coefficients[i][3]) * time +
                       coefficients[i][2]) *
                          time +
                      coefficients[i][1]) *
                         time +
                     coefficients[i][0];
    }
  }

  std::vector<urcl::vector6d_t> createSegment(const double& time, const urcl::vector6d_t& start_position,
                                              const urcl::vector6d_t& end_position,
                                              const urcl::vector6d_t& start_velocity,
                                              const urcl::vector6d_t& end_velocity,
                                              const urcl::vector6d_t& start_acceleration,
                                              const urcl::vector6d_t& end_acceleration)
  {
    std::vector<urcl::vector6d_t> coefficients;
    for (unsigned int i = 0; i < 6; ++i)
    {
      urcl::vector6d_t coefficient;
      double time2 = pow(time, 2);
      coefficient[0] = start_position[i];
      coefficient[1] = start_velocity[i];
      coefficient[2] = 0.5 * start_acceleration[i];
      coefficient[3] = (-20.0 * start_position[i] + 20.0 * end_position[i] - 3.0 * start_acceleration[i] * time2 +
                        end_acceleration[i] * time2 - 12.0 * start_velocity[i] * time - 8.0 * end_velocity[i] * time) /
                       (2.0 * pow(time, 3));
      coefficient[4] =
          (30.0 * start_position[i] - 30.0 * end_position[i] + 3.0 * start_acceleration[i] * time2 -
           2.0 * end_acceleration[i] * time2 + 16.0 * start_velocity[i] * time + 14.0 * end_velocity[i] * time) /
          (2.0 * pow(time, 4));
      coefficient[5] = (-12.0 * start_position[i] + 12.0 * end_position[i] - start_acceleration[i] * time2 +
                        end_acceleration[i] * time2 - 6.0 * start_velocity[i] * time - 6.0 * end_velocity[i] * time) /
                       (2.0 * pow(time, 5));
      coefficients.push_back(coefficient);
    }
    return coefficients;
  }

  std::vector<urcl::vector6d_t> createSegment(const double& time, const urcl::vector6d_t& start_position,
                                              const urcl::vector6d_t& end_position,
                                              const urcl::vector6d_t& start_velocity,
                                              const urcl::vector6d_t& end_velocity)
  {
    std::vector<urcl::vector6d_t> coefficients;
    for (unsigned int i = 0; i < 6; ++i)
    {
      urcl::vector6d_t coefficient;
      coefficient[0] = start_position[i];
      coefficient[1] = start_velocity[i];
      coefficient[2] =
          (-3 * start_position[i] + end_position[i] * 3 - start_velocity[i] * 2 * time - end_velocity[i] * time) /
          pow(time, 2);
      coefficient[3] =
          (2 * start_position[i] - 2 * end_position[i] + start_velocity[i] * time + end_velocity[i] * time) /
          pow(time, 3);
      coefficient[4] = 0.0;
      coefficient[5] = 0.0;
      coefficients.push_back(coefficient);
    }
    return coefficients;
  }

  void waitForTrajectoryStarted()
  {
    bool trajectory_started = false;
    double timeout = 1;
    double cur_time = 0.0;
    while (trajectory_started == false)
    {
      std::unique_ptr<rtde_interface::DataPackage> data_pkg;
      readDataPackage(data_pkg);
      double spline_travel_time = 0.0;
      data_pkg->getData("output_double_register_1", spline_travel_time);

      // Keep connection alive
      ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));
      if (std::abs(spline_travel_time - 0.0) < 0.01)
      {
        return;
      }
      if (cur_time > timeout)
      {
        std::cout << "Trajectory didn't start within timeout, is the spline travel time written to output float "
                     "register 1?"
                  << std::endl;
        GTEST_FAIL();
      }
      cur_time += step_time_;
    }
  }

  void readDataPackage(std::unique_ptr<rtde_interface::DataPackage>& data_pkg)
  {
    data_pkg = g_ur_driver_->getDataPackage();
    if (data_pkg == nullptr)
    {
      std::cout << "Failed to get data package from robot" << std::endl;
      GTEST_FAIL();
    }
  }

  // Allowed difference between expected trajectory and actual robot trajectory
  double eps_ = 0.02;

  // Robot step time
  double step_time_;

  // Deceleration variables if changed in the script, they should be changed here as well
  double deceleration_time_ = 0.4189;
  double max_deceleration_ = 15;
};

TEST_F(SplineInterpolationTest, cubic_spline_with_end_point_velocity)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions, joint_velocities;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

  std::vector<urcl::vector6d_t> s_pos, s_vel;
  s_pos.push_back(joint_positions);

  urcl::vector6d_t end_velocity = { 0.273 * 0.5, 0.273 * 0.4, 0.273 * 0.3, 0.273 * 0.2, 0.273 * 0.1, 0.273 };
  s_vel.push_back(end_velocity);

  std::vector<double> s_time = { 2.0000000e+00 };

  // Build the segment
  std::vector<urcl::vector6d_t> coefficients =
      createSegment(s_time[0], joint_positions, s_pos[0], joint_velocities, s_vel[0]);

  // Data for logging
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, expected_positions;
  std::vector<double> time_vec;

  // Send the trajectory to the robot
  sendTrajectory(s_pos, s_vel, std::vector<urcl::vector6d_t>(), s_time);
  g_trajectory_running_ = true;

  // Make sure that the trajectory has started before we start testing for trajectory execution
  waitForTrajectoryStarted();

  double old_spline_travel_time = 0.0;
  double plot_time = 0.0;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);

    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));

    // Read spline travel time from the robot
    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    double spline_travel_step_time = spline_travel_time - old_spline_travel_time;
    old_spline_travel_time = spline_travel_time;

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - spline_travel_step_time);
    double time_left = s_time[0] - spline_travel_time;

    // Ensure that we follow the joint trajectory
    urcl::vector6d_t expected_joint_positions;
    interpolate(spline_travel_time, expected_joint_positions, coefficients);
    for (unsigned int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(expected_joint_positions[i], joint_positions[i], eps_);
    }

    if (time_left < deceleration_time_)
    {
      double max_speed = max_deceleration_ * deceleration_time_;
      double max_allowable_speed = max_speed - (deceleration_time_ - time_left) * max_deceleration_;

      for (unsigned int i = 0; i < 6; ++i)
      {
        // Test that the robot stays within the maximum allowed speed. At very low velocities we might see the joint
        // velocity being slightly larger than the max_allowable_speed, which is the motivation for the check. But if
        // the robot doesn't slow down as intended it will be cached by this check.
        EXPECT_LT(std::abs(joint_velocities[i]) - max_allowable_speed, 0.01);
      }
    }
    expected_positions.push_back(expected_joint_positions);
    actual_positions.push_back(joint_positions);
    actual_velocities.push_back(joint_velocities);
    time_vec.push_back(plot_time);
    plot_time += step_time_;
  }
  // Make sure the velocity is zero when the trajectory has finished
  readDataPackage(data_pkg);
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_velocities[i], 0.0);
  }

  std::ofstream outfile("../test_artifacts/cubic_spline_with_end_point_velocity.txt");
  for (unsigned int i = 0; i < actual_positions.size(); ++i)
  {
    outfile << time_vec[i] << "," << actual_positions[i][0] << "," << actual_positions[i][1] << ","
            << actual_positions[i][2] << "," << actual_positions[i][3] << "," << actual_positions[i][4] << ","
            << actual_positions[i][5] << "," << actual_velocities[i][0] << "," << actual_velocities[i][1] << ","
            << actual_velocities[i][2] << "," << actual_velocities[i][3] << "," << actual_velocities[i][4] << ","
            << actual_velocities[i][5] << "," << expected_positions[i][0] << "," << expected_positions[i][1] << ","
            << expected_positions[i][2] << "," << expected_positions[i][3] << "," << expected_positions[i][4] << ","
            << expected_positions[i][5] << "\n";
  }
}

TEST_F(SplineInterpolationTest, quintic_spline_with_end_point_velocity)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions, joint_velocities;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

  std::vector<urcl::vector6d_t> s_pos, s_vel, s_acc;
  s_pos.push_back(joint_positions);

  urcl::vector6d_t end_velocity = { 0.273 * 0.5, 0.273 * 0.4, 0.273 * 0.3, 0.273 * 0.2, 0.273 * 0.1, 0.273 };
  s_vel.push_back(end_velocity);

  urcl::vector6d_t end_accleration = { 1.713 * 0.5, 1.713 * 0.4, 1.713 * 0.3, 1.713 * 0.2, 1.713 * 0.1, 1.713 };
  s_acc.push_back(end_accleration);

  std::vector<double> s_time = { 2.0 };

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::vector<urcl::vector6d_t> coefficients =
      createSegment(s_time[0], joint_positions, s_pos[0], joint_velocities, s_vel[0], zeros, s_acc[0]);

  // Data for logging
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, expected_positions;
  std::vector<double> time_vec;

  // Send trajectory to the robot
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_trajectory_running_ = true;

  // Make sure that trajectory has started before we start testing for trajectory execution
  waitForTrajectoryStarted();

  double old_spline_travel_time = 0.0;
  double plot_time = 0.0;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

    // Read spline travel time from the robot
    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    double spline_travel_step_time = spline_travel_time - old_spline_travel_time;
    old_spline_travel_time = spline_travel_time;

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - spline_travel_step_time);
    double time_left = s_time[0] - spline_travel_time;

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));

    // Ensure that we follow the joint trajectory
    urcl::vector6d_t expected_joint_positions;
    interpolate(spline_travel_time, expected_joint_positions, coefficients);
    for (unsigned int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(expected_joint_positions[i], joint_positions[i], eps_);
    }

    if (time_left < deceleration_time_)
    {
      double max_speed = max_deceleration_ * deceleration_time_;
      double max_allowable_speed = max_speed - (deceleration_time_ - time_left) * max_deceleration_;

      for (unsigned int i = 0; i < 6; ++i)
      {
        // Test that the robot stays within the maximum allowed speed. At very low velocities we might see the joint
        // velocity being slightly larger than the max_allowable_speed, which is the motivation for the check. But if
        // the robot doesn't slow down as intended it will be cached by this check.
        EXPECT_LT(std::abs(joint_velocities[i]) - max_allowable_speed, 0.01);
      }
    }
    expected_positions.push_back(expected_joint_positions);
    actual_positions.push_back(joint_positions);
    actual_velocities.push_back(joint_velocities);
    time_vec.push_back(plot_time);
    plot_time += step_time_;
  }

  // Make sure the velocity is zero when the trajectory has finished
  readDataPackage(data_pkg);
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_velocities[i], 0.0);
  }

  std::ofstream outfile("../test_artifacts/quintic_spline_with_end_point_velocity.txt");
  for (unsigned int i = 0; i < actual_positions.size(); ++i)
  {
    outfile << time_vec[i] << "," << actual_positions[i][0] << "," << actual_positions[i][1] << ","
            << actual_positions[i][2] << "," << actual_positions[i][3] << "," << actual_positions[i][4] << ","
            << actual_positions[i][5] << "," << actual_velocities[i][0] << "," << actual_velocities[i][1] << ","
            << actual_velocities[i][2] << "," << actual_velocities[i][3] << "," << actual_velocities[i][4] << ","
            << actual_velocities[i][5] << "," << expected_positions[i][0] << "," << expected_positions[i][1] << ","
            << expected_positions[i][2] << "," << expected_positions[i][3] << "," << expected_positions[i][4] << ","
            << expected_positions[i][5] << "\n";
  }
}

TEST_F(SplineInterpolationTest, spline_interpolation_cubic)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_ur_driver_->getDataPackage();
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }
  urcl::vector6d_t joint_positions;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  urcl::vector6d_t joint_velocities;
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

  std::vector<urcl::vector6d_t> s_pos, s_vel;
  urcl::vector6d_t first_point = { joint_positions[0], joint_positions[1], joint_positions[2],
                                   joint_positions[3], joint_positions[4], 4.15364583e-03 };
  urcl::vector6d_t second_point = { joint_positions[0], joint_positions[1], joint_positions[2],
                                    joint_positions[3], joint_positions[4], -1.74088542e-02 };
  s_pos.push_back(first_point);
  s_pos.push_back(second_point);
  s_pos.push_back(joint_positions);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t first_velocity = { 0.0, 0.0, 0.0, 0.0, 0.0, -2.01015625e-01 };
  urcl::vector6d_t second_velocity = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.72812500e-01 };
  s_vel.push_back(first_velocity);
  s_vel.push_back(second_velocity);
  s_vel.push_back(zeros);

  std::vector<double> s_time = { 1.0000000e+00, 5.00100000e+00, 4.00000000e+00 };

  // Build first segment
  std::vector<urcl::vector6d_t> coefficients =
      createSegment(s_time[0], joint_positions, s_pos[0], joint_velocities, s_vel[0]);

  // Data for logging
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, expected_positions;
  std::vector<double> time_vec;

  // Send the trajectory to the robot
  sendTrajectory(s_pos, s_vel, std::vector<urcl::vector6d_t>(), s_time);
  g_trajectory_running_ = true;

  // Make sure that trajectory has started before we start testing for trajectory execution
  waitForTrajectoryStarted();

  int segment_idx = 0;
  double old_spline_travel_time = 0.0;
  double plot_time = 0.0;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - step_time_);

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));

    if (old_spline_travel_time > spline_travel_time)
    {
      // New segment started
      segment_idx++;
      coefficients =
          createSegment(s_time[segment_idx], joint_positions, s_pos[segment_idx], joint_velocities, s_vel[segment_idx]);
    }
    old_spline_travel_time = spline_travel_time;

    urcl::vector6d_t expected_joint_positions;
    interpolate(spline_travel_time, expected_joint_positions, coefficients);

    for (unsigned int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(expected_joint_positions[i], joint_positions[i], eps_);
    }

    expected_positions.push_back(expected_joint_positions);
    actual_positions.push_back(joint_positions);
    actual_velocities.push_back(joint_velocities);
    time_vec.push_back(plot_time);
    plot_time += step_time_;
  }

  std::ofstream outfile("../test_artifacts/spline_interpolation_cubic.txt");
  for (unsigned int i = 0; i < actual_positions.size(); ++i)
  {
    outfile << time_vec[i] << "," << actual_positions[i][0] << "," << actual_positions[i][1] << ","
            << actual_positions[i][2] << "," << actual_positions[i][3] << "," << actual_positions[i][4] << ","
            << actual_positions[i][5] << "," << actual_velocities[i][0] << "," << actual_velocities[i][1] << ","
            << actual_velocities[i][2] << "," << actual_velocities[i][3] << "," << actual_velocities[i][4] << ","
            << actual_velocities[i][5] << "," << expected_positions[i][0] << "," << expected_positions[i][1] << ","
            << expected_positions[i][2] << "," << expected_positions[i][3] << "," << expected_positions[i][4] << ","
            << expected_positions[i][5] << "\n";
  }
}

TEST_F(SplineInterpolationTest, spline_interpolation_quintic)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions, joint_velocities;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));

  std::vector<urcl::vector6d_t> s_pos, s_vel, s_acc;
  urcl::vector6d_t first_point = { joint_positions[0], joint_positions[1], joint_positions[2],
                                   joint_positions[3], joint_positions[4], 4.15364583e-03 };
  urcl::vector6d_t second_point = { joint_positions[0], joint_positions[1], joint_positions[2],
                                    joint_positions[3], joint_positions[4], -1.74088542e-02 };
  s_pos.push_back(first_point);
  s_pos.push_back(second_point);
  s_pos.push_back(joint_positions);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t first_velocity = { 0.0, 0.0, 0.0, 0.0, 0.0, -2.01015625e-01 };
  urcl::vector6d_t second_velocity = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.72812500e-01 };
  s_vel.push_back(first_velocity);
  s_vel.push_back(second_velocity);
  s_vel.push_back(zeros);

  urcl::vector6d_t first_acceleration = { 0.0, 0.0, 0.0, 0.0, 0.0, 2.55885417e+00 };
  urcl::vector6d_t second_acceleration = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.71276042e+00 };
  s_acc.push_back(first_acceleration);
  s_acc.push_back(second_acceleration);
  s_acc.push_back(zeros);

  std::vector<double> s_time = { 1.0000000e+00, 5.00100000e+00, 4.00000000e+00 };

  // Build first segment
  std::vector<urcl::vector6d_t> coefficients =
      createSegment(s_time[0], joint_positions, s_pos[0], joint_velocities, s_vel[0], zeros, s_acc[0]);

  // Data for logging
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, expected_positions;
  std::vector<double> time_vec;

  // Send the trajectory to the robot
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_trajectory_running_ = true;

  // Make sure that trajectory has started before we start testing for trajectory execution
  waitForTrajectoryStarted();

  int segment_idx = 0;
  double old_spline_travel_time = 0.0;
  double plot_time = 0.0;
  g_trajectory_running_ = true;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - step_time_);

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeJointCommand(vector6d_t(), comm::ControlMode::MODE_FORWARD));

    if (old_spline_travel_time > spline_travel_time)
    {
      // New segment started
      segment_idx++;
      coefficients = createSegment(s_time[segment_idx], joint_positions, s_pos[segment_idx], joint_velocities,
                                   s_vel[segment_idx], s_acc[segment_idx - 1], s_acc[segment_idx]);
    }
    old_spline_travel_time = spline_travel_time;

    urcl::vector6d_t expected_joint_positions;
    interpolate(spline_travel_time, expected_joint_positions, coefficients);

    for (unsigned int i = 0; i < 6; ++i)
    {
      EXPECT_NEAR(expected_joint_positions[i], joint_positions[i], eps_);
    }
    expected_positions.push_back(expected_joint_positions);
    actual_positions.push_back(joint_positions);
    actual_velocities.push_back(joint_velocities);
    time_vec.push_back(plot_time);
    plot_time += step_time_;
  }

  std::ofstream outfile("../test_artifacts/spline_interpolation_quintic.txt");
  for (unsigned int i = 0; i < actual_positions.size(); ++i)
  {
    outfile << time_vec[i] << "," << actual_positions[i][0] << "," << actual_positions[i][1] << ","
            << actual_positions[i][2] << "," << actual_positions[i][3] << "," << actual_positions[i][4] << ","
            << actual_positions[i][5] << "," << actual_velocities[i][0] << "," << actual_velocities[i][1] << ","
            << actual_velocities[i][2] << "," << actual_velocities[i][3] << "," << actual_velocities[i][4] << ","
            << actual_velocities[i][5] << "," << expected_positions[i][0] << "," << expected_positions[i][1] << ","
            << expected_positions[i][2] << "," << expected_positions[i][3] << "," << expected_positions[i][4] << ","
            << expected_positions[i][5] << "\n";
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      ROBOT_IP = argv[i + 1];
      break;
    }
  }

  return RUN_ALL_TESTS();
}