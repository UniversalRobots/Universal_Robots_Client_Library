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
#include <ostream>
#include <thread>

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string SPLINE_SCRIPT_FILE = "spline_external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe_spline.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
std::string ROBOT_IP = "192.168.56.101";

std::unique_ptr<UrDriver> g_ur_driver_;
std::unique_ptr<DashboardClient> g_dashboard_client_;

bool g_program_running;
std::condition_variable g_program_not_running_cv_;
std::mutex g_program_not_running_mutex_;
std::condition_variable g_program_running_cv_;
std::mutex g_program_running_mutex_;

// Helper functions for the driver
void handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
  if (program_running)
  {
    std::lock_guard<std::mutex> lk(g_program_running_mutex_);
    g_program_running = program_running;
    g_program_running_cv_.notify_one();
  }
  else
  {
    std::lock_guard<std::mutex> lk(g_program_not_running_mutex_);
    g_program_running = program_running;
    g_program_not_running_cv_.notify_one();
  }
}

bool g_trajectory_running_;
control::TrajectoryResult g_trajectory_result_;
void handleTrajectoryState(control::TrajectoryResult state)
{
  g_trajectory_result_ = state;
  g_trajectory_running_ = false;
}

bool g_rtde_read_thread_running_ = false;
bool g_consume_rtde_packages_ = false;
std::mutex g_read_package_mutex_;
std::thread g_rtde_read_thread;

void rtdeConsumeThread()
{
  while (g_rtde_read_thread_running_)
  {
    // Consume package to prevent pipeline overflow
    if (g_consume_rtde_packages_ == true)
    {
      std::lock_guard<std::mutex> lk(g_read_package_mutex_);
      std::unique_ptr<rtde_interface::DataPackage> data_pkg;
      data_pkg = g_ur_driver_->getDataPackage();
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

int sign(double val)
{
  return (0.0 < val) - (val < 0.0);
}

bool nearly_equal(double a, double b, double eps = 1e-15)
{
  const double c(a - b);
  return c < eps || -c < eps;
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
    try
    {
      g_ur_driver_.reset(new UrDriver(ROBOT_IP, SPLINE_SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE,
                                      &handleRobotProgramState, HEADLESS, std::move(tool_comm_setup),
                                      CALIBRATION_CHECKSUM));
    }
    catch (UrException& exp)
    {
      std::cout << "caught exception " << exp.what() << " while launch driver, retrying once in 10 seconds"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(10));
      g_ur_driver_.reset(new UrDriver(ROBOT_IP, SPLINE_SCRIPT_FILE, OUTPUT_RECIPE, INPUT_RECIPE,
                                      &handleRobotProgramState, HEADLESS, std::move(tool_comm_setup),
                                      CALIBRATION_CHECKSUM));
    }

    g_ur_driver_->registerTrajectoryDoneCallback(&handleTrajectoryState);

    g_ur_driver_->startRTDECommunication();
    // Setup rtde read thread
    g_rtde_read_thread_running_ = true;
    g_rtde_read_thread = std::thread(rtdeConsumeThread);
  }

  static void TearDownTestSuite()
  {
    // Set target speed scaling to 100% as one test change this value
    g_ur_driver_->getRTDEWriter().sendSpeedSlider(1);

    g_rtde_read_thread_running_ = false;
    g_rtde_read_thread.join();
    g_dashboard_client_->disconnect();
    // Remove temporary file again
    std::remove(SPLINE_SCRIPT_FILE.c_str());
  }

  void TearDown()
  {
    // Set target speed scaling to 100% as one test change this value
    g_ur_driver_->getRTDEWriter().sendSpeedSlider(1);
  }

  void SetUp()
  {
    step_time_ = 0.002;
    if (g_ur_driver_->getVersion().major < 5)
    {
      step_time_ = 0.008;
    }
    // Make sure script is running on the robot
    if (g_program_running == false)
    {
      g_consume_rtde_packages_ = true;
      g_ur_driver_->sendRobotProgram();
      ASSERT_TRUE(waitForProgramRunning(1000));
    }
    g_consume_rtde_packages_ = false;
  }

  void sendIdle()
  {
    ASSERT_TRUE(g_ur_driver_->writeKeepalive(RobotReceiveTimeout::sec()));
  }

  void sendTrajectory(const std::vector<urcl::vector6d_t>& s_pos, const std::vector<urcl::vector6d_t>& s_vel,
                      const std::vector<urcl::vector6d_t>& s_acc, const std::vector<double>& s_time)
  {
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));

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
      ASSERT_TRUE(
          g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));
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

  bool waitForProgramRunning(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(g_program_running_mutex_);
    if (g_program_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        g_program_running == true)
    {
      return true;
    }
    return false;
  }

  bool waitForProgramNotRunning(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(g_program_not_running_mutex_);
    if (g_program_not_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        g_program_running == false)
    {
      return true;
    }
    return false;
  }

  void readDataPackage(std::unique_ptr<rtde_interface::DataPackage>& data_pkg)
  {
    if (g_consume_rtde_packages_ == true)
    {
      URCL_LOG_ERROR("Unable to read packages while consuming, this should not happen!");
      GTEST_FAIL();
    }
    std::lock_guard<std::mutex> lk(g_read_package_mutex_);
    data_pkg = g_ur_driver_->getDataPackage();
    if (data_pkg == nullptr)
    {
      URCL_LOG_ERROR("Timed out waiting for a new package from the robot");
      GTEST_FAIL();
    }
  }

  void writeTrajectoryToFile(const char* filename, std::vector<double> time_vec,
                             std::vector<urcl::vector6d_t> expected_positions,
                             std::vector<urcl::vector6d_t> actual_positions,
                             std::vector<urcl::vector6d_t> actual_velocities, std::vector<urcl::vector6d_t> actual_acc,
                             std::vector<double> speed_scaling, std::vector<double> spline_time)
  {
    std::ofstream outfile(filename);
    // Header
    outfile << "time, "
            << "actual_positions0, "
            << "actual_positions1, "
            << "actual_positions2, "
            << "actual_positions3, "
            << "actual_positions4, "
            << "actual_positions5, "
            << "actual_velocities0, "
            << "actual_velocities1, "
            << "actual_velocities2, "
            << "actual_velocities3, "
            << "actual_velocities4, "
            << "actual_velocities5, "
            << "actual_acceleration0, "
            << "actual_acceleration1, "
            << "actual_acceleration2, "
            << "actual_acceleration3, "
            << "actual_acceleration4, "
            << "actual_acceleration5, "
            << "error_positions0, "
            << "error_positions1, "
            << "error_positions2, "
            << "error_positions3, "
            << "error_positions4, "
            << "error_positions5, "
            << "speed_scaling, "
            << "spline_time"
            << "\n";

    // Data
    for (unsigned int i = 0; i < actual_positions.size(); ++i)
    {
      outfile << time_vec[i] << ", " << actual_positions[i][0] << ", " << actual_positions[i][1] << ", "
              << actual_positions[i][2] << ", " << actual_positions[i][3] << ", " << actual_positions[i][4] << ", "
              << actual_positions[i][5] << ", " << actual_velocities[i][0] << ", " << actual_velocities[i][1] << ", "
              << actual_velocities[i][2] << ", " << actual_velocities[i][3] << ", " << actual_velocities[i][4] << ", "
              << actual_velocities[i][5] << ", " << actual_acc[i][0] << ", " << actual_acc[i][1] << ", "
              << actual_acc[i][2] << ", " << actual_acc[i][3] << ", " << actual_acc[i][4] << ", " << actual_acc[i][5]
              << ", " << actual_positions[i][0] - expected_positions[i][0] << ", "
              << actual_positions[i][1] - expected_positions[i][1] << ", "
              << actual_positions[i][2] - expected_positions[i][2] << ", "
              << actual_positions[i][3] - expected_positions[i][3] << ", "
              << actual_positions[i][4] - expected_positions[i][4] << ", "
              << actual_positions[i][5] - expected_positions[i][5] << ", " << speed_scaling[i] << ", " << spline_time[i]
              << "\n";
    }
  }

  // Allowed difference between expected trajectory and actual robot trajectory
  const double eps_ = 0.02;

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

  urcl::vector6d_t joint_positions, joint_velocities, joint_acc;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));

  double speed_scaling;
  ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));
  std::vector<double> speed_scaling_vec;

  std::vector<urcl::vector6d_t> s_pos, s_vel;
  s_pos.push_back(joint_positions);

  urcl::vector6d_t end_velocity = { 0.273 * 0.5, 0.273 * 0.4, 0.273 * 0.3, 0.273 * 0.2, 0.273 * 0.1, 0.273 };
  s_vel.push_back(end_velocity);

  std::vector<double> s_time = { 2.0000000e+00 };

  // Build the segment
  std::vector<urcl::vector6d_t> coefficients =
      createSegment(s_time[0], joint_positions, s_pos[0], joint_velocities, s_vel[0]);

  // Data for logging
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, actual_acc, expected_positions;
  std::vector<double> time_vec, spline_time;

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
    ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));
    ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));

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
    actual_acc.push_back(joint_acc);
    speed_scaling_vec.push_back(speed_scaling);
    time_vec.push_back(plot_time);
    spline_time.push_back(spline_travel_time);
    plot_time += step_time_;
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  // Make sure the velocity is zero when the trajectory has finished
  readDataPackage(data_pkg);
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_velocities[i], 0.0);
  }

  // Verify that the full trajectory have been executed
  double spline_travel_time;
  data_pkg->getData("output_double_register_1", spline_travel_time);
  ASSERT_NEAR(spline_travel_time, s_time.back(), 1e-5);

  writeTrajectoryToFile("../test_artifacts/cubic_spline_with_end_point_velocity.csv", time_vec, expected_positions,
                        actual_positions, actual_velocities, actual_acc, speed_scaling_vec, spline_time);
}

TEST_F(SplineInterpolationTest, quintic_spline_with_end_point_velocity_with_speedscaling)
{
  // Set speed scaling to 25% to test interpolation with speed scaling active
  const unsigned int REDUSE_FACTOR(4);
  g_ur_driver_->getRTDEWriter().sendSpeedSlider(1.0 / REDUSE_FACTOR);

  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  // Align timestep
  sendIdle();
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions, joint_velocities, joint_acc;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));

  double speed_scaling;
  ASSERT_TRUE(data_pkg->getData("target_speed_fraction", speed_scaling));
  std::vector<double> speed_scaling_vec;

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
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, actual_acc, expected_positions;
  std::vector<double> time_vec, spline_time;

  // Send trajectory to the robot
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_trajectory_running_ = true;

  // Make sure that trajectory has started before we start testing for trajectory execution
  waitForTrajectoryStarted();

  double old_spline_travel_time = 0.0;
  double plot_time = 0.0;
  unsigned int loop_count = 0;
  bool init_acc_test = true;
  urcl::vector6d_t last_joint_acc = joint_acc, last_change_acc;
  const double EPS_ACC_CHANGE(1e-15);
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
    ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));
    ASSERT_TRUE(data_pkg->getData("target_speed_fraction", speed_scaling));

    // Read spline travel time from the robot
    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    double spline_travel_step_time = spline_travel_time - old_spline_travel_time;
    old_spline_travel_time = spline_travel_time;

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - spline_travel_step_time);
    double time_left = s_time[0] - spline_travel_time;

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));

    // Ensure that we follow the joint trajectory
    urcl::vector6d_t expected_joint_positions, change_acc;
    interpolate(spline_travel_time, expected_joint_positions, coefficients);

    if (init_acc_test && loop_count == 0)
    {
      last_joint_acc = joint_acc;
    }
    else if (init_acc_test && last_joint_acc != joint_acc)
    {
      init_acc_test = false;
      loop_count = 1;
      last_joint_acc = joint_acc;
      last_change_acc.fill(0.0);
    }

    if (loop_count % REDUSE_FACTOR == 0)
    {
      last_joint_acc = joint_acc;
      last_change_acc.fill(0.0);
    }
    else
    {
      for (unsigned int i = 0; i < last_joint_acc.size(); ++i)
      {
        change_acc[i] = joint_acc[i] - last_joint_acc[i];

        if (!nearly_equal(change_acc[i], 0.0, EPS_ACC_CHANGE) && !nearly_equal(last_change_acc[i], 0.0, EPS_ACC_CHANGE))
        {
          // Acceleration should only increase or be constant within one scaled timescale.
          // It should not fluctuate to zero or overshoot
          EXPECT_EQ(sign(last_change_acc[i]), sign(change_acc[i]))
              << " acceleration change direction doing "
                 "one scaled step"
              << loop_count << " Numbers:\n"
              << last_change_acc[i] << " | " << change_acc[i] << "\n";
        }
      }
      last_change_acc = change_acc;
    }

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
    actual_acc.push_back(joint_acc);
    speed_scaling_vec.push_back(speed_scaling);
    time_vec.push_back(plot_time);
    spline_time.push_back(spline_travel_time);
    plot_time += step_time_;
    loop_count += 1;
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  // Make sure the velocity is zero when the trajectory has finished
  readDataPackage(data_pkg);
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_velocities[i], 0.0);
  }

  // Verify that the full trajectory have been executed
  double spline_travel_time;
  data_pkg->getData("output_double_register_1", spline_travel_time);
  ASSERT_NEAR(spline_travel_time, s_time.back(), 1e-5);

  writeTrajectoryToFile("../test_artifacts/quintic_spline_with_end_point_velocity_speedscaling.csv", time_vec,
                        expected_positions, actual_positions, actual_velocities, actual_acc, speed_scaling_vec,
                        spline_time);
}

TEST_F(SplineInterpolationTest, spline_interpolation_cubic)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions, joint_velocities, joint_acc;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));

  double speed_scaling;
  ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));
  std::vector<double> speed_scaling_vec;

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
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, actual_acc, expected_positions;
  std::vector<double> time_vec, spline_time;

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
    ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));
    ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));

    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - step_time_);

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));

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
    actual_acc.push_back(joint_acc);
    speed_scaling_vec.push_back(speed_scaling);
    time_vec.push_back(plot_time);
    spline_time.push_back(spline_travel_time);
    plot_time += step_time_;
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  // Verify that the full trajectory have been executed
  double spline_travel_time;
  data_pkg->getData("output_double_register_1", spline_travel_time);
  ASSERT_NEAR(spline_travel_time, s_time.back(), 1e-5);

  writeTrajectoryToFile("../test_artifacts/spline_interpolation_cubic.csv", time_vec, expected_positions,
                        actual_positions, actual_velocities, actual_acc, speed_scaling_vec, spline_time);
}

TEST_F(SplineInterpolationTest, spline_interpolation_quintic)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions, joint_velocities, joint_acc;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
  ASSERT_TRUE(data_pkg->getData("target_qd", joint_velocities));
  ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));

  double speed_scaling;
  ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));
  std::vector<double> speed_scaling_vec;

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
  std::vector<urcl::vector6d_t> actual_positions, actual_velocities, actual_acc, expected_positions;
  std::vector<double> time_vec, spline_time;

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
    ASSERT_TRUE(data_pkg->getData("target_qdd", joint_acc));
    ASSERT_TRUE(data_pkg->getData("speed_scaling", speed_scaling));

    double spline_travel_time = 0.0;
    data_pkg->getData("output_double_register_1", spline_travel_time);

    // The data received from the robot is one time step behind where the robot is interpolating
    spline_travel_time = (spline_travel_time == 0 ? spline_travel_time : spline_travel_time - step_time_);

    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));

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
    actual_acc.push_back(joint_acc);
    speed_scaling_vec.push_back(speed_scaling);
    time_vec.push_back(plot_time);
    spline_time.push_back(spline_travel_time);
    plot_time += step_time_;
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  // Verify that the full trajectory have been executed
  double spline_travel_time;
  data_pkg->getData("output_double_register_1", spline_travel_time);
  ASSERT_NEAR(spline_travel_time, s_time.back(), 1e-5);

  writeTrajectoryToFile("../test_artifacts/spline_interpolation_quintic.csv", time_vec, expected_positions,
                        actual_positions, actual_velocities, actual_acc, speed_scaling_vec, spline_time);
}

TEST_F(SplineInterpolationTest, zero_time_trajectory_cubic_spline)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions_before;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_before));

  // Start consuming rtde packages to avoid pipeline overflows while testing that the control script aborts correctly
  g_consume_rtde_packages_ = true;

  // Create illegal trajectory
  std::vector<urcl::vector6d_t> s_pos, s_vel;
  urcl::vector6d_t first_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.1
  };
  urcl::vector6d_t second_point = { joint_positions_before[0], joint_positions_before[1],
                                    joint_positions_before[2], joint_positions_before[3],
                                    joint_positions_before[4], joint_positions_before[5] - 0.5 };
  s_pos.push_back(first_point);
  s_pos.push_back(second_point);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  s_vel.push_back(zeros);
  s_vel.push_back(zeros);

  std::vector<double> s_time = { 0.0, 1.0 };

  // Send illegal trajectory to the robot
  sendTrajectory(s_pos, s_vel, std::vector<urcl::vector6d_t>(), s_time);
  g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::off());

  // When an illegal trajectory is send to the robot, the control script should keep running but the trajectory result
  // should be canceled.
  ASSERT_FALSE(waitForProgramNotRunning(1000));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED, g_trajectory_result_);

  // Stop consuming rtde packages
  g_consume_rtde_packages_ = false;

  // Ensure that the robot hasn't moved
  readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions_after;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_positions_before[i], joint_positions_after[i]);
  }

  // If the first point is very close to the current position, then a 0 time for that is fine
  first_point = { joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
                  joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] };
  second_point = { -1.6, -1.72, -2.175, -0.78, 1.623, -0.5 };
  urcl::vector6d_t third_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.1
  };
  s_pos = { first_point, second_point, third_point };
  s_vel = { zeros, zeros, zeros };
  s_time = { 0.0, 1.0, 2.0 };
  sendTrajectory(s_pos, s_vel, std::vector<urcl::vector6d_t>(), s_time);
  g_trajectory_running_ = true;
  waitForTrajectoryStarted();
  urcl::vector6d_t joint_positions;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(joint_positions[i], third_point[i], eps_);
  }
}

TEST_F(SplineInterpolationTest, zero_time_trajectory_quintic_spline)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions_before;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_before));

  // Start consuming rtde packages to avoid pipeline overflows while testing
  g_consume_rtde_packages_ = true;

  // Create illegal trajectory
  std::vector<urcl::vector6d_t> s_pos, s_vel, s_acc;
  urcl::vector6d_t first_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.2
  };
  s_pos.push_back(first_point);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  s_vel.push_back(zeros);
  s_acc.push_back(zeros);

  std::vector<double> s_time = { 0.0 };

  // Send illegal trajectory to the robot
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::off());

  // When an illegal trajectory is send to the robot, the control script should keep running but the trajectory result
  // should be canceled
  ASSERT_FALSE(waitForProgramNotRunning(1000));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED, g_trajectory_result_);

  // Stop consuming rtde packages
  g_consume_rtde_packages_ = false;

  // Ensure that the robot hasn't moved
  readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions_after;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_positions_before[i], joint_positions_after[i]);
  }
  //
  // If the first point is very close to the current position, then a 0 time for that is fine
  first_point = { joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
                  joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] };
  urcl::vector6d_t second_point = { -1.6, -1.72, -2.175, -0.78, 1.623, -0.5 };
  urcl::vector6d_t third_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.1
  };
  s_pos = { first_point, second_point, third_point };
  s_vel = { zeros, zeros, zeros };
  s_acc = { zeros, zeros, zeros };
  s_time = { 0.0, 1.0, 2.0 };
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_trajectory_running_ = true;
  waitForTrajectoryStarted();
  urcl::vector6d_t joint_positions;
  while (g_trajectory_running_)
  {
    readDataPackage(data_pkg);
    ASSERT_TRUE(data_pkg->getData("target_q", joint_positions));
    // Keep connection alive
    ASSERT_TRUE(g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP));
  }
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, g_trajectory_result_);

  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_NEAR(joint_positions[i], third_point[i], eps_);
  }
}

TEST_F(SplineInterpolationTest, physically_unfeasible_trajectory_cubic_spline)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions_before;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_before));

  // Start consuming rtde packages to avoid pipeline overflows while testing that the control script aborts correctly
  g_consume_rtde_packages_ = true;

  // Create a trajectory that cannot be executed within the robots limits
  std::vector<urcl::vector6d_t> s_pos, s_vel;
  urcl::vector6d_t first_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.5
  };
  s_pos.push_back(first_point);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  s_vel.push_back(zeros);

  std::vector<double> s_time = { 0.02 };

  // Send unfeasible trajectory to the robot
  sendTrajectory(s_pos, s_vel, std::vector<urcl::vector6d_t>(), s_time);
  g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::off());

  // When an unfeasible trajectory is send to the robot, the control script should keep running but the trajectory
  // result should be canceled
  ASSERT_FALSE(waitForProgramNotRunning(1000));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED, g_trajectory_result_);

  // Stop consuming rtde packages
  g_consume_rtde_packages_ = false;

  // Ensure that the robot hasn't moved
  readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions_after;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_positions_before[i], joint_positions_after[i]);
  }
}

TEST_F(SplineInterpolationTest, physically_unfeasible_trajectory_quintic_spline)
{
  std::unique_ptr<rtde_interface::DataPackage> data_pkg;
  readDataPackage(data_pkg);

  urcl::vector6d_t joint_positions_before;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_before));

  // Start consuming rtde packages to avoid pipeline overflows while testing that the control script aborts correctly
  g_consume_rtde_packages_ = true;

  // Create a trajectory that cannot be executed within the robots limits
  std::vector<urcl::vector6d_t> s_pos, s_vel, s_acc;
  urcl::vector6d_t first_point = {
    joint_positions_before[0], joint_positions_before[1], joint_positions_before[2],
    joint_positions_before[3], joint_positions_before[4], joint_positions_before[5] + 0.5
  };
  s_pos.push_back(first_point);

  urcl::vector6d_t zeros = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  s_vel.push_back(zeros);
  s_acc.push_back(zeros);

  std::vector<double> s_time = { 0.02 };

  // Send unfeasible trajectory to the robot
  sendTrajectory(s_pos, s_vel, s_acc, s_time);
  g_ur_driver_->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP, -1,
                                              RobotReceiveTimeout::off());

  // When an unfeasible trajectory is send to the robot, the control script should keep running but the trajectory
  // result should be canceled
  ASSERT_FALSE(waitForProgramNotRunning(1000));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED, g_trajectory_result_);

  // Stop consuming rtde packages
  g_consume_rtde_packages_ = false;

  // Ensure that the robot hasn't moved
  readDataPackage(data_pkg);
  urcl::vector6d_t joint_positions_after;
  ASSERT_TRUE(data_pkg->getData("target_q", joint_positions_after));
  for (unsigned int i = 0; i < 6; ++i)
  {
    EXPECT_FLOAT_EQ(joint_positions_before[i], joint_positions_after[i]);
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
