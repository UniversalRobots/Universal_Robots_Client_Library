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

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/example_robot_wrapper.h"
#include "ur_client_library/log.h"
#include "ur_client_library/types.h"

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

std::condition_variable g_trajectory_result_cv;
std::mutex g_trajectory_result_mutex;
control::TrajectoryResult g_trajectory_result = control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;
bool g_trajectory_result_received = false;

void handleTrajectoryState(control::TrajectoryResult state)
{
  std::lock_guard<std::mutex> lk(g_trajectory_result_mutex);
  g_trajectory_result = state;
  g_trajectory_result_received = true;
  g_trajectory_result_cv.notify_one();
  URCL_LOG_INFO("Received trajectory result %s", control::trajectoryResultToString(state).c_str());
}

class TrajectoryStreamingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    // Streaming tests are headless only. The URCap path doesn't exercise
    // anything different at the URScript level for streaming, so there is
    // no value in maintaining a separate _urcap variant.
    ASSERT_TRUE(g_HEADLESS) << "trajectory streaming tests require headless mode";

    g_my_robot = std::make_unique<ExampleRobotWrapper>(g_ROBOT_IP, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS,
                                                       "external_control.urp", SCRIPT_FILE);
    ASSERT_TRUE(g_my_robot->isHealthy());
    g_my_robot->getUrDriver()->registerTrajectoryDoneCallback(&handleTrajectoryState);
  }

  void SetUp() override
  {
    if (!g_my_robot->isHealthy())
    {
      ASSERT_TRUE(g_my_robot->resendRobotProgram());
      ASSERT_TRUE(g_my_robot->waitForProgramRunning(500));
    }
    resetTrajectoryResultState();
  }

  static void resetTrajectoryResultState()
  {
    std::lock_guard<std::mutex> lk(g_trajectory_result_mutex);
    g_trajectory_result = control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;
    g_trajectory_result_received = false;
  }

  // Pump TRAJECTORY_NOOP on the reverse socket so the urscript's main
  // dispatcher read does not time out while we wait for the trajectory
  // result callback. Returns the captured result, or
  // TRAJECTORY_RESULT_UNKNOWN if the deadline elapsed first.
  control::TrajectoryResult waitForTrajectoryResultPumpingNoops(std::chrono::milliseconds total_timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + total_timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::unique_lock<std::mutex> lk(g_trajectory_result_mutex);
      if (g_trajectory_result_cv.wait_for(lk, std::chrono::milliseconds(100),
                                          [] { return g_trajectory_result_received; }))
      {
        return g_trajectory_result;
      }
      lk.unlock();
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
          control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
    return control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;
  }
};

// Clean end-to-end stream: STREAM_START, N spline points at a held pose,
// STREAM_END(N). Expect TRAJECTORY_RESULT_SUCCESS via the end callback.
// All points sit at the same pose so the test exercises the streaming
// protocol without exercising motion dynamics.
TEST_F(TrajectoryStreamingTest, stream_end_yields_success)
{
  const vector6d_t held_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Position the robot at held_pose with a one-shot finite trajectory.
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Stream: STREAM_START, N points, STREAM_END(N).
  const int k_num_points = 50;
  const float k_step_time = 0.01f;
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, k_step_time));
  }
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// STREAM_START with no spline points and no STREAM_END. The trajectoryThread's
// first socket_read on the trajectory socket times out (0.5s before any motion),
// trips the underrun else-branch with both was_streaming_at_start and
// trajectory_streaming still True, and emits TRAJECTORY_RESULT_FAILURE.
TEST_F(TrajectoryStreamingTest, stream_underrun_yields_failure)
{
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(3)));
}

// STREAM_START followed by points, then TRAJECTORY_CANCEL before STREAM_END.
// The existing cancel pathway tears down the trajectory thread, drains
// remaining points via clearTrajectoryPointsThread (whose loop terminates
// when the trajectory socket goes silent on first timeout), and emits
// TRAJECTORY_RESULT_CANCELED.
TEST_F(TrajectoryStreamingTest, stream_cancel_yields_canceled)
{
  const vector6d_t held_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  // Pile a batch of points into the OS socket buffer so the cleanup path
  // has something to drain.
  for (int i = 0; i < 100; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, 0.01f));
  }
  // Give the trajectoryThread a moment to consume a handful of points so
  // the cancel hits mid-stream rather than racing the spawn.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_CANCEL));

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      g_ROBOT_IP = argv[i + 1];
      ++i;
    }
    if (std::string(argv[i]) == "--headless" && i + 1 < argc)
    {
      std::string headless = argv[i + 1];
      g_HEADLESS = headless == "true" || headless == "1" || headless == "True" || headless == "TRUE";
      ++i;
    }
  }

  return RUN_ALL_TESTS();
}
