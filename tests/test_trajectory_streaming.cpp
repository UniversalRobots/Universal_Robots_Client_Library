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
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/example_robot_wrapper.h"
#include "ur_client_library/log.h"
#include "ur_client_library/rtde/data_package.h"
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
    // The tests which assert on motion read the joint positions from RTDE, and
    // getDataPackage() fails immediately unless the background reader has been
    // started.
    g_my_robot->startRTDECommununication(true);
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

  // Read the current joint positions of the arm. The output recipe this fixture
  // uses already carries actual_q, so observing motion requires no additional
  // configuration. We report a read which produced nothing rather than asserting
  // on it, so that a caller polling for a position can treat it as a reason to
  // try again instead of as a failure.
  static bool readJointPositions(vector6d_t& joint_positions)
  {
    rtde_interface::DataPackage data_pkg(g_my_robot->getUrDriver()->getRTDEOutputRecipe());
    if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
    {
      return false;
    }
    return data_pkg.getData("actual_q", joint_positions);
  }

  static bool jointsNear(const vector6d_t& lhs, const vector6d_t& rhs, double tolerance)
  {
    for (size_t i = 0; i < lhs.size(); ++i)
    {
      if (std::fabs(lhs[i] - rhs[i]) > tolerance)
      {
        return false;
      }
    }
    return true;
  }

  // Poll until every joint lies within `tolerance` of `target`, or until the
  // deadline passes, and return the last sample we read in either case. The
  // background reader returns the newest package it has seen without blocking,
  // so this loop has to pace itself. We return the sample rather than a bool
  // because callers follow this with EXPECT_NEAR, which then reports the
  // position the arm actually reached when it fails.
  static vector6d_t waitForJointsNear(const vector6d_t& target, double tolerance, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    vector6d_t joint_positions = { 0, 0, 0, 0, 0, 0 };
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (readJointPositions(joint_positions) && jointsNear(joint_positions, target, tolerance))
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return joint_positions;
  }

  // Assert that the arm came to rest at `target`. We have to assert on motion
  // because the trajectory result on its own cannot distinguish a move which ran
  // from a move which reported success without ever moving the arm.
  static void expectArmReached(const vector6d_t& target)
  {
    const double k_position_tolerance = 0.01;
    const vector6d_t reached = waitForJointsNear(target, k_position_tolerance, std::chrono::seconds(2));
    for (size_t i = 0; i < target.size(); ++i)
    {
      EXPECT_NEAR(target[i], reached[i], k_position_tolerance) << "joint " << i << " did not reach its target";
    }
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
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP);
    }
    return control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;
  }

  // Variant that fires a single user-supplied action exactly once after
  // `inject_at` has elapsed since the call started. The action takes the
  // place of one NOOP-pump iteration. Used to inject a stray trajectory
  // control message mid-flight without spinning up a second thread.
  control::TrajectoryResult waitForTrajectoryResultPumpingNoopsWithInjection(std::chrono::milliseconds total_timeout,
                                                                             std::chrono::milliseconds inject_at,
                                                                             std::function<void()> inject_action)
  {
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + total_timeout;
    const auto inject_deadline = start + inject_at;
    bool injected = false;
    while (std::chrono::steady_clock::now() < deadline)
    {
      std::unique_lock<std::mutex> lk(g_trajectory_result_mutex);
      if (g_trajectory_result_cv.wait_for(lk, std::chrono::milliseconds(50),
                                          [] { return g_trajectory_result_received; }))
      {
        return g_trajectory_result;
      }
      lk.unlock();
      if (!injected && std::chrono::steady_clock::now() >= inject_deadline)
      {
        inject_action();
        injected = true;
      }
      else
      {
        g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP);
      }
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
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
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
      control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 100));

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// Regression test for PR #528 cursor-bot review comment 1a.
// A stray TRAJECTORY_STREAM_END dispatched during a legacy finite
// TRAJECTORY_START trajectory must not corrupt trajectory_points_left.
// Without the streaming-guard fix, the dispatcher's STREAM_END math
// (which assumes trajectory_points_left is a sentinel-based credit
// budget) computes a huge negative value, the trajectoryThread's
// "trajectory_points_left > 0" predicate fails on the next iteration,
// the loop exits while trajectory_result is still SUCCESS, and the
// callback fires SUCCESS even though motion was truncated.
TEST_F(TrajectoryStreamingTest, stray_stream_end_during_finite_trajectory_does_not_truncate)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.40, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_c = { 0.0, -1.20, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Pre-position
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Send a 3-point finite trajectory, each segment ~1 second.
  const float k_segment_time = 1.0f;
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 3));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_b, zero, zero, k_segment_time));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_c, zero, zero, k_segment_time));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, k_segment_time));

  const auto motion_start = std::chrono::steady_clock::now();
  const auto result =
      waitForTrajectoryResultPumpingNoopsWithInjection(std::chrono::seconds(6), std::chrono::milliseconds(500), [] {
        g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
            control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, 0);
      });
  const auto elapsed = std::chrono::steady_clock::now() - motion_start;

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, result);
  // Three 1-second segments expect ~3 s of motion. Without the fix, the
  // stray STREAM_END truncates the loop after segment 1, callback fires
  // in ~1 s.
  EXPECT_GE(elapsed, std::chrono::milliseconds(2500)) << "Expected ~3 s of motion; stray STREAM_END appears to have "
                                                         "truncated the trajectory.";
}

// Regression test for PR #528 cursor-bot review comment 1c.
// A second TRAJECTORY_STREAM_END dispatched during the drain phase
// re-applies the sentinel-based math to a non-sentinel
// trajectory_points_left value, producing a huge negative value and
// truncating the drain while reporting SUCCESS. Without the streaming-
// guard fix the second STREAM_END is destructive; with the fix it is
// a no-op because trajectory_streaming was cleared by the first.
TEST_F(TrajectoryStreamingTest, double_stream_end_does_not_truncate)
{
  const vector6d_t held_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Pre-position
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Stream 20 points with tmptime=0.1 s each. Nominal consumer-side
  // execution is ~2 s.
  const int k_num_points = 20;
  const float k_step_time = 0.1f;

  const auto motion_start = std::chrono::steady_clock::now();

  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, k_step_time));
  }
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));
  // Second STREAM_END immediately after the first. Without the guard fix
  // this re-applies the sentinel-based math to the already-recomputed
  // counter and truncates the drain.
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));

  const auto result = waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5));
  const auto elapsed = std::chrono::steady_clock::now() - motion_start;

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS, result);
  EXPECT_GE(elapsed, std::chrono::milliseconds(1500)) << "Expected ~2 s of consumer-side execution; double STREAM_END "
                                                         "appears to have truncated the drain.";
}

// Regression test for PR #528 cursor-bot review comment 2.
// A post-STREAM_END timeout-on-read with trajectory_points_left still
// positive must be reported as TRAJECTORY_RESULT_FAILURE, not silently
// swallowed as SUCCESS. The current clean-end shortcut fires whenever
// trajectory_streaming was just cleared, regardless of whether the
// consumer still expects more points. The fix narrows the shortcut to
// the only legitimate case (count_after_decrement < 0), which can only
// arise via a race where STREAM_END landed during the read block after
// the consumer had already drained the buffer.
TEST_F(TrajectoryStreamingTest, stream_end_with_overcount_yields_failure)
{
  const vector6d_t held_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Pre-position
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Send 50 real points but tell STREAM_END the producer wrote 100.
  const int k_actually_sent = 50;
  const int k_announced = 100;
  const float k_step_time = 0.01f;

  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_actually_sent; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(held_pose, zero, zero, k_step_time));
  }
  // The STREAM_END must arrive after the trajectoryThread has executed
  // its `was_streaming_at_start = trajectory_streaming` assignment with
  // the streaming flag still True. Otherwise a race - URScript
  // scheduling the dispatcher's next iteration before the new thread's
  // first statement runs - causes was_streaming_at_start to be captured
  // as False, which sends any underrun straight down the legacy FAILURE
  // arm regardless of the bug under test. Pump NOOPs for ~250 ms to
  // give the thread plenty of time to start and consume some points
  // before STREAM_END is dispatched.
  for (int i = 0; i < 5; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP));
  }
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_announced));

  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// A streaming producer is obliged to keep the trajectory socket fed, but that
// obligation is about trajectory time rather than about how often it writes.
// Here the producer writes twenty points of 0.1s each and then stops writing for
// 400ms, while continuing to answer the reverse interface with NOOPs. The gap is
// much longer than the per-point socket read timeout, which is `get_steptime`
// (2-8ms) once the robot is moving, but much shorter than the two seconds of
// trajectory time already in the socket. The `trajectoryThread` should read
// straight across the gap and the stream should end in SUCCESS. A FAILURE would
// mean the read timeout starves the thread while unread points remain.
TEST_F(TrajectoryStreamingTest, stream_survives_send_gap_while_unread_points_remain)
{
  // A small wrist-3 twist the arm can actually execute, with zero velocity and
  // acceleration at each endpoint. Because the arm moves, `is_robot_moving`
  // stays true and the per-point read uses the `get_steptime` timeout rather
  // than the 0.5s timeout that applies before motion starts.
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Position the robot at `pose_a` with a one-shot finite trajectory.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Write two seconds of trajectory time to the socket up front, alternating
  // between `pose_b` and `pose_a`.
  const int k_num_points = 20;
  const float k_step_time = 0.1f;
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, k_step_time));
  }

  // Stop writing to the trajectory socket for 400ms while the thread consumes
  // what we already sent, still answering the reverse interface with NOOPs so
  // that only the trajectory socket goes quiet. The gap stays under the script's
  // 0.5s constant, which sleep_for could otherwise overshoot.
  for (int i = 0; i < 8; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP));
  }

  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// The companion to `stream_survives_send_gap_while_unread_points_remain`. Rather
// than writing the whole stream up front, the producer writes one point per
// segment duration, so nothing is ever in the socket ahead of what the robot is
// executing. The producer keeps pace on average, one 0.1s point every 100ms, so
// trajectory time is never actually missing. The question is whether the
// per-point read timeout, `get_steptime` (2-8ms) once the robot is moving, is
// wide enough to absorb the jitter between our write and the robot's read.
TEST_F(TrajectoryStreamingTest, stream_survives_producer_with_no_lookahead)
{
  // The same poses as `stream_survives_send_gap_while_unread_points_remain`.
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Position the robot at pose_a with a one-shot finite trajectory.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  const int k_num_points = 20;
  const float k_step_time = 0.1f;
  const auto k_send_period = std::chrono::milliseconds(100);
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  // Write one point, then sleep a full segment before writing the next, so no
  // point arrives before the robot needs it.
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, k_step_time));
    // Answer the reverse interface without writing anything further to the
    // trajectory socket.
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_NOOP));
    std::this_thread::sleep_for(k_send_period);
  }
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// A control test, and the one case in this group that passes. When a finite
// trajectory underruns, `trajectory_points_left` is left at the declared count
// minus the points actually read and `trajectory_streaming` is false, so the
// cleanup that runs before the next trajectory has a real bound and stops once
// it has drained that many points. The next trajectory then runs normally. The
// streaming cases below differ only in that no count is ever declared, so we pin
// the finite behavior here to keep it from regressing.
TEST_F(TrajectoryStreamingTest, finite_underrun_then_finite_move_recovers)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Pre-position.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Starve a finite trajectory: declare two points, send only one. The thread
  // consumes the first, starts moving, then underruns on the missing second.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 2));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_b, zero, zero, 0.1f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Recovery: a plain finite move must succeed.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// Recovery from a streaming underrun by way of a finite move, taking the
// `TRAJECTORY_MODE_RECEIVE` branch of the dispatcher.
//
// A stream which aborted used to leave `trajectory_points_left` holding
// `STREAMING_SENTINEL` and `trajectory_streaming` still true, which gave
// `clearTrajectoryPointsThread` no real bound to drain against, so it went on
// reading until the socket fell quiet. Those reads consumed the point belonging
// to the recovery move on the next trajectory command, and so the recovery move
// starved in its turn. The underrun now resets both of those globals before it
// reports failure, which is what this test holds in place.
//
// See https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/550.
TEST_F(TrajectoryStreamingTest, stream_underrun_then_finite_move_recovers)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Pre-position.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Starve the stream while the arm is moving: open it, write enough points to
  // get the arm going, then stop writing. The thread consumes what we sent and
  // then underruns. We never send a STREAM_END, so `trajectory_points_left` is
  // still `STREAMING_SENTINEL` when the thread reports FAILURE.
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, 0.1f));
  }
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Recovery: a plain finite move must succeed.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// The same underrun as `stream_underrun_then_finite_move_recovers`, except that
// the recovery is itself a stream, so the dispatcher takes the
// `TRAJECTORY_MODE_STREAM_START` branch rather than `TRAJECTORY_MODE_RECEIVE`.
// The two branches used to fail in the same way, and a change which repaired
// only one of them would still be caught here.
//
// See https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/550.
TEST_F(TrajectoryStreamingTest, stream_underrun_then_stream_recovers)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Pre-position.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Starve the stream while the arm is moving, as in
  // `stream_underrun_then_finite_move_recovers`.
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, 0.1f));
  }
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Recovery: a new stream that we keep fed must complete with SUCCESS.
  const int k_num_points = 20;
  const float k_step_time = 0.1f;
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(
        g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, k_step_time));
  }
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
}

// After a stream underruns, the producer does not learn of the failure
// immediately and carries on writing. Those leftover points stay in the
// trajectory socket, and the next trajectory must not execute them. This is the
// complaint that the issue below was filed about.
//
// We can observe it without reading any positions, by making the leftover point
// one that the robot will refuse. A wrist move of about one radian demanded in
// 1ms is roughly 1000 rad/s, far above `JOINT_IGNORE_SPEED`, so
// `targetWithinLimits` rejects it and whichever trajectory reads it ends in
// CANCELED. A result of SUCCESS therefore tells us the recovery ran its own
// point, and CANCELED tells us it ran the leftover one.
//
// That distinction is not sufficient on its own, because it cannot tell either
// of those outcomes apart from a recovery which ran nothing at all. The loop in
// the trajectory thread runs for as long as the result is SUCCESS and points
// remain, and the result begins as SUCCESS, so a recovery which consumed the
// leftover record, reached zero and exited without moving would report SUCCESS
// as well. We therefore have the recovery target a pose which neither the stream
// nor the leftover point ever asked for, and assert that the arm arrived at it.
//
// See https://github.com/UniversalRobots/Universal_Robots_Client_Library/issues/550.
TEST_F(TrajectoryStreamingTest, stream_underrun_stale_points_not_executed_by_recovery)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t pose_b = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.15 };
  const vector6d_t recovery_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, -0.30 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // Pre-position.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Starve the stream while the arm is moving. The thread consumes what we sent,
  // underruns, and exits, leaving the trajectory socket empty and unattended.
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < 3; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(i % 2 == 0 ? pose_b : pose_a, zero, zero, 0.1f));
  }
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Stand in for a producer that has not yet learned of the failure by writing
  // one more point, the invalid one described above.
  const vector6d_t stale_fast_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 1.0 };
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(stale_fast_pose, zero, zero, 0.001f));

  // Recovery: a finite move with one valid point. It must run its own point, end
  // in SUCCESS, and actually move the arm to the pose that point asked for.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(recovery_pose, zero, zero, 2.0f));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  expectArmReached(recovery_pose);
}

// A client which cancels a stream with a count smaller than the number of points
// it actually wrote leaves those records queued on the trajectory socket. The
// dispatcher works the bound for the cleanup out as the count it was given less
// the number already consumed, so a count of zero produces a negative bound, and
// `clearTrajectoryPointsThread`, whose loop runs only while
// `trajectory_points_left` is above zero, drains nothing at all. This is the bug
// that the Viam module shipped, which sent a count of zero. It degrades rather
// than breaks, because the eager drain misses those records and the move which
// follows disposes of them instead, by recognizing that they name an earlier
// move.
//
// Were the identifiers not there, the recovery would read the first of the
// leftover records as though it were its own point, execute it, and report
// SUCCESS from the wrong pose. Only the assertion on motion catches that, which
// is why the recovery targets a pose the cancelled stream never asked for.
//
// The cancel leaves a run of records queued rather than only one, so a skip
// which worked for the first record alone would not be enough to pass here.
TEST_F(TrajectoryStreamingTest, cancel_with_short_count_then_finite_move_recovers)
{
  const vector6d_t pose_a = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const vector6d_t stale_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, 0.30 };
  const vector6d_t recovery_pose = { 0.0, -1.57, 0.0, -1.57, 0.0, -0.30 };
  const vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Pre-position.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 2.0f));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Pile points into the socket faster than the arm can run them, so that a
  // cancel leaves most of them unread.
  const int k_num_points = 20;
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_STREAM_START));
  for (int i = 0; i < k_num_points; ++i)
  {
    ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(stale_pose, zero, zero, 0.1f));
  }

  // Let the thread consume a point or two so that the cancel arrives in the
  // middle of the stream rather than racing the spawn of the thread, and then
  // cancel with a count which understates what we wrote.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      control::TrajectoryControlMessage::TRAJECTORY_CANCEL, 0));
  ASSERT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  resetTrajectoryResultState();

  // Recovery: one valid point, to a pose the cancelled stream never asked for.
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1));
  ASSERT_TRUE(g_my_robot->getUrDriver()->writeTrajectorySplinePoint(recovery_pose, zero, zero, 2.0f));
  EXPECT_EQ(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS,
            waitForTrajectoryResultPumpingNoops(std::chrono::seconds(5)));
  expectArmReached(recovery_pose);
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
