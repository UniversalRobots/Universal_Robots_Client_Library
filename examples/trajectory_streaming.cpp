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

// ----------------------------------------------------------------------------
// Trajectory streaming example
//
// Demonstrates the open-ended (streaming) trajectory feature: a producer
// streams points to the controller without declaring a total point count up
// front, then signals end-of-stream when finished. Contrast with
// examples/trajectory_point_interface.cpp, which uses the finite-trajectory
// path where the total point count is fixed in the initial TRAJECTORY_START
// message.
//
// The streaming communication contract -- the STREAM_START / STREAM_END
// handshake, the end-of-stream point count, the producer's obligation not to
// starve the controller, the controlled-stop terminal point, and the
// trajectory result codes -- is documented in
// doc/architecture/reverse_interface.rst. A narrative walkthrough of this
// example lives in doc/examples/trajectory_streaming.rst.
// ----------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ur_client_library/example_robot_wrapper.h>
#include "ur_client_library/control/trajectory_point_interface.h"
#include "ur_client_library/log.h"
#include "ur_client_library/types.h"

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";

std::unique_ptr<urcl::ExampleRobotWrapper> g_my_robot;
std::atomic<bool> g_trajectory_done{ false };
std::atomic<urcl::control::TrajectoryResult> g_trajectory_result{
  urcl::control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN
};

void trajDoneCallback(const urcl::control::TrajectoryResult& result)
{
  g_trajectory_result = result;
  g_trajectory_done = true;
  URCL_LOG_INFO("Trajectory done with result %s", urcl::control::trajectoryResultToString(result).c_str());
}

// Pump TRAJECTORY_NOOP on the reverse socket while waiting for the
// trajectory-done callback. Without this the URScript dispatcher's
// reverse_socket read times out, which terminates the external_control
// program entirely.
void waitForTrajectoryDoneWithKeepalives()
{
  while (!g_trajectory_done)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    g_my_robot->getUrDriver()->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_NOOP);
  }
}

// Sample a quintic-Hermite interpolation between q_start and q_end with zero
// boundary velocity and acceleration, evaluated at fractional time s in [0, 1]
// for a segment of total duration t_total:
//
//   q(s)   = q_start + (q_end - q_start) * (10s^3 - 15s^4 + 6s^5)
//   qd(t)  = (q_end - q_start) * (30s^2 - 60s^3 + 30s^4) / t_total
//   qdd(t) = (q_end - q_start) * (60s  - 180s^2 + 120s^3) / t_total^2
struct SampledPoint
{
  urcl::vector6d_t q;
  urcl::vector6d_t qd;
  urcl::vector6d_t qdd;
};

SampledPoint sampleQuinticHermite(const urcl::vector6d_t& q_start, const urcl::vector6d_t& q_end, const double s,
                                  const double t_total)
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;
  const double pos_blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  const double vel_blend = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / t_total;
  const double acc_blend = (60.0 * s - 180.0 * s2 + 120.0 * s3) / (t_total * t_total);

  SampledPoint out{};
  for (size_t i = 0; i < q_start.size(); ++i)
  {
    const double delta = q_end[i] - q_start[i];
    out.q[i] = q_start[i] + delta * pos_blend;
    out.qd[i] = delta * vel_blend;
    out.qdd[i] = delta * acc_blend;
  }
  return out;
}

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = argv[1];
  }

  const bool headless_mode = true;
  g_my_robot = std::make_unique<urcl::ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                           "external_control.urp");
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Robot initialization failed. See preceding output for details.");
    return 1;
  }

  g_my_robot->getUrDriver()->registerTrajectoryDoneCallback(&trajDoneCallback);

  // --------------- PRE-POSITION VIA FINITE TRAJECTORY ----------------
  // Park the robot at a known starting pose before the streaming demo. This
  // also exercises the finite TRAJECTORY_START path for comparison.

  const urcl::vector6d_t pose_a = { -1.57, -1.57, 0.0, -1.57, 0.0, 0.0 };
  const urcl::vector6d_t pose_b = { -1.57, -1.20, 0.5, -1.57, 0.0, 0.0 };
  const urcl::vector6d_t zero = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  URCL_LOG_INFO("Pre-positioning to pose A via finite trajectory");
  g_trajectory_done = false;
  g_my_robot->getUrDriver()->writeTrajectoryControlMessage(urcl::control::TrajectoryControlMessage::TRAJECTORY_START,
                                                           1);
  g_my_robot->getUrDriver()->writeTrajectorySplinePoint(pose_a, zero, zero, 3.0f);
  waitForTrajectoryDoneWithKeepalives();
  if (g_trajectory_result != urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS)
  {
    URCL_LOG_ERROR("Pre-positioning failed");
    return 1;
  }

  // ----------------- STREAMING TRAJECTORY DEMO -----------------------------
  // Stream a quintic-Hermite motion from pose A to pose B. All points are
  // precomputed and written back-to-back; the robot works through them at its
  // own pace. See doc/examples/trajectory_streaming.rst for details.

  const int k_num_points = 200;
  const double k_motion_duration_s = 2.0;
  const double dt = k_motion_duration_s / k_num_points;

  std::vector<SampledPoint> points;
  points.reserve(k_num_points);
  for (int i = 1; i <= k_num_points; ++i)
  {
    const double s = static_cast<double>(i) / k_num_points;
    points.push_back(sampleQuinticHermite(pose_a, pose_b, s, k_motion_duration_s));
  }
  // The final quintic-Hermite point has qd=0, qdd=0: the controlled-stop
  // terminal the streaming contract requires.

  URCL_LOG_INFO("Streaming %d points over %.2f s nominal motion duration", k_num_points, k_motion_duration_s);
  g_trajectory_done = false;
  g_trajectory_result = urcl::control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;

  const auto stream_start_wall = std::chrono::steady_clock::now();
  g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      urcl::control::TrajectoryControlMessage::TRAJECTORY_STREAM_START);
  for (const auto& p : points)
  {
    if (!g_my_robot->getUrDriver()->writeTrajectorySplinePoint(p.q, p.qd, p.qdd, static_cast<float>(dt)))
    {
      URCL_LOG_ERROR("writeTrajectorySplinePoint failed mid-stream");
      return 1;
    }
  }
  const auto stream_end_send_wall = std::chrono::steady_clock::now();
  g_my_robot->getUrDriver()->writeTrajectoryControlMessage(
      urcl::control::TrajectoryControlMessage::TRAJECTORY_STREAM_END, k_num_points);

  waitForTrajectoryDoneWithKeepalives();
  const auto callback_wall = std::chrono::steady_clock::now();

  const auto send_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stream_end_send_wall - stream_start_wall);
  const auto end_to_callback =
      std::chrono::duration_cast<std::chrono::milliseconds>(callback_wall - stream_end_send_wall);
  URCL_LOG_INFO("All %d points written in %lld ms wall-clock", k_num_points,
                static_cast<long long>(send_duration.count()));
  // Time from STREAM_END to the done callback. Here it is dominated by the
  // consumer draining the batched backlog, not by STREAM_END processing; a
  // steady-state producer would see roughly one controller step plus the
  // stopj at trajectory thread exit. See doc/examples/trajectory_streaming.rst.
  URCL_LOG_INFO("Time from sending STREAM_END to trajectory-done callback: %lld ms",
                static_cast<long long>(end_to_callback.count()));
  URCL_LOG_INFO("Final result: %s", urcl::control::trajectoryResultToString(g_trajectory_result).c_str());

  if (g_trajectory_result != urcl::control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS)
  {
    URCL_LOG_ERROR("Streaming trajectory did not complete successfully");
    return 1;
  }

  g_my_robot->getUrDriver()->stopControl();
  return 0;
}
