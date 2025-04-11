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

#include "ur_client_library/ur/robot_receive_timeout.h"
#include "ur_client_library/exceptions.h"

using namespace urcl;

const std::chrono::milliseconds G_STEP_TIME(2);

TEST(RobotReceiveTimeout, milliseconds_configuration)
{
  const int expected_timeout = 100;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(expected_timeout);
  EXPECT_EQ(robot_receive_timeout.getAsMilliseconds().count(), expected_timeout);
}

TEST(RobotReceiveTimeout, empty_milliseconds_configuration)
{
  const int expected_timeout = 20;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec();
  EXPECT_EQ(robot_receive_timeout.getAsMilliseconds().count(), expected_timeout);
}

TEST(RobotReceiveTimeout, seconds_configuration)
{
  float timeout = 0.1;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::sec(timeout);
  const int expected_timeout = timeout * 1000;  // Convert to milliseconds
  EXPECT_EQ(robot_receive_timeout.getAsMilliseconds().count(), expected_timeout);
}

TEST(RobotReceiveTimeout, empty_seconds_configuration)
{
  const int expected_timeout = 20;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::sec();
  EXPECT_EQ(robot_receive_timeout.getAsMilliseconds().count(), expected_timeout);
}

TEST(RobotReceiveTimeout, off_configuration)
{
  const int expected_timeout = 0;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::off();
  EXPECT_EQ(robot_receive_timeout.getAsMilliseconds().count(), expected_timeout);
}

TEST(RobotReceiveTimeout, off_realtime_control_modes)
{
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::off();

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout =
        robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, G_STEP_TIME.count());
  }
}

TEST(RobotReceiveTimeout, off_non_realtime_control_modes)
{
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::off();

  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(
        comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    const int expected_timeout = 0;
    EXPECT_EQ(actual_timeout, expected_timeout);
  }
}

TEST(RobotReceiveTimeout, timeout_below_step_time_realtime_control_modes)
{
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(1);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout =
        robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, G_STEP_TIME.count());
  }
}

TEST(RobotReceiveTimeout, timeout_below_step_time_non_realtime_control_modes)
{
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(1);

  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(
        comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, G_STEP_TIME.count());
  }
}

TEST(RobotReceiveTimeout, timeout_above_max_allowed_timeout_realtime_control_modes)
{
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(400);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout =
        robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, RobotReceiveTimeout::MAX_RT_RECEIVE_TIMEOUT_MS.count());
  }
}

TEST(RobotReceiveTimeout, timeout_within_limit_realtime_control_modes)
{
  const int expected_timeout = 100;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(expected_timeout);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout =
        robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, expected_timeout);
  }
}

TEST(RobotReceiveTimeout, timeout_within_limit_non_realtime_control_modes)
{
  const int expected_timeout = 500;
  RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(expected_timeout);

  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    const int actual_timeout = robot_receive_timeout.verifyRobotReceiveTimeout(
        comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], G_STEP_TIME);
    EXPECT_EQ(actual_timeout, expected_timeout);
  }
}

TEST(RobotReceiveTimeout, unknown_control_mode)
{
  // If the control mode is neither realtime or non realtime, the function should throw an exception
  for (unsigned int i = 0; i < comm::ControlModeTypes::STATIONARY_CONTROL_MODES.size(); ++i)
  {
    RobotReceiveTimeout robot_receive_timeout = RobotReceiveTimeout::millisec(200);
    EXPECT_THROW(robot_receive_timeout.verifyRobotReceiveTimeout(comm::ControlModeTypes::STATIONARY_CONTROL_MODES[i],
                                                                 G_STEP_TIME),
                 UrException);
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
