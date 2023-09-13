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

#include "ur_client_library/ur/watchdog.h"
#include "ur_client_library/exceptions.h"

using namespace urcl;

TEST(watchdog, watchdog_initialization)
{
  Watchdog watchdog;
  const std::chrono::milliseconds expected_timeout(20);
  EXPECT_EQ(watchdog.timeout_.count(), expected_timeout.count());
}

TEST(watchdog, watchdog_seconds_configuration)
{
  const std::chrono::milliseconds expected_timeout(100);
  Watchdog watchdog = Watchdog::millisec(expected_timeout);
  EXPECT_EQ(watchdog.timeout_.count(), expected_timeout.count());
}

TEST(watchdog, watchdog_empty_seconds_configuration)
{
  const std::chrono::milliseconds expected_timeout(20);
  Watchdog watchdog = Watchdog::millisec();
  EXPECT_EQ(watchdog.timeout_.count(), expected_timeout.count());
}

TEST(watchdog, watchdog_off_configuration)
{
  const std::chrono::milliseconds expected_timeout(0);
  Watchdog watchdog = Watchdog::off();
  EXPECT_EQ(watchdog.timeout_.count(), expected_timeout.count());
}

TEST(watchdog, watchdog_off_realtime_control_modes)
{
  Watchdog watchdog = Watchdog::off();
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), step_time.count());
  }
}

TEST(watchdog, watchdog_off_non_realtime_control_modes)
{
  Watchdog watchdog = Watchdog::off();
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], step_time);
    const std::chrono::milliseconds expected_timeout(0);
    EXPECT_EQ(actual_timeout.count(), expected_timeout.count());
  }
}

TEST(watchdog, watchdog_timeout_below_step_time_realtime_control_modes)
{
  Watchdog watchdog = Watchdog::millisec(std::chrono::milliseconds(1));
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), step_time.count());
  }
}

TEST(watchdog, watchdog_timeout_below_step_time_non_realtime_control_modes)
{
  Watchdog watchdog = Watchdog::millisec(std::chrono::milliseconds(1));
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), step_time.count());
  }
}

TEST(watchdog, watchdog_timeout_above_max_allowed_timeout_realtime_control_modes)
{
  std::chrono::milliseconds max_allowed_timeout(1000);
  Watchdog watchdog = Watchdog::millisec(std::chrono::milliseconds(1200));
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), max_allowed_timeout.count());
  }
}

TEST(watchdog, watchdog_timeout_within_limit_realtime_control_modes)
{
  const std::chrono::milliseconds expected_timeout(100);
  Watchdog watchdog = Watchdog::millisec(expected_timeout);
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), expected_timeout.count());
  }
}

TEST(watchdog, watchdog_timeout_within_limit_non_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  const std::chrono::milliseconds expected_timeout(500);
  Watchdog watchdog = Watchdog::millisec(expected_timeout);
  const std::chrono::milliseconds step_time(2);

  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    const std::chrono::milliseconds actual_timeout =
        watchdog.verifyWatchdogTimeout(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout.count(), expected_timeout.count());
  }
}

TEST(watchdog, watchdog_timeout_unknown_control_mode)
{
  // If the control mode is neither realtime or non realtime, the function should throw an exception
  Watchdog watchdog = Watchdog::millisec(std::chrono::milliseconds(200));
  const std::chrono::milliseconds step_time(2);
  EXPECT_THROW(watchdog.verifyWatchdogTimeout(comm::ControlMode::MODE_STOPPED, step_time), UrException);
  EXPECT_THROW(watchdog.verifyWatchdogTimeout(comm::ControlMode::MODE_UNINITIALIZED, step_time), UrException);
  EXPECT_THROW(watchdog.verifyWatchdogTimeout(comm::ControlMode::MODE_TOOL_IN_CONTACT, step_time), UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
