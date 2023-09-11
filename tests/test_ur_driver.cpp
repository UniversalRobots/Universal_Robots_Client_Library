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

#define private public
#include <ur_client_library/ur/ur_driver.h>

using namespace urcl;

TEST(ur_driver, read_non_existing_script_file)
{
  const std::string non_existing_script_file = "";
  EXPECT_THROW(UrDriver::readScriptFile(non_existing_script_file), UrException);
}

TEST(ur_driver, read_existing_script_file)
{
  char existing_script_file[] = "urscript.XXXXXX";
  int fd = mkstemp(existing_script_file);
  if (fd == -1)
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  EXPECT_NO_THROW(UrDriver::readScriptFile(existing_script_file));

  // clean up
  close(fd);
  unlink(existing_script_file);
}

TEST(ur_driver, watchdog_off_realtime_control_modes)
{
  // It shouldn't be possible to set watchdog::off, when the commands are realtime commands, instead it should be set to
  // step time
  Watchdog watchdog = Watchdog::off();
  float step_time = 0.002;
  unsigned int length =
      sizeof(comm::ControlModeTypes::REALTIME_CONTROL_MODES) / sizeof(*comm::ControlModeTypes::REALTIME_CONTROL_MODES);

  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, step_time);
  }
}

TEST(ur_driver, watchdog_off_non_realtime_control_modes)
{
  // It should be possible to set watchdog::off, when the commands aren't realtime commands
  Watchdog watchdog = Watchdog::off();
  float step_time = 0.002;
  unsigned int length = sizeof(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES) /
                        sizeof(*comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES);

  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, 0.0);
  }
}

TEST(ur_driver, watchdog_timeout_below_step_time_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  Watchdog watchdog = Watchdog::sec(0.001);
  float step_time = 0.002;

  unsigned int length =
      sizeof(comm::ControlModeTypes::REALTIME_CONTROL_MODES) / sizeof(*comm::ControlModeTypes::REALTIME_CONTROL_MODES);
  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, step_time);
  }
}

TEST(ur_driver, watchdog_timeout_below_step_time_non_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  Watchdog watchdog = Watchdog::sec(0.001);
  float step_time = 0.002;

  unsigned int length = sizeof(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES) /
                        sizeof(*comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES);
  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, step_time);
  }
}

TEST(ur_driver, watchdog_timeout_above_max_allowed_timeout_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  float max_allowed_timeout = 1.0;
  Watchdog watchdog = Watchdog::sec(1.2);
  float step_time = 0.002;

  unsigned int length =
      sizeof(comm::ControlModeTypes::REALTIME_CONTROL_MODES) / sizeof(*comm::ControlModeTypes::REALTIME_CONTROL_MODES);
  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, max_allowed_timeout);
  }
}

TEST(ur_driver, watchdog_timeout_within_limit_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  float expected_timeout = 0.1;
  Watchdog watchdog = Watchdog::sec(expected_timeout);
  float step_time = 0.002;

  unsigned int length =
      sizeof(comm::ControlModeTypes::REALTIME_CONTROL_MODES) / sizeof(*comm::ControlModeTypes::REALTIME_CONTROL_MODES);
  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, expected_timeout);
  }
}

TEST(ur_driver, watchdog_timeout_within_limit_non_realtime_control_modes)
{
  // When watchdog timeout is set below step_time, the timeout should be set to step time
  float expected_timeout = 0.5;
  Watchdog watchdog = Watchdog::sec(expected_timeout);
  float step_time = 0.002;

  unsigned int length =
      sizeof(comm::ControlModeTypes::REALTIME_CONTROL_MODES) / sizeof(*comm::ControlModeTypes::REALTIME_CONTROL_MODES);
  for (unsigned int i = 0; i < length; ++i)
  {
    float actual_timeout =
        UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlModeTypes::REALTIME_CONTROL_MODES[i], step_time);
    EXPECT_EQ(actual_timeout, expected_timeout);
  }
}

TEST(ur_driver, watchdog_timeout_unknown_control_mode)
{
  // If the control mode is neither realtime or non realtime, the function should throw an exception
  Watchdog watchdog = Watchdog::sec(0.2);
  EXPECT_THROW(UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlMode::MODE_STOPPED, 0.002), UrException);
  EXPECT_THROW(UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlMode::MODE_UNINITIALIZED, 0.002), UrException);
  EXPECT_THROW(UrDriver::verifyWatchdogTimeout(watchdog, comm::ControlMode::MODE_TOOL_IN_CONTACT, 0.002), UrException);
}

// TODO we should add more tests for the UrDriver class.

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
