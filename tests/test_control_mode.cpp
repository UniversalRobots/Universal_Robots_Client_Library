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

#include <ur_client_library/comm/control_mode.h>
#include <ur_client_library/types.h>

using namespace urcl;

TEST(control_mode, is_control_mode_realtime_with_realtime_control_modes)
{
  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    EXPECT_TRUE(comm::ControlModeTypes::isControlModeRealtime(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i]));
  }
}

TEST(control_mode, is_control_mode_realtime_with_non_realtime_control_modes)
{
  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    EXPECT_FALSE(comm::ControlModeTypes::isControlModeRealtime(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i]));
  }

  for (unsigned int i = 0; i < comm::ControlModeTypes::STATIONARY_CONTROL_MODES.size(); ++i)
  {
    EXPECT_FALSE(comm::ControlModeTypes::isControlModeRealtime(comm::ControlModeTypes::STATIONARY_CONTROL_MODES[i]));
  }
}

TEST(control_mode, is_control_mode_non_realtime_with_non_realtime_control_modes)
{
  for (unsigned int i = 0; i < comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size(); ++i)
  {
    EXPECT_TRUE(
        comm::ControlModeTypes::isControlModeNonRealtime(comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES[i]));
  }
}

TEST(control_mode, is_control_mode_non_realtime_with_realtime_control_modes)
{
  for (unsigned int i = 0; i < comm::ControlModeTypes::REALTIME_CONTROL_MODES.size(); ++i)
  {
    EXPECT_FALSE(comm::ControlModeTypes::isControlModeNonRealtime(comm::ControlModeTypes::REALTIME_CONTROL_MODES[i]));
  }
  for (unsigned int i = 0; i < comm::ControlModeTypes::STATIONARY_CONTROL_MODES.size(); ++i)
  {
    EXPECT_FALSE(comm::ControlModeTypes::isControlModeNonRealtime(comm::ControlModeTypes::STATIONARY_CONTROL_MODES[i]));
  }
}

TEST(control_mode, control_mode_types_size_equals_number_off_control_modes)
{
  int number_of_control_modes = 0;
  for (int index = toUnderlying(comm::ControlMode::MODE_STOPPED); index < toUnderlying(comm::ControlMode::END); ++index)
  {
    number_of_control_modes++;
  }
  int number_of_control_mode_types = comm::ControlModeTypes::REALTIME_CONTROL_MODES.size() +
                                     comm::ControlModeTypes::NON_REALTIME_CONTROL_MODES.size() +
                                     comm::ControlModeTypes::STATIONARY_CONTROL_MODES.size();
  EXPECT_EQ(number_of_control_mode_types, number_of_control_modes);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
