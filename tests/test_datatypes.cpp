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

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

#include <ur_client_library/ur/datatypes.h>

using namespace urcl;

TEST(TestDatatypes, RobotModeString_all_values)
{
  EXPECT_EQ(robotModeString(RobotMode::UNKNOWN), "UNKNOWN");
  EXPECT_EQ(robotModeString(RobotMode::NO_CONTROLLER), "NO_CONTROLLER");
  EXPECT_EQ(robotModeString(RobotMode::DISCONNECTED), "DISCONNECTED");
  EXPECT_EQ(robotModeString(RobotMode::CONFIRM_SAFETY), "CONFIRM_SAFETY");
  EXPECT_EQ(robotModeString(RobotMode::BOOTING), "BOOTING");
  EXPECT_EQ(robotModeString(RobotMode::POWER_OFF), "POWER_OFF");
  EXPECT_EQ(robotModeString(RobotMode::POWER_ON), "POWER_ON");
  EXPECT_EQ(robotModeString(RobotMode::IDLE), "IDLE");
  EXPECT_EQ(robotModeString(RobotMode::BACKDRIVE), "BACKDRIVE");
  EXPECT_EQ(robotModeString(RobotMode::RUNNING), "RUNNING");
  EXPECT_EQ(robotModeString(RobotMode::UPDATING_FIRMWARE), "UPDATING_FIRMWARE");
}

TEST(TestDatatypes, RobotModeString_invalid_throws)
{
  const RobotMode invalid = static_cast<RobotMode>(99);
  EXPECT_THROW(robotModeString(invalid), std::invalid_argument);
}

TEST(TestDatatypes, SafetyModeString_all_values)
{
  EXPECT_EQ(safetyModeString(SafetyMode::NORMAL), "NORMAL");
  EXPECT_EQ(safetyModeString(SafetyMode::REDUCED), "REDUCED");
  EXPECT_EQ(safetyModeString(SafetyMode::PROTECTIVE_STOP), "PROTECTIVE_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::RECOVERY), "RECOVERY");
  EXPECT_EQ(safetyModeString(SafetyMode::SAFEGUARD_STOP), "SAFEGUARD_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::SYSTEM_EMERGENCY_STOP), "SYSTEM_EMERGENCY_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::ROBOT_EMERGENCY_STOP), "ROBOT_EMERGENCY_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::VIOLATION), "VIOLATION");
  EXPECT_EQ(safetyModeString(SafetyMode::FAULT), "FAULT");
  EXPECT_EQ(safetyModeString(SafetyMode::VALIDATE_JOINT_ID), "VALIDATE_JOINT_ID");
  EXPECT_EQ(safetyModeString(SafetyMode::UNDEFINED_SAFETY_MODE), "UNDEFINED_SAFETY_MODE");
  EXPECT_EQ(safetyModeString(SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP), "AUTOMATIC_MODE_SAFEGUARD_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP), "SYSTEM_THREE_POSITION_ENABLING_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::TP_THREE_POSITION_ENABLING_STOP), "TP_THREE_POSITION_ENABLING_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::IMMI_EMERGENCY_STOP), "IMMI_EMERGENCY_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::IMMI_SAFEGUARD_STOP), "IMMI_SAFEGUARD_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::PROFISAFE_WAITING_FOR_PARAMETERS), "PROFISAFE_WAITING_FOR_PARAMETERS");
  EXPECT_EQ(safetyModeString(SafetyMode::PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_STOP), "PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_"
                                                                                   "STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::PROFISAFE_SAFEGUARD_STOP), "PROFISAFE_SAFEGUARD_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::PROFISAFE_EMERGENCY_STOP), "PROFISAFE_EMERGENCY_STOP");
  EXPECT_EQ(safetyModeString(SafetyMode::SAFETY_API_SAFEGUARD_STOP), "SAFETY_API_SAFEGUARD_STOP");
}

TEST(TestDatatypes, SafetyModeString_invalid_throws)
{
  const SafetyMode invalid = static_cast<SafetyMode>(0);
  EXPECT_THROW(safetyModeString(invalid), std::invalid_argument);

  const SafetyMode also_invalid = static_cast<SafetyMode>(250);
  EXPECT_THROW(safetyModeString(also_invalid), std::invalid_argument);
}

TEST(TestDatatypes, SafetyStatusString_all_values)
{
  EXPECT_EQ(safetyStatusString(SafetyStatus::NORMAL), "NORMAL");
  EXPECT_EQ(safetyStatusString(SafetyStatus::REDUCED), "REDUCED");
  EXPECT_EQ(safetyStatusString(SafetyStatus::PROTECTIVE_STOP), "PROTECTIVE_STOP");
  EXPECT_EQ(safetyStatusString(SafetyStatus::RECOVERY), "RECOVERY");
  EXPECT_EQ(safetyStatusString(SafetyStatus::SAFEGUARD_STOP), "SAFEGUARD_STOP");
  EXPECT_EQ(safetyStatusString(SafetyStatus::SYSTEM_EMERGENCY_STOP), "SYSTEM_EMERGENCY_STOP");
  EXPECT_EQ(safetyStatusString(SafetyStatus::ROBOT_EMERGENCY_STOP), "ROBOT_EMERGENCY_STOP");
  EXPECT_EQ(safetyStatusString(SafetyStatus::VIOLATION), "VIOLATION");
  EXPECT_EQ(safetyStatusString(SafetyStatus::FAULT), "FAULT");
  EXPECT_EQ(safetyStatusString(SafetyStatus::VALIDATE_JOINT_ID), "VALIDATE_JOINT_ID");
  EXPECT_EQ(safetyStatusString(SafetyStatus::UNDEFINED_SAFETY_MODE), "UNDEFINED_SAFETY_MODE");
  EXPECT_EQ(safetyStatusString(SafetyStatus::AUTOMATIC_MODE_SAFEGUARD_STOP), "AUTOMATIC_MODE_SAFEGUARD_STOP");
  EXPECT_EQ(safetyStatusString(SafetyStatus::SYSTEM_THREE_POSITION_ENABLING_STOP), "SYSTEM_THREE_POSITION_ENABLING_"
                                                                                   "STOP");
}

TEST(TestDatatypes, SafetyStatusString_invalid_throws)
{
  const SafetyStatus invalid = static_cast<SafetyStatus>(0);
  EXPECT_THROW(safetyStatusString(invalid), std::invalid_argument);

  const SafetyStatus also_invalid = static_cast<SafetyStatus>(99);
  EXPECT_THROW(safetyStatusString(also_invalid), std::invalid_argument);
}

TEST(TestDatatypes, RobotTypeString_all_values)
{
  EXPECT_EQ(robotTypeString(RobotType::UNDEFINED), "UNDEFINED");
  EXPECT_EQ(robotTypeString(RobotType::UR3), "UR3");
  EXPECT_EQ(robotTypeString(RobotType::UR5), "UR5");
  EXPECT_EQ(robotTypeString(RobotType::UR8LONG), "UR8_LONG");
  EXPECT_EQ(robotTypeString(RobotType::UR10), "UR10");
  EXPECT_EQ(robotTypeString(RobotType::UR15), "UR15");
  EXPECT_EQ(robotTypeString(RobotType::UR16), "UR16");
  EXPECT_EQ(robotTypeString(RobotType::UR18), "UR18");
  EXPECT_EQ(robotTypeString(RobotType::UR20), "UR20");
  EXPECT_EQ(robotTypeString(RobotType::UR30), "UR30");
}

TEST(TestDatatypes, RobotTypeString_invalid_throws)
{
  const RobotType invalid = static_cast<RobotType>(0);
  EXPECT_THROW(robotTypeString(invalid), std::invalid_argument);

  const RobotType also_invalid = static_cast<RobotType>(99);
  EXPECT_THROW(robotTypeString(also_invalid), std::invalid_argument);
}

TEST(TestDatatypes, RobotSeriesString_all_values)
{
  EXPECT_EQ(robotSeriesString(RobotSeries::UNDEFINED), "UNDEFINED");
  EXPECT_EQ(robotSeriesString(RobotSeries::CB3), "CB3");
  EXPECT_EQ(robotSeriesString(RobotSeries::E_SERIES), "E_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::UR_SERIES), "UR_SERIES");
}

TEST(TestDatatypes, RobotSeriesString_invalid_throws)
{
  const RobotSeries invalid = static_cast<RobotSeries>(0);
  EXPECT_THROW(robotSeriesString(invalid), std::invalid_argument);

  const RobotSeries also_invalid = static_cast<RobotSeries>(42);
  EXPECT_THROW(robotSeriesString(also_invalid), std::invalid_argument);
}
