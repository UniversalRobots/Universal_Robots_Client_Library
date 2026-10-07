// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 * This file contains enums for internal mode representations.
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------
#pragma once

#include <ur_client_library/types.h>
#include "ur_client_library/log.h"
#include <sstream>

namespace urcl
{
enum class RobotMode : int8_t
{
  UNKNOWN = -128,  // This is not defined by UR but only inside this driver
  NO_CONTROLLER = -1,
  DISCONNECTED = 0,
  CONFIRM_SAFETY = 1,
  BOOTING = 2,
  POWER_OFF = 3,
  POWER_ON = 4,
  IDLE = 5,
  BACKDRIVE = 6,
  RUNNING = 7,
  UPDATING_FIRMWARE = 8
};

/*!
 * \brief Maps a robot-mode number from the primary interface.
 *
 * Any value that is not a mode the controller defines becomes RobotMode::UNKNOWN.
 */
inline RobotMode robotModeFromWire(const int32_t wire_value)
{
  switch (wire_value)
  {
    case -1:
      return RobotMode::NO_CONTROLLER;
    case 0:
      return RobotMode::DISCONNECTED;
    case 1:
      return RobotMode::CONFIRM_SAFETY;
    case 2:
      return RobotMode::BOOTING;
    case 3:
      return RobotMode::POWER_OFF;
    case 4:
      return RobotMode::POWER_ON;
    case 5:
      return RobotMode::IDLE;
    case 6:
      return RobotMode::BACKDRIVE;
    case 7:
      return RobotMode::RUNNING;
    case 8:
      return RobotMode::UPDATING_FIRMWARE;
    default:
      URCL_LOG_ERROR("Unknown robot mode %d", wire_value);
      return RobotMode::UNKNOWN;
  }
}

enum class SafetyMode : uint8_t
{
  NORMAL = 1,
  REDUCED = 2,
  PROTECTIVE_STOP = 3,
  RECOVERY = 4,
  SAFEGUARD_STOP = 5,
  SYSTEM_EMERGENCY_STOP = 6,
  ROBOT_EMERGENCY_STOP = 7,
  VIOLATION = 8,
  FAULT = 9,
  VALIDATE_JOINT_ID = 10,
  UNDEFINED_SAFETY_MODE = 11,
  AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
  SYSTEM_THREE_POSITION_ENABLING_STOP = 13,
  TP_THREE_POSITION_ENABLING_STOP = 14,
  IMMI_EMERGENCY_STOP = 15,
  IMMI_SAFEGUARD_STOP = 16,
  PROFISAFE_WAITING_FOR_PARAMETERS = 17,
  PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_STOP = 18,
  PROFISAFE_SAFEGUARD_STOP = 19,
  PROFISAFE_EMERGENCY_STOP = 20,
  SAFETY_API_SAFEGUARD_STOP = 22
};

enum class SafetyStatus : int8_t  // Only available on 3.10/5.4
{
  NORMAL = 1,
  REDUCED = 2,
  PROTECTIVE_STOP = 3,
  RECOVERY = 4,
  SAFEGUARD_STOP = 5,
  SYSTEM_EMERGENCY_STOP = 6,
  ROBOT_EMERGENCY_STOP = 7,
  VIOLATION = 8,
  FAULT = 9,
  VALIDATE_JOINT_ID = 10,
  UNDEFINED_SAFETY_MODE = 11,
  AUTOMATIC_MODE_SAFEGUARD_STOP = 12,
  SYSTEM_THREE_POSITION_ENABLING_STOP = 13
};

enum class AnalogOutputType : int8_t
{
  SET_ON_TEACH_PENDANT = -1,
  CURRENT = 0,
  VOLTAGE = 1
};

enum class RobotType : int32_t
{
  UNDEFINED = -128,  // This is not defined by UR but only inside this driver
  UR5 = 1,
  UR10 = 2,
  UR3 = 3,
  UR16 = 4,
  UR18 = 5,
  UR8LONG = 6,
  UR20 = 7,
  UR30 = 8,
  UR15 = 9,
  UR10G_1750 = 12,
  UR17G_1300 = 13,
  UR18G_950 = 14
};

/*!
 * \brief Maps a robot-type number from the primary interface.
 *
 * Any value that is not a known robot type becomes RobotType::UNDEFINED.
 */
inline RobotType robotTypeFromWire(const int32_t wire_value)
{
  switch (wire_value)
  {
    case 1:
      return RobotType::UR5;
    case 2:
      return RobotType::UR10;
    case 3:
      return RobotType::UR3;
    case 4:
      return RobotType::UR16;
    case 5:
      return RobotType::UR18;
    case 6:
      return RobotType::UR8LONG;
    case 7:
      return RobotType::UR20;
    case 8:
      return RobotType::UR30;
    case 9:
      return RobotType::UR15;
    case 12:
      return RobotType::UR10G_1750;
    case 13:
      return RobotType::UR17G_1300;
    case 14:
      return RobotType::UR18G_950;
    default:
      URCL_LOG_ERROR("Unknown robot type %d", wire_value);
      return RobotType::UNDEFINED;
  }
}

enum class RobotSeries
{
  UNDEFINED = -128,
  CB3 = 1,
  E_SERIES = 2,
  UR_SERIES = 3,
  G_SERIES = 4
};

/*!
 * \brief Maps a robot-series number.
 *
 * Any value that is not a known series becomes RobotSeries::UNDEFINED.
 */
inline RobotSeries robotSeriesFromWire(const int32_t wire_value)
{
  switch (wire_value)
  {
    case 1:
      return RobotSeries::CB3;
    case 2:
      return RobotSeries::E_SERIES;
    case 3:
      return RobotSeries::UR_SERIES;
    case 4:
      return RobotSeries::G_SERIES;
    default:
      URCL_LOG_ERROR("Unknown robot series %d", wire_value);
      return RobotSeries::UNDEFINED;
  }
}

enum class ReportLevel : int32_t
{
  DEBUG = 0,
  INFO = 1,
  WARNING = 2,
  VIOLATION = 3,
  FAULT = 4,
  CRITICAL_FAULT = 5,
  DEVL_DEBUG = 128,
  DEVL_INFO = 129,
  DEVL_WARNING = 130,
  DEVL_VIOLATION = 131,
  DEVL_FAULT = 132,
  DEVL_CRITICAL_FAULT = 133
};

enum class ControlBoxType : uint16_t
{
  UNKNOWN = 0,
  CB5 = 5,
  CB7 = 7
};

/*!
 * \brief Maps a control-box type from the primary interface or from RTDE property byte 0.
 *
 * Primary packets have used two numberings: 1 and 5 mean CB5, 2 and 7 mean CB7. Any other value
 * is unknown. Early patch releases of 5.26 and 10.13 sent 1 and 2; later releases switched to 5
 * and 7 to match the control-box numbering used in the RTDE properties package.
 */
inline ControlBoxType controlBoxTypeFromWire(const uint16_t wire_value)
{
  switch (wire_value)
  {
    case 1:
    case 5:
      return ControlBoxType::CB5;
    case 2:
    case 7:
      return ControlBoxType::CB7;
    default:
      URCL_LOG_ERROR("Unknown control box type %u", static_cast<unsigned>(wire_value));
      return ControlBoxType::UNKNOWN;
  }
}

enum class ToolFlangeType : uint16_t
{
  UNKNOWN = 0,
  V1 = 1,
  V2 = 2
};

/*!
 * \brief Maps a tool-flange type from the primary interface.
 *
 * Any value other than 1 or 2 becomes ToolFlangeType::UNKNOWN.
 */
inline ToolFlangeType toolFlangeTypeFromWire(const uint16_t wire_value)
{
  switch (wire_value)
  {
    case 1:
      return ToolFlangeType::V1;
    case 2:
      return ToolFlangeType::V2;
    default:
      URCL_LOG_ERROR("Unknown tool flange type %u", static_cast<unsigned>(wire_value));
      return ToolFlangeType::UNKNOWN;
  }
}

inline std::string reportLevelString(const ReportLevel& code)
{
  switch (code)
  {
    case ReportLevel::DEBUG:
      return "DEBUG";
    case ReportLevel::INFO:
      return "INFO";
    case ReportLevel::WARNING:
      return "WARNING";
    case ReportLevel::VIOLATION:
      return "VIOLATION";
    case ReportLevel::FAULT:
      return "FAULT";
    case ReportLevel::CRITICAL_FAULT:
      return "CRITICAL_FAULT";
    case ReportLevel::DEVL_DEBUG:
      return "DEVL_DEBUG";
    case ReportLevel::DEVL_INFO:
      return "DEVL_INFO";
    case ReportLevel::DEVL_WARNING:
      return "DEVL_WARNING";
    case ReportLevel::DEVL_VIOLATION:
      return "DEVL_VIOLATION";
    case ReportLevel::DEVL_FAULT:
      return "DEVL_FAULT";
    case ReportLevel::DEVL_CRITICAL_FAULT:
      return "DEVL_CRITICAL_FAULT";
  }
  throw std::invalid_argument("Unknown report level: " + std::to_string(static_cast<int>(code)));
}

inline std::string robotModeString(const RobotMode& mode)
{
  switch (mode)
  {
    case RobotMode::NO_CONTROLLER:
      return "NO_CONTROLLER";
    case RobotMode::DISCONNECTED:
      return "DISCONNECTED";
    case RobotMode::CONFIRM_SAFETY:
      return "CONFIRM_SAFETY";
    case RobotMode::BOOTING:
      return "BOOTING";
    case RobotMode::POWER_OFF:
      return "POWER_OFF";
    case RobotMode::POWER_ON:
      return "POWER_ON";
    case RobotMode::IDLE:
      return "IDLE";
    case RobotMode::BACKDRIVE:
      return "BACKDRIVE";
    case RobotMode::RUNNING:
      return "RUNNING";
    case RobotMode::UPDATING_FIRMWARE:
      return "UPDATING_FIRMWARE";
    case RobotMode::UNKNOWN:
      return "UNKNOWN";
  }
  throw std::invalid_argument("Unknown robot mode: " + std::to_string(static_cast<int>(mode)));
}

inline std::string safetyModeString(const SafetyMode& mode)
{
  switch (mode)
  {
    case SafetyMode::NORMAL:
      return "NORMAL";
    case SafetyMode::REDUCED:
      return "REDUCED";
    case SafetyMode::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyMode::RECOVERY:
      return "RECOVERY";
    case SafetyMode::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyMode::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyMode::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyMode::VIOLATION:
      return "VIOLATION";
    case SafetyMode::FAULT:
      return "FAULT";
    case SafetyMode::VALIDATE_JOINT_ID:
      return "VALIDATE_JOINT_ID";
    case SafetyMode::UNDEFINED_SAFETY_MODE:
      return "UNDEFINED_SAFETY_MODE";
    case SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP:
      return "AUTOMATIC_MODE_SAFEGUARD_STOP";
    case SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP:
      return "SYSTEM_THREE_POSITION_ENABLING_STOP";
    case SafetyMode::TP_THREE_POSITION_ENABLING_STOP:
      return "TP_THREE_POSITION_ENABLING_STOP";
    case SafetyMode::IMMI_EMERGENCY_STOP:
      return "IMMI_EMERGENCY_STOP";
    case SafetyMode::IMMI_SAFEGUARD_STOP:
      return "IMMI_SAFEGUARD_STOP";
    case SafetyMode::PROFISAFE_WAITING_FOR_PARAMETERS:
      return "PROFISAFE_WAITING_FOR_PARAMETERS";
    case SafetyMode::PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_STOP:
      return "PROFISAFE_AUTOMATIC_MODE_SAFEGUARD_STOP";
    case SafetyMode::PROFISAFE_SAFEGUARD_STOP:
      return "PROFISAFE_SAFEGUARD_STOP";
    case SafetyMode::PROFISAFE_EMERGENCY_STOP:
      return "PROFISAFE_EMERGENCY_STOP";
    case SafetyMode::SAFETY_API_SAFEGUARD_STOP:
      return "SAFETY_API_SAFEGUARD_STOP";
  }
  throw std::invalid_argument("Unknown safety mode: " + std::to_string(static_cast<int>(mode)));
}

inline std::string safetyStatusString(const SafetyStatus& status)
{
  switch (status)
  {
    case SafetyStatus::NORMAL:
      return "NORMAL";
    case SafetyStatus::REDUCED:
      return "REDUCED";
    case SafetyStatus::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyStatus::RECOVERY:
      return "RECOVERY";
    case SafetyStatus::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyStatus::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyStatus::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyStatus::VIOLATION:
      return "VIOLATION";
    case SafetyStatus::FAULT:
      return "FAULT";
    case SafetyStatus::VALIDATE_JOINT_ID:
      return "VALIDATE_JOINT_ID";
    case SafetyStatus::UNDEFINED_SAFETY_MODE:
      return "UNDEFINED_SAFETY_MODE";
    case SafetyStatus::AUTOMATIC_MODE_SAFEGUARD_STOP:
      return "AUTOMATIC_MODE_SAFEGUARD_STOP";
    case SafetyStatus::SYSTEM_THREE_POSITION_ENABLING_STOP:
      return "SYSTEM_THREE_POSITION_ENABLING_STOP";
  }
  throw std::invalid_argument("Unknown safety status: " + std::to_string(static_cast<int>(status)));
}

inline std::string robotTypeString(const RobotType& type)
{
  switch (type)
  {
    case RobotType::UR3:
      return "UR3";
    case RobotType::UR5:
      return "UR5";
    case RobotType::UR8LONG:
      return "UR8_LONG";
    case RobotType::UR10:
      return "UR10";
    case RobotType::UR15:
      return "UR15";
    case RobotType::UR16:
      return "UR16";
    case RobotType::UR18:
      return "UR18";
    case RobotType::UR20:
      return "UR20";
    case RobotType::UR30:
      return "UR30";
    case RobotType::UR10G_1750:
      return "UR10g-1750";
    case RobotType::UR17G_1300:
      return "UR17g-1300";
    case RobotType::UR18G_950:
      return "UR18g-950";
    case RobotType::UNDEFINED:
      return "UNDEFINED";
  }
  throw std::invalid_argument("Unknown robot type: " + std::to_string(static_cast<int>(type)));
}

/**
 * @brief Converts a RobotSeries enum value to its corresponding string representation.
 *
 * This function takes a RobotSeries enum value and returns a string that represents the robot series.
 * If the provided RobotSeries value does not match any known series, it logs a warning and returns "UNDEFINED".
 *
 * @param series The RobotSeries enum value to convert.
 * @return A string representation of the robot series.
 */
inline std::string robotSeriesString(const RobotSeries& series)
{
  switch (series)
  {
    case RobotSeries::CB3:
      return "CB3";
    case RobotSeries::E_SERIES:
      return "E_SERIES";
    case RobotSeries::UR_SERIES:
      return "UR_SERIES";
    case RobotSeries::G_SERIES:
      return "G_SERIES";
    case RobotSeries::UNDEFINED:
      return "UNDEFINED";
  }
  throw std::invalid_argument("Unknown robot series: " + std::to_string(static_cast<int>(series)));
}

/*!
 * \brief Converts a control-box type to its name.
 *
 * \returns "CB5", "CB7", or "UNKNOWN". A value outside the enum throws std::invalid_argument.
 */
inline std::string controlBoxTypeString(const ControlBoxType type)
{
  switch (type)
  {
    case ControlBoxType::UNKNOWN:
      return "UNKNOWN";
    case ControlBoxType::CB5:
      return "CB5";
    case ControlBoxType::CB7:
      return "CB7";
  }
  throw std::invalid_argument("Unknown control box type: " + std::to_string(static_cast<int>(type)));
}

}  // namespace urcl
