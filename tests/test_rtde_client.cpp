// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-09
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <utility>
#include "ur_client_library/exceptions.h"

#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/version_information.h>

using namespace urcl;

std::string ROBOT_IP = "192.168.56.101";

// Based on https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
using MinCbSeriesVersion = std::optional<std::string>;
using MinESeriesVersion = std::optional<std::string>;
using OutputNamesWithCompatibility = std::unordered_map<std::string, std::pair<MinCbSeriesVersion, MinESeriesVersion>>;
const OutputNamesWithCompatibility EXHAUSTIVE_OUTPUTS_WITH_COMPATIBILITY = {
  { "timestamp", { std::nullopt, std::nullopt } },
  { "target_q", { std::nullopt, std::nullopt } },
  { "target_qd", { std::nullopt, std::nullopt } },
  { "target_qdd", { std::nullopt, std::nullopt } },
  { "target_current", { std::nullopt, std::nullopt } },
  { "target_moment", { std::nullopt, std::nullopt } },
  { "actual_q", { std::nullopt, std::nullopt } },
  { "actual_qd", { std::nullopt, std::nullopt } },
  { "actual_current", { std::nullopt, std::nullopt } },
  { "actual_current_window", { std::nullopt, std::nullopt } },
  { "joint_control_output", { std::nullopt, std::nullopt } },
  { "actual_TCP_pose", { std::nullopt, std::nullopt } },
  { "actual_TCP_speed", { std::nullopt, std::nullopt } },
  { "actual_TCP_force", { std::nullopt, std::nullopt } },
  { "target_TCP_pose", { std::nullopt, std::nullopt } },
  { "target_TCP_speed", { std::nullopt, std::nullopt } },
  { "actual_digital_input_bits", { std::nullopt, std::nullopt } },
  { "joint_temperatures", { std::nullopt, std::nullopt } },
  { "actual_execution_time", { std::nullopt, std::nullopt } },
  { "robot_mode", { std::nullopt, std::nullopt } },
  { "joint_mode", { std::nullopt, std::nullopt } },
  { "safety_mode", { std::nullopt, std::nullopt } },
  { "safety_status", { "3.10.0", "5.4.0" } },
  { "actual_tool_accelerometer", { std::nullopt, std::nullopt } },
  { "speed_scaling", { std::nullopt, std::nullopt } },
  { "target_speed_fraction", { std::nullopt, std::nullopt } },
  { "actual_momentum", { std::nullopt, std::nullopt } },
  { "actual_main_voltage", { std::nullopt, std::nullopt } },
  { "actual_robot_voltage", { std::nullopt, std::nullopt } },
  { "actual_robot_current", { std::nullopt, std::nullopt } },
  { "actual_joint_voltage", { std::nullopt, std::nullopt } },
  { "actual_digital_output_bits", { std::nullopt, std::nullopt } },
  { "runtime_state", { std::nullopt, std::nullopt } },
  { "elbow_position", { "3.5.0", "5.0.0" } },
  { "elbow_velocity", { "3.5.0", "5.0.0" } },
  { "robot_status_bits", { std::nullopt, std::nullopt } },
  { "safety_status_bits", { std::nullopt, std::nullopt } },
  { "analog_io_types", { std::nullopt, std::nullopt } },
  { "standard_analog_input0", { std::nullopt, std::nullopt } },
  { "standard_analog_input1", { std::nullopt, std::nullopt } },
  { "standard_analog_output0", { std::nullopt, std::nullopt } },
  { "standard_analog_output1", { std::nullopt, std::nullopt } },
  { "io_current", { std::nullopt, std::nullopt } },
  { "euromap67_input_bits", { std::nullopt, std::nullopt } },
  { "euromap67_output_bits", { std::nullopt, std::nullopt } },
  { "euromap67_24V_voltage", { std::nullopt, std::nullopt } },
  { "euromap67_24V_current", { std::nullopt, std::nullopt } },
  { "tool_mode", { std::nullopt, std::nullopt } },
  { "tool_analog_input_types", { std::nullopt, std::nullopt } },
  { "tool_analog_input0", { std::nullopt, std::nullopt } },
  { "tool_analog_input1", { std::nullopt, std::nullopt } },
  { "tool_output_voltage", { std::nullopt, std::nullopt } },
  { "tool_output_current", { std::nullopt, std::nullopt } },
  { "tool_temperature", { std::nullopt, std::nullopt } },
  { "tcp_force_scalar", { std::nullopt, std::nullopt } },
  { "output_bit_registers0_to_31", { std::nullopt, std::nullopt } },
  { "output_bit_registers32_to_63", { std::nullopt, std::nullopt } },
  { "output_bit_register_64", { "3.9.0", "5.3.0" } },
  { "output_bit_register_65", { "3.9.0", "5.3.0" } },
  { "output_bit_register_66", { "3.9.0", "5.3.0" } },
  { "output_bit_register_67", { "3.9.0", "5.3.0" } },
  { "output_bit_register_68", { "3.9.0", "5.3.0" } },
  { "output_bit_register_69", { "3.9.0", "5.3.0" } },
  { "output_bit_register_70", { "3.9.0", "5.3.0" } },
  { "output_bit_register_71", { "3.9.0", "5.3.0" } },
  { "output_bit_register_72", { "3.9.0", "5.3.0" } },
  { "output_bit_register_73", { "3.9.0", "5.3.0" } },
  { "output_bit_register_74", { "3.9.0", "5.3.0" } },
  { "output_bit_register_75", { "3.9.0", "5.3.0" } },
  { "output_bit_register_76", { "3.9.0", "5.3.0" } },
  { "output_bit_register_77", { "3.9.0", "5.3.0" } },
  { "output_bit_register_78", { "3.9.0", "5.3.0" } },
  { "output_bit_register_79", { "3.9.0", "5.3.0" } },
  { "output_bit_register_80", { "3.9.0", "5.3.0" } },
  { "output_bit_register_81", { "3.9.0", "5.3.0" } },
  { "output_bit_register_82", { "3.9.0", "5.3.0" } },
  { "output_bit_register_83", { "3.9.0", "5.3.0" } },
  { "output_bit_register_84", { "3.9.0", "5.3.0" } },
  { "output_bit_register_85", { "3.9.0", "5.3.0" } },
  { "output_bit_register_86", { "3.9.0", "5.3.0" } },
  { "output_bit_register_87", { "3.9.0", "5.3.0" } },
  { "output_bit_register_88", { "3.9.0", "5.3.0" } },
  { "output_bit_register_89", { "3.9.0", "5.3.0" } },
  { "output_bit_register_90", { "3.9.0", "5.3.0" } },
  { "output_bit_register_91", { "3.9.0", "5.3.0" } },
  { "output_bit_register_92", { "3.9.0", "5.3.0" } },
  { "output_bit_register_93", { "3.9.0", "5.3.0" } },
  { "output_bit_register_94", { "3.9.0", "5.3.0" } },
  { "output_bit_register_95", { "3.9.0", "5.3.0" } },
  { "output_bit_register_96", { "3.9.0", "5.3.0" } },
  { "output_bit_register_97", { "3.9.0", "5.3.0" } },
  { "output_bit_register_98", { "3.9.0", "5.3.0" } },
  { "output_bit_register_99", { "3.9.0", "5.3.0" } },
  { "output_bit_register_100", { "3.9.0", "5.3.0" } },
  { "output_bit_register_101", { "3.9.0", "5.3.0" } },
  { "output_bit_register_102", { "3.9.0", "5.3.0" } },
  { "output_bit_register_103", { "3.9.0", "5.3.0" } },
  { "output_bit_register_104", { "3.9.0", "5.3.0" } },
  { "output_bit_register_105", { "3.9.0", "5.3.0" } },
  { "output_bit_register_106", { "3.9.0", "5.3.0" } },
  { "output_bit_register_107", { "3.9.0", "5.3.0" } },
  { "output_bit_register_108", { "3.9.0", "5.3.0" } },
  { "output_bit_register_109", { "3.9.0", "5.3.0" } },
  { "output_bit_register_110", { "3.9.0", "5.3.0" } },
  { "output_bit_register_111", { "3.9.0", "5.3.0" } },
  { "output_bit_register_112", { "3.9.0", "5.3.0" } },
  { "output_bit_register_113", { "3.9.0", "5.3.0" } },
  { "output_bit_register_114", { "3.9.0", "5.3.0" } },
  { "output_bit_register_115", { "3.9.0", "5.3.0" } },
  { "output_bit_register_116", { "3.9.0", "5.3.0" } },
  { "output_bit_register_117", { "3.9.0", "5.3.0" } },
  { "output_bit_register_118", { "3.9.0", "5.3.0" } },
  { "output_bit_register_119", { "3.9.0", "5.3.0" } },
  { "output_bit_register_120", { "3.9.0", "5.3.0" } },
  { "output_bit_register_121", { "3.9.0", "5.3.0" } },
  { "output_bit_register_122", { "3.9.0", "5.3.0" } },
  { "output_bit_register_123", { "3.9.0", "5.3.0" } },
  { "output_bit_register_124", { "3.9.0", "5.3.0" } },
  { "output_bit_register_125", { "3.9.0", "5.3.0" } },
  { "output_bit_register_126", { "3.9.0", "5.3.0" } },
  { "output_bit_register_127", { "3.9.0", "5.3.0" } },
  { "output_int_register_0", { "3.4.0", std::nullopt } },
  { "output_int_register_1", { "3.4.0", std::nullopt } },
  { "output_int_register_2", { "3.4.0", std::nullopt } },
  { "output_int_register_3", { "3.4.0", std::nullopt } },
  { "output_int_register_4", { "3.4.0", std::nullopt } },
  { "output_int_register_5", { "3.4.0", std::nullopt } },
  { "output_int_register_6", { "3.4.0", std::nullopt } },
  { "output_int_register_7", { "3.4.0", std::nullopt } },
  { "output_int_register_8", { "3.4.0", std::nullopt } },
  { "output_int_register_9", { "3.4.0", std::nullopt } },
  { "output_int_register_10", { "3.4.0", std::nullopt } },
  { "output_int_register_11", { "3.4.0", std::nullopt } },
  { "output_int_register_12", { "3.4.0", std::nullopt } },
  { "output_int_register_13", { "3.4.0", std::nullopt } },
  { "output_int_register_14", { "3.4.0", std::nullopt } },
  { "output_int_register_15", { "3.4.0", std::nullopt } },
  { "output_int_register_16", { "3.4.0", std::nullopt } },
  { "output_int_register_17", { "3.4.0", std::nullopt } },
  { "output_int_register_18", { "3.4.0", std::nullopt } },
  { "output_int_register_19", { "3.4.0", std::nullopt } },
  { "output_int_register_20", { "3.4.0", std::nullopt } },
  { "output_int_register_21", { "3.4.0", std::nullopt } },
  { "output_int_register_22", { "3.4.0", std::nullopt } },
  { "output_int_register_23", { "3.4.0", std::nullopt } },
  { "output_int_register_24", { "3.9.0", "5.3.0" } },
  { "output_int_register_25", { "3.9.0", "5.3.0" } },
  { "output_int_register_26", { "3.9.0", "5.3.0" } },
  { "output_int_register_27", { "3.9.0", "5.3.0" } },
  { "output_int_register_28", { "3.9.0", "5.3.0" } },
  { "output_int_register_29", { "3.9.0", "5.3.0" } },
  { "output_int_register_30", { "3.9.0", "5.3.0" } },
  { "output_int_register_31", { "3.9.0", "5.3.0" } },
  { "output_int_register_32", { "3.9.0", "5.3.0" } },
  { "output_int_register_33", { "3.9.0", "5.3.0" } },
  { "output_int_register_34", { "3.9.0", "5.3.0" } },
  { "output_int_register_35", { "3.9.0", "5.3.0" } },
  { "output_int_register_36", { "3.9.0", "5.3.0" } },
  { "output_int_register_37", { "3.9.0", "5.3.0" } },
  { "output_int_register_38", { "3.9.0", "5.3.0" } },
  { "output_int_register_39", { "3.9.0", "5.3.0" } },
  { "output_int_register_40", { "3.9.0", "5.3.0" } },
  { "output_int_register_41", { "3.9.0", "5.3.0" } },
  { "output_int_register_42", { "3.9.0", "5.3.0" } },
  { "output_int_register_43", { "3.9.0", "5.3.0" } },
  { "output_int_register_44", { "3.9.0", "5.3.0" } },
  { "output_int_register_45", { "3.9.0", "5.3.0" } },
  { "output_int_register_46", { "3.9.0", "5.3.0" } },
  { "output_int_register_47", { "3.9.0", "5.3.0" } },
  { "output_double_register_0", { "3.4.0", std::nullopt } },
  { "output_double_register_1", { "3.4.0", std::nullopt } },
  { "output_double_register_2", { "3.4.0", std::nullopt } },
  { "output_double_register_3", { "3.4.0", std::nullopt } },
  { "output_double_register_4", { "3.4.0", std::nullopt } },
  { "output_double_register_5", { "3.4.0", std::nullopt } },
  { "output_double_register_6", { "3.4.0", std::nullopt } },
  { "output_double_register_7", { "3.4.0", std::nullopt } },
  { "output_double_register_8", { "3.4.0", std::nullopt } },
  { "output_double_register_9", { "3.4.0", std::nullopt } },
  { "output_double_register_10", { "3.4.0", std::nullopt } },
  { "output_double_register_11", { "3.4.0", std::nullopt } },
  { "output_double_register_12", { "3.4.0", std::nullopt } },
  { "output_double_register_13", { "3.4.0", std::nullopt } },
  { "output_double_register_14", { "3.4.0", std::nullopt } },
  { "output_double_register_15", { "3.4.0", std::nullopt } },
  { "output_double_register_16", { "3.4.0", std::nullopt } },
  { "output_double_register_17", { "3.4.0", std::nullopt } },
  { "output_double_register_18", { "3.4.0", std::nullopt } },
  { "output_double_register_19", { "3.4.0", std::nullopt } },
  { "output_double_register_20", { "3.4.0", std::nullopt } },
  { "output_double_register_21", { "3.4.0", std::nullopt } },
  { "output_double_register_22", { "3.4.0", std::nullopt } },
  { "output_double_register_23", { "3.4.0", std::nullopt } },
  { "output_double_register_24", { "3.9.0", "5.3.0" } },
  { "output_double_register_25", { "3.9.0", "5.3.0" } },
  { "output_double_register_26", { "3.9.0", "5.3.0" } },
  { "output_double_register_27", { "3.9.0", "5.3.0" } },
  { "output_double_register_28", { "3.9.0", "5.3.0" } },
  { "output_double_register_29", { "3.9.0", "5.3.0" } },
  { "output_double_register_30", { "3.9.0", "5.3.0" } },
  { "output_double_register_31", { "3.9.0", "5.3.0" } },
  { "output_double_register_32", { "3.9.0", "5.3.0" } },
  { "output_double_register_33", { "3.9.0", "5.3.0" } },
  { "output_double_register_34", { "3.9.0", "5.3.0" } },
  { "output_double_register_35", { "3.9.0", "5.3.0" } },
  { "output_double_register_36", { "3.9.0", "5.3.0" } },
  { "output_double_register_37", { "3.9.0", "5.3.0" } },
  { "output_double_register_38", { "3.9.0", "5.3.0" } },
  { "output_double_register_39", { "3.9.0", "5.3.0" } },
  { "output_double_register_40", { "3.9.0", "5.3.0" } },
  { "output_double_register_41", { "3.9.0", "5.3.0" } },
  { "output_double_register_42", { "3.9.0", "5.3.0" } },
  { "output_double_register_43", { "3.9.0", "5.3.0" } },
  { "output_double_register_44", { "3.9.0", "5.3.0" } },
  { "output_double_register_45", { "3.9.0", "5.3.0" } },
  { "output_double_register_46", { "3.9.0", "5.3.0" } },
  { "output_double_register_47", { "3.9.0", "5.3.0" } },
  { "input_bit_registers0_to_31", { "3.4.0", std::nullopt } },
  { "input_bit_registers32_to_63", { "3.4.0", std::nullopt } },
  { "input_bit_register_64", { "3.9.0", "5.3.0" } },
  { "input_bit_register_65", { "3.9.0", "5.3.0" } },
  { "input_bit_register_66", { "3.9.0", "5.3.0" } },
  { "input_bit_register_67", { "3.9.0", "5.3.0" } },
  { "input_bit_register_68", { "3.9.0", "5.3.0" } },
  { "input_bit_register_69", { "3.9.0", "5.3.0" } },
  { "input_bit_register_70", { "3.9.0", "5.3.0" } },
  { "input_bit_register_71", { "3.9.0", "5.3.0" } },
  { "input_bit_register_72", { "3.9.0", "5.3.0" } },
  { "input_bit_register_73", { "3.9.0", "5.3.0" } },
  { "input_bit_register_74", { "3.9.0", "5.3.0" } },
  { "input_bit_register_75", { "3.9.0", "5.3.0" } },
  { "input_bit_register_76", { "3.9.0", "5.3.0" } },
  { "input_bit_register_77", { "3.9.0", "5.3.0" } },
  { "input_bit_register_78", { "3.9.0", "5.3.0" } },
  { "input_bit_register_79", { "3.9.0", "5.3.0" } },
  { "input_bit_register_80", { "3.9.0", "5.3.0" } },
  { "input_bit_register_81", { "3.9.0", "5.3.0" } },
  { "input_bit_register_82", { "3.9.0", "5.3.0" } },
  { "input_bit_register_83", { "3.9.0", "5.3.0" } },
  { "input_bit_register_84", { "3.9.0", "5.3.0" } },
  { "input_bit_register_85", { "3.9.0", "5.3.0" } },
  { "input_bit_register_86", { "3.9.0", "5.3.0" } },
  { "input_bit_register_87", { "3.9.0", "5.3.0" } },
  { "input_bit_register_88", { "3.9.0", "5.3.0" } },
  { "input_bit_register_89", { "3.9.0", "5.3.0" } },
  { "input_bit_register_90", { "3.9.0", "5.3.0" } },
  { "input_bit_register_91", { "3.9.0", "5.3.0" } },
  { "input_bit_register_92", { "3.9.0", "5.3.0" } },
  { "input_bit_register_93", { "3.9.0", "5.3.0" } },
  { "input_bit_register_94", { "3.9.0", "5.3.0" } },
  { "input_bit_register_95", { "3.9.0", "5.3.0" } },
  { "input_bit_register_96", { "3.9.0", "5.3.0" } },
  { "input_bit_register_97", { "3.9.0", "5.3.0" } },
  { "input_bit_register_98", { "3.9.0", "5.3.0" } },
  { "input_bit_register_99", { "3.9.0", "5.3.0" } },
  { "input_bit_register_100", { "3.9.0", "5.3.0" } },
  { "input_bit_register_101", { "3.9.0", "5.3.0" } },
  { "input_bit_register_102", { "3.9.0", "5.3.0" } },
  { "input_bit_register_103", { "3.9.0", "5.3.0" } },
  { "input_bit_register_104", { "3.9.0", "5.3.0" } },
  { "input_bit_register_105", { "3.9.0", "5.3.0" } },
  { "input_bit_register_106", { "3.9.0", "5.3.0" } },
  { "input_bit_register_107", { "3.9.0", "5.3.0" } },
  { "input_bit_register_108", { "3.9.0", "5.3.0" } },
  { "input_bit_register_109", { "3.9.0", "5.3.0" } },
  { "input_bit_register_110", { "3.9.0", "5.3.0" } },
  { "input_bit_register_111", { "3.9.0", "5.3.0" } },
  { "input_bit_register_112", { "3.9.0", "5.3.0" } },
  { "input_bit_register_113", { "3.9.0", "5.3.0" } },
  { "input_bit_register_114", { "3.9.0", "5.3.0" } },
  { "input_bit_register_115", { "3.9.0", "5.3.0" } },
  { "input_bit_register_116", { "3.9.0", "5.3.0" } },
  { "input_bit_register_117", { "3.9.0", "5.3.0" } },
  { "input_bit_register_118", { "3.9.0", "5.3.0" } },
  { "input_bit_register_119", { "3.9.0", "5.3.0" } },
  { "input_bit_register_120", { "3.9.0", "5.3.0" } },
  { "input_bit_register_121", { "3.9.0", "5.3.0" } },
  { "input_bit_register_122", { "3.9.0", "5.3.0" } },
  { "input_bit_register_123", { "3.9.0", "5.3.0" } },
  { "input_bit_register_124", { "3.9.0", "5.3.0" } },
  { "input_bit_register_125", { "3.9.0", "5.3.0" } },
  { "input_bit_register_126", { "3.9.0", "5.3.0" } },
  { "input_bit_register_127", { "3.9.0", "5.3.0" } },
  { "input_double_register_0", { "3.4.0", std::nullopt } },
  { "input_double_register_1", { "3.4.0", std::nullopt } },
  { "input_double_register_2", { "3.4.0", std::nullopt } },
  { "input_double_register_3", { "3.4.0", std::nullopt } },
  { "input_double_register_4", { "3.4.0", std::nullopt } },
  { "input_double_register_5", { "3.4.0", std::nullopt } },
  { "input_double_register_6", { "3.4.0", std::nullopt } },
  { "input_double_register_7", { "3.4.0", std::nullopt } },
  { "input_double_register_8", { "3.4.0", std::nullopt } },
  { "input_double_register_9", { "3.4.0", std::nullopt } },
  { "input_double_register_10", { "3.4.0", std::nullopt } },
  { "input_double_register_11", { "3.4.0", std::nullopt } },
  { "input_double_register_12", { "3.4.0", std::nullopt } },
  { "input_double_register_13", { "3.4.0", std::nullopt } },
  { "input_double_register_14", { "3.4.0", std::nullopt } },
  { "input_double_register_15", { "3.4.0", std::nullopt } },
  { "input_double_register_16", { "3.4.0", std::nullopt } },
  { "input_double_register_17", { "3.4.0", std::nullopt } },
  { "input_double_register_18", { "3.4.0", std::nullopt } },
  { "input_double_register_19", { "3.4.0", std::nullopt } },
  { "input_double_register_20", { "3.4.0", std::nullopt } },
  { "input_double_register_21", { "3.4.0", std::nullopt } },
  { "input_double_register_22", { "3.4.0", std::nullopt } },
  { "input_double_register_23", { "3.4.0", std::nullopt } },
  { "input_double_register_24", { "3.9.0", "5.3.0" } },
  { "input_double_register_25", { "3.9.0", "5.3.0" } },
  { "input_double_register_26", { "3.9.0", "5.3.0" } },
  { "input_double_register_27", { "3.9.0", "5.3.0" } },
  { "input_double_register_28", { "3.9.0", "5.3.0" } },
  { "input_double_register_29", { "3.9.0", "5.3.0" } },
  { "input_double_register_30", { "3.9.0", "5.3.0" } },
  { "input_double_register_31", { "3.9.0", "5.3.0" } },
  { "input_double_register_32", { "3.9.0", "5.3.0" } },
  { "input_double_register_33", { "3.9.0", "5.3.0" } },
  { "input_double_register_34", { "3.9.0", "5.3.0" } },
  { "input_double_register_35", { "3.9.0", "5.3.0" } },
  { "input_double_register_36", { "3.9.0", "5.3.0" } },
  { "input_double_register_37", { "3.9.0", "5.3.0" } },
  { "input_double_register_38", { "3.9.0", "5.3.0" } },
  { "input_double_register_39", { "3.9.0", "5.3.0" } },
  { "input_double_register_40", { "3.9.0", "5.3.0" } },
  { "input_double_register_41", { "3.9.0", "5.3.0" } },
  { "input_double_register_42", { "3.9.0", "5.3.0" } },
  { "input_double_register_43", { "3.9.0", "5.3.0" } },
  { "input_double_register_44", { "3.9.0", "5.3.0" } },
  { "input_double_register_45", { "3.9.0", "5.3.0" } },
  { "input_double_register_46", { "3.9.0", "5.3.0" } },
  { "input_double_register_47", { "3.9.0", "5.3.0" } },
  { "tool_output_mode", { std::nullopt, std::nullopt } },
  { "tool_digital_output0_mode", { std::nullopt, std::nullopt } },
  { "tool_digital_output1_mode", { std::nullopt, std::nullopt } },
  { "payload", { "3.11.0", "5.5.1" } },
  { "payload_cog", { "3.11.0", "5.5.1" } },
  { "payload_inertia", { std::nullopt, std::nullopt } },
  { "script_control_line", { std::nullopt, std::nullopt } },
  { "ft_raw_wrench", { "5.9.0", "5.9.0" } },
  { "joint_position_deviation_ratio", { std::nullopt, std::nullopt } },
  { "collision_detection_ratio", { "5.15.0", "5.15.0" } },
  { "time_scale_source", { "5.17.0", "5.17.0" } }
};

class RTDEClientTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_));
  }

  void TearDown()
  {
    client_.reset();
    // If we don't sleep we can get a conflict between two tests controlling the same rtde inputs
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  void filterOutputRecipeToBeCompatibleWith(std::vector<std::string>& output_recipe, VersionInformation sw_version)
  {
    const auto get_min_version = [&sw_version](const std::string& output_name) -> std::optional<std::string> {
      return sw_version.isESeries() ? EXHAUSTIVE_OUTPUTS_WITH_COMPATIBILITY.at(output_name).second :
                                      EXHAUSTIVE_OUTPUTS_WITH_COMPATIBILITY.at(output_name).first;
    };
    const auto output_incompatible_with_version = [get_min_version,
                                                   &sw_version](const std::string& output_name) -> bool {
      const auto min_version = get_min_version(output_name);
      return min_version.has_value() && sw_version < VersionInformation::fromString(min_version.value());
    };

    output_recipe.erase(std::remove_if(output_recipe.begin(), output_recipe.end(), output_incompatible_with_version),
                        output_recipe.end());
  }

  std::string output_recipe_file_ = "resources/rtde_output_recipe.txt";
  std::string input_recipe_file_ = "resources/rtde_input_recipe.txt";
  comm::INotifier notifier_;
  std::unique_ptr<rtde_interface::RTDEClient> client_;

  std::vector<std::string> resources_output_recipe_ = { "timestamp",
                                                        "actual_q",
                                                        "actual_qd",
                                                        "speed_scaling",
                                                        "target_speed_fraction",
                                                        "runtime_state",
                                                        "actual_TCP_force",
                                                        "actual_TCP_pose",
                                                        "actual_digital_input_bits",
                                                        "actual_digital_output_bits",
                                                        "standard_analog_input0",
                                                        "standard_analog_input1",
                                                        "standard_analog_output0",
                                                        "standard_analog_output1",
                                                        "analog_io_types",
                                                        "tool_mode",
                                                        "tool_analog_input_types",
                                                        "tool_analog_input0",
                                                        "tool_analog_input1",
                                                        "tool_output_voltage",
                                                        "tool_output_current",
                                                        "tool_temperature",
                                                        "robot_mode",
                                                        "safety_mode",
                                                        "robot_status_bits",
                                                        "safety_status_bits",
                                                        "actual_current",
                                                        "tcp_offset" };

  std::vector<std::string> resources_input_recipe_ = { "speed_slider_mask",
                                                       "speed_slider_fraction",
                                                       "standard_digital_output_mask",
                                                       "standard_digital_output",
                                                       "configurable_digital_output_mask",
                                                       "configurable_digital_output",
                                                       "tool_digital_output_mask",
                                                       "tool_digital_output",
                                                       "standard_analog_output_mask",
                                                       "standard_analog_output_type",
                                                       "standard_analog_output_0",
                                                       "standard_analog_output_1" };
};

TEST_F(RTDEClientTest, rtde_handshake)
{
  EXPECT_TRUE(client_->init());
}

TEST_F(RTDEClientTest, no_recipe)
{
  std::string output_recipe_file = "";
  std::string input_recipe_file = "";
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file, input_recipe_file)),
      UrException);

  // Only input recipe is unconfigured
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file)),
      UrException);
}

TEST_F(RTDEClientTest, empty_recipe_file)
{
  std::string output_recipe_file = "resources/empty.txt";
  std::string input_recipe_file = "resources/empty.txt";
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file, input_recipe_file)),
      UrException);

  // Only input recipe is empty
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file)),
      UrException);
}

TEST_F(RTDEClientTest, invalid_target_frequency)
{
  // Setting target frequency below 0 or above 500, should throw an exception
  client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_, -1.0));

  EXPECT_THROW(client_->init(), UrException);

  client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_, 1000));

  EXPECT_THROW(client_->init(), UrException);
}

TEST_F(RTDEClientTest, unconfigured_target_frequency)
{
  // When the target frequency is unconfigured, it should be zero before the client has been initialized
  double expected_target_frequency = 0.0;
  EXPECT_EQ(client_->getTargetFrequency(), expected_target_frequency);

  client_->init();

  // When the target frequency is unconfigured, it should be equal to the maximum frequency after initialization
  EXPECT_EQ(client_->getTargetFrequency(), client_->getMaxFrequency());
}

TEST_F(RTDEClientTest, set_target_frequency)
{
  client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_, 1));
  client_->init();

  // Maximum frequency should still be equal to the robot's maximum frequency
  if (client_->getVersion().major >= 5)
  {
    double expected_max_frequency = 500;
    EXPECT_EQ(client_->getMaxFrequency(), expected_max_frequency);
  }
  else
  {
    double expected_max_frequency = 125;
    EXPECT_EQ(client_->getMaxFrequency(), expected_max_frequency);
  }

  double expected_target_frequency = 1;
  EXPECT_EQ(client_->getTargetFrequency(), expected_target_frequency);

  EXPECT_TRUE(client_->start());

  // Test that we receive packages with a frequency of 2 Hz
  const std::chrono::milliseconds read_timeout{ 10000 };
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(read_timeout);
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  double first_time_stamp = 0.0;
  data_pkg->getData("timestamp", first_time_stamp);

  data_pkg = client_->getDataPackage(read_timeout);
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  double second_time_stamp = 0.0;
  data_pkg->getData("timestamp", second_time_stamp);

  // There should be 1 second between each timestamp
  EXPECT_EQ(second_time_stamp - first_time_stamp, 1);

  client_->pause();
}

TEST_F(RTDEClientTest, start_uninitialized_client)
{
  // It shouldn't be possible to start an uninitialized client
  EXPECT_FALSE(client_->start());
}

TEST_F(RTDEClientTest, start_client)
{
  client_->init();
  EXPECT_TRUE(client_->start());

  client_->pause();

  // We should be able to start the client again after it has been paused
  EXPECT_TRUE(client_->start());

  client_->pause();
}

TEST_F(RTDEClientTest, pause_client_before_it_was_started)
{
  // We shouldn't be able to pause the client before it has been initialized
  EXPECT_FALSE(client_->pause());

  // We shouldn't be able to pause the client after it has been initialized
  client_->init();
  EXPECT_FALSE(client_->pause());
}

TEST_F(RTDEClientTest, pause_client)
{
  client_->init();
  client_->start();

  EXPECT_TRUE(client_->pause());
}

TEST_F(RTDEClientTest, output_recipe_file)
{
  std::vector<std::string> actual_output_recipe = client_->getOutputRecipe();
  // Verify that the size is the same
  ASSERT_EQ(resources_output_recipe_.size(), actual_output_recipe.size());

  // Verify that the order and contect is equal
  for (unsigned int i = 0; i < resources_output_recipe_.size(); ++i)
  {
    EXPECT_EQ(resources_output_recipe_[i], actual_output_recipe[i]);
  }
}

TEST_F(RTDEClientTest, recipe_compairson)
{
  // Check that vectorized constructor provides same recipes as from file
  auto client = rtde_interface::RTDEClient(ROBOT_IP, notifier_, resources_output_recipe_, resources_input_recipe_);

  std::vector<std::string> output_recipe_from_file = client_->getOutputRecipe();
  std::vector<std::string> output_recipe_from_vector = client.getOutputRecipe();
  for (unsigned int i = 0; i < output_recipe_from_file.size(); ++i)
  {
    EXPECT_EQ(output_recipe_from_file[i], output_recipe_from_vector[i]);
  }
}

TEST_F(RTDEClientTest, get_data_package)
{
  client_->init();
  client_->start();

  // Test that we can receive a package and extract data from the received package
  const std::chrono::milliseconds read_timeout{ 100 };
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(read_timeout);
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  urcl::vector6d_t actual_q;
  EXPECT_TRUE(data_pkg->getData("actual_q", actual_q));

  client_->pause();
}

TEST_F(RTDEClientTest, write_rtde_data)
{
  client_->init();
  client_->start();

  bool send_digital_output = true;
  EXPECT_TRUE(client_->getWriter().sendStandardDigitalOutput(0, send_digital_output));

  // Make sure that the data has been written to the robot
  const std::chrono::milliseconds read_timeout{ 100 };
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(read_timeout);
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  std::bitset<18> actual_dig_out_bits;
  data_pkg->getData<uint64_t>("actual_digital_output_bits", actual_dig_out_bits);

  // If we get the data package to soon the digital output might not have been updated, therefore we get the package a
  // couple of times
  int max_tries = 100;
  int counter = 0;
  while (actual_dig_out_bits[0] != send_digital_output)
  {
    data_pkg = client_->getDataPackage(read_timeout);
    data_pkg->getData<uint64_t>("actual_digital_output_bits", actual_dig_out_bits);
    if (counter == max_tries)
    {
      break;
    }
    counter++;
  }

  EXPECT_EQ(send_digital_output, actual_dig_out_bits[0]);

  client_->pause();
}

TEST_F(RTDEClientTest, output_recipe_without_timestamp)
{
  std::string output_recipe_file = "resources/rtde_output_recipe_without_timestamp.txt";
  client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe_file, input_recipe_file_));

  std::vector<std::string> actual_output_recipe_from_file = client_->getOutputRecipe();
  const std::string timestamp = "timestamp";
  auto it = std::find(actual_output_recipe_from_file.begin(), actual_output_recipe_from_file.end(), timestamp);
  EXPECT_FALSE(it == actual_output_recipe_from_file.end());

  // Verify that timestamp is added to the recipe when using the vectorized constructor
  std::vector<std::string> output_recipe = { "actual_q", "actual_qd" };
  auto client = rtde_interface::RTDEClient(ROBOT_IP, notifier_, output_recipe, resources_input_recipe_);
  std::vector<std::string> actual_output_recipe_from_vector = client.getOutputRecipe();
  it = std::find(actual_output_recipe_from_vector.begin(), actual_output_recipe_from_vector.end(), timestamp);
  EXPECT_FALSE(it == actual_output_recipe_from_vector.end());
}

TEST_F(RTDEClientTest, connect_non_running_robot)
{
  // We use an IP address on the integration_test's subnet
  client_.reset(
      new rtde_interface::RTDEClient("192.168.56.123", notifier_, resources_output_recipe_, resources_input_recipe_));
  auto start = std::chrono::system_clock::now();
  EXPECT_THROW(client_->init(2, std::chrono::milliseconds(500)), UrException);
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  // This is only a rough estimate, obviously.
  // Since this isn't done on the loopback device, trying to open a socket on a non-existing address
  // takes considerably longer.
  EXPECT_LT(elapsed, 2 * comm::TCPSocket::DEFAULT_RECONNECTION_TIME);
}

TEST_F(RTDEClientTest, check_all_rtde_output_variables_exist)
{
  client_->init();

  // Create an output recipe as exhaustive as possible but compatible with the current version
  std::vector<std::string> exhaustive_compatible_output_recipe;

  const VersionInformation sw_version = client_->getVersion();
  std::transform(
      EXHAUSTIVE_OUTPUTS_WITH_COMPATIBILITY.begin(), EXHAUSTIVE_OUTPUTS_WITH_COMPATIBILITY.end(),
      std::back_inserter(exhaustive_compatible_output_recipe),
      [](OutputNamesWithCompatibility::const_reference data_name) -> std::string { return data_name.first; });
  filterOutputRecipeToBeCompatibleWith(exhaustive_compatible_output_recipe, sw_version);

  client_.reset(new rtde_interface::RTDEClient(ROBOT_IP, notifier_, exhaustive_compatible_output_recipe,
                                               resources_input_recipe_));

  EXPECT_NO_THROW(client_->init());
  client_->start();

  // Test that we can receive and parse the timestamp from the received package to prove the setup was successful
  const std::chrono::milliseconds read_timeout{ 100 };
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(read_timeout);

  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  double timestamp;
  EXPECT_TRUE(data_pkg->getData("timestamp", timestamp));

  client_->pause();
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
