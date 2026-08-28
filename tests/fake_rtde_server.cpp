#include "fake_rtde_server.h"
#include "rtde_test_helpers.h"
#include <stdexcept>
#include <unordered_map>
#include "ur_client_library/comm/package_serializer.h"
#include "ur_client_library/log.h"

namespace urcl
{
namespace
{
// The RTDE data type of every field a robot knows about. On a real robot this information is part
// of the answer to a recipe setup request, which is where the client library takes it from, so the
// test double has to be able to answer the same way.
//
// tests/resources/generate_rtde_outputs.py reads the output fields out of this table, so keep the
// section comments below intact.
// clang-format off
const std::unordered_map<std::string, std::string> g_variable_types{
  // INPUTS
  { "speed_slider_mask", "UINT32" },
  { "speed_slider_fraction", "DOUBLE" },
  { "standard_digital_output_mask", "UINT8" },
  { "standard_digital_output", "UINT8" },
  { "configurable_digital_output_mask", "UINT8" },
  { "configurable_digital_output", "UINT8" },
  { "standard_analog_output_mask", "UINT8" },
  { "standard_analog_output_type", "UINT8" },
  { "standard_analog_output_0", "DOUBLE" },
  { "standard_analog_output_1", "DOUBLE" },
  { "external_force_torque", "VECTOR6D" },

  // INPUT / OUTPUT
  { "input_bit_registers0_to_31", "UINT32" },
  { "input_bit_registers32_to_63", "UINT32" },
  { "input_bit_register_64", "BOOL" },
  { "input_bit_register_65", "BOOL" },
  { "input_bit_register_66", "BOOL" },
  { "input_bit_register_67", "BOOL" },
  { "input_bit_register_68", "BOOL" },
  { "input_bit_register_69", "BOOL" },
  { "input_bit_register_70", "BOOL" },
  { "input_bit_register_71", "BOOL" },
  { "input_bit_register_72", "BOOL" },
  { "input_bit_register_73", "BOOL" },
  { "input_bit_register_74", "BOOL" },
  { "input_bit_register_75", "BOOL" },
  { "input_bit_register_76", "BOOL" },
  { "input_bit_register_77", "BOOL" },
  { "input_bit_register_78", "BOOL" },
  { "input_bit_register_79", "BOOL" },
  { "input_bit_register_80", "BOOL" },
  { "input_bit_register_81", "BOOL" },
  { "input_bit_register_82", "BOOL" },
  { "input_bit_register_83", "BOOL" },
  { "input_bit_register_84", "BOOL" },
  { "input_bit_register_85", "BOOL" },
  { "input_bit_register_86", "BOOL" },
  { "input_bit_register_87", "BOOL" },
  { "input_bit_register_88", "BOOL" },
  { "input_bit_register_89", "BOOL" },
  { "input_bit_register_90", "BOOL" },
  { "input_bit_register_91", "BOOL" },
  { "input_bit_register_92", "BOOL" },
  { "input_bit_register_93", "BOOL" },
  { "input_bit_register_94", "BOOL" },
  { "input_bit_register_95", "BOOL" },
  { "input_bit_register_96", "BOOL" },
  { "input_bit_register_97", "BOOL" },
  { "input_bit_register_98", "BOOL" },
  { "input_bit_register_99", "BOOL" },
  { "input_bit_register_100", "BOOL" },
  { "input_bit_register_101", "BOOL" },
  { "input_bit_register_102", "BOOL" },
  { "input_bit_register_103", "BOOL" },
  { "input_bit_register_104", "BOOL" },
  { "input_bit_register_105", "BOOL" },
  { "input_bit_register_106", "BOOL" },
  { "input_bit_register_107", "BOOL" },
  { "input_bit_register_108", "BOOL" },
  { "input_bit_register_109", "BOOL" },
  { "input_bit_register_110", "BOOL" },
  { "input_bit_register_111", "BOOL" },
  { "input_bit_register_112", "BOOL" },
  { "input_bit_register_113", "BOOL" },
  { "input_bit_register_114", "BOOL" },
  { "input_bit_register_115", "BOOL" },
  { "input_bit_register_116", "BOOL" },
  { "input_bit_register_117", "BOOL" },
  { "input_bit_register_118", "BOOL" },
  { "input_bit_register_119", "BOOL" },
  { "input_bit_register_120", "BOOL" },
  { "input_bit_register_121", "BOOL" },
  { "input_bit_register_122", "BOOL" },
  { "input_bit_register_123", "BOOL" },
  { "input_bit_register_124", "BOOL" },
  { "input_bit_register_125", "BOOL" },
  { "input_bit_register_126", "BOOL" },
  { "input_bit_register_127", "BOOL" },
  { "input_int_register_0", "INT32" },
  { "input_int_register_1", "INT32" },
  { "input_int_register_2", "INT32" },
  { "input_int_register_3", "INT32" },
  { "input_int_register_4", "INT32" },
  { "input_int_register_5", "INT32" },
  { "input_int_register_6", "INT32" },
  { "input_int_register_7", "INT32" },
  { "input_int_register_8", "INT32" },
  { "input_int_register_9", "INT32" },
  { "input_int_register_10", "INT32" },
  { "input_int_register_11", "INT32" },
  { "input_int_register_12", "INT32" },
  { "input_int_register_13", "INT32" },
  { "input_int_register_14", "INT32" },
  { "input_int_register_15", "INT32" },
  { "input_int_register_16", "INT32" },
  { "input_int_register_17", "INT32" },
  { "input_int_register_18", "INT32" },
  { "input_int_register_19", "INT32" },
  { "input_int_register_20", "INT32" },
  { "input_int_register_21", "INT32" },
  { "input_int_register_22", "INT32" },
  { "input_int_register_23", "INT32" },
  { "input_int_register_24", "INT32" },
  { "input_int_register_25", "INT32" },
  { "input_int_register_26", "INT32" },
  { "input_int_register_27", "INT32" },
  { "input_int_register_28", "INT32" },
  { "input_int_register_29", "INT32" },
  { "input_int_register_30", "INT32" },
  { "input_int_register_31", "INT32" },
  { "input_int_register_32", "INT32" },
  { "input_int_register_33", "INT32" },
  { "input_int_register_34", "INT32" },
  { "input_int_register_35", "INT32" },
  { "input_int_register_36", "INT32" },
  { "input_int_register_37", "INT32" },
  { "input_int_register_38", "INT32" },
  { "input_int_register_39", "INT32" },
  { "input_int_register_40", "INT32" },
  { "input_int_register_41", "INT32" },
  { "input_int_register_42", "INT32" },
  { "input_int_register_43", "INT32" },
  { "input_int_register_44", "INT32" },
  { "input_int_register_45", "INT32" },
  { "input_int_register_46", "INT32" },
  { "input_int_register_47", "INT32" },
  { "input_double_register_0", "DOUBLE" },
  { "input_double_register_1", "DOUBLE" },
  { "input_double_register_2", "DOUBLE" },
  { "input_double_register_3", "DOUBLE" },
  { "input_double_register_4", "DOUBLE" },
  { "input_double_register_5", "DOUBLE" },
  { "input_double_register_6", "DOUBLE" },
  { "input_double_register_7", "DOUBLE" },
  { "input_double_register_8", "DOUBLE" },
  { "input_double_register_9", "DOUBLE" },
  { "input_double_register_10", "DOUBLE" },
  { "input_double_register_11", "DOUBLE" },
  { "input_double_register_12", "DOUBLE" },
  { "input_double_register_13", "DOUBLE" },
  { "input_double_register_14", "DOUBLE" },
  { "input_double_register_15", "DOUBLE" },
  { "input_double_register_16", "DOUBLE" },
  { "input_double_register_17", "DOUBLE" },
  { "input_double_register_18", "DOUBLE" },
  { "input_double_register_19", "DOUBLE" },
  { "input_double_register_20", "DOUBLE" },
  { "input_double_register_21", "DOUBLE" },
  { "input_double_register_22", "DOUBLE" },
  { "input_double_register_23", "DOUBLE" },
  { "input_double_register_24", "DOUBLE" },
  { "input_double_register_25", "DOUBLE" },
  { "input_double_register_26", "DOUBLE" },
  { "input_double_register_27", "DOUBLE" },
  { "input_double_register_28", "DOUBLE" },
  { "input_double_register_29", "DOUBLE" },
  { "input_double_register_30", "DOUBLE" },
  { "input_double_register_31", "DOUBLE" },
  { "input_double_register_32", "DOUBLE" },
  { "input_double_register_33", "DOUBLE" },
  { "input_double_register_34", "DOUBLE" },
  { "input_double_register_35", "DOUBLE" },
  { "input_double_register_36", "DOUBLE" },
  { "input_double_register_37", "DOUBLE" },
  { "input_double_register_38", "DOUBLE" },
  { "input_double_register_39", "DOUBLE" },
  { "input_double_register_40", "DOUBLE" },
  { "input_double_register_41", "DOUBLE" },
  { "input_double_register_42", "DOUBLE" },
  { "input_double_register_43", "DOUBLE" },
  { "input_double_register_44", "DOUBLE" },
  { "input_double_register_45", "DOUBLE" },
  { "input_double_register_46", "DOUBLE" },
  { "input_double_register_47", "DOUBLE" },

  // OUTPUTS
  { "timestamp", "DOUBLE" },
  { "target_q", "VECTOR6D" },
  { "target_qd", "VECTOR6D" },
  { "target_qdd", "VECTOR6D" },
  { "target_current", "VECTOR6D" },
  { "target_moment", "VECTOR6D" },
  { "actual_q", "VECTOR6D" },
  { "actual_qd", "VECTOR6D" },
  { "actual_current", "VECTOR6D" },
  { "actual_current_window", "VECTOR6D" },
  { "actual_current_as_torque", "VECTOR6D" },
  { "joint_control_output", "VECTOR6D" },
  { "actual_TCP_pose", "VECTOR6D" },
  { "actual_TCP_speed", "VECTOR6D" },
  { "actual_TCP_force", "VECTOR6D" },
  { "target_TCP_pose", "VECTOR6D" },
  { "target_TCP_speed", "VECTOR6D" },
  { "tcp_offset", "VECTOR6D" },
  { "actual_TCP_acceleration", "VECTOR6D" },
  { "target_TCP_acceleration", "VECTOR6D" },
  { "actual_digital_input_bits", "UINT64" },
  { "actual_configurable_digital_input_bits", "UINT64" },
  { "joint_temperatures", "VECTOR6D" },
  { "actual_execution_time", "DOUBLE" },
  { "target_execution_time", "DOUBLE" },
  { "robot_mode", "INT32" },
  { "joint_mode", "VECTOR6INT32" },
  { "safety_mode", "INT32" },
  { "safety_status", "INT32" },
  { "actual_tool_accelerometer", "VECTOR3D" },
  { "speed_scaling", "DOUBLE" },
  { "target_speed_fraction", "DOUBLE" },
  { "actual_momentum", "DOUBLE" },
  { "actual_main_voltage", "DOUBLE" },
  { "actual_robot_voltage", "DOUBLE" },
  { "actual_robot_current", "DOUBLE" },
  { "actual_joint_voltage", "VECTOR6D" },
  { "actual_digital_output_bits", "UINT64" },
  { "actual_configurable_digital_output_bits", "UINT64" },
  { "runtime_state", "UINT32" },
  { "elbow_position", "VECTOR3D" },
  { "elbow_velocity", "VECTOR3D" },
  { "robot_status_bits", "UINT32" },
  { "safety_status_bits", "UINT32" },
  { "analog_io_types", "UINT32" },
  { "standard_analog_input0", "DOUBLE" },
  { "standard_analog_input1", "DOUBLE" },
  { "standard_analog_output0", "DOUBLE" },
  { "standard_analog_output1", "DOUBLE" },
  { "io_current", "DOUBLE" },
  { "output_bit_registers0_to_31", "UINT32" },
  { "output_bit_registers32_to_63", "UINT32" },
  { "output_bit_register_64", "BOOL" },
  { "output_bit_register_65", "BOOL" },
  { "output_bit_register_66", "BOOL" },
  { "output_bit_register_67", "BOOL" },
  { "output_bit_register_68", "BOOL" },
  { "output_bit_register_69", "BOOL" },
  { "output_bit_register_70", "BOOL" },
  { "output_bit_register_71", "BOOL" },
  { "output_bit_register_72", "BOOL" },
  { "output_bit_register_73", "BOOL" },
  { "output_bit_register_74", "BOOL" },
  { "output_bit_register_75", "BOOL" },
  { "output_bit_register_76", "BOOL" },
  { "output_bit_register_77", "BOOL" },
  { "output_bit_register_78", "BOOL" },
  { "output_bit_register_79", "BOOL" },
  { "output_bit_register_80", "BOOL" },
  { "output_bit_register_81", "BOOL" },
  { "output_bit_register_82", "BOOL" },
  { "output_bit_register_83", "BOOL" },
  { "output_bit_register_84", "BOOL" },
  { "output_bit_register_85", "BOOL" },
  { "output_bit_register_86", "BOOL" },
  { "output_bit_register_87", "BOOL" },
  { "output_bit_register_88", "BOOL" },
  { "output_bit_register_89", "BOOL" },
  { "output_bit_register_90", "BOOL" },
  { "output_bit_register_91", "BOOL" },
  { "output_bit_register_92", "BOOL" },
  { "output_bit_register_93", "BOOL" },
  { "output_bit_register_94", "BOOL" },
  { "output_bit_register_95", "BOOL" },
  { "output_bit_register_96", "BOOL" },
  { "output_bit_register_97", "BOOL" },
  { "output_bit_register_98", "BOOL" },
  { "output_bit_register_99", "BOOL" },
  { "output_bit_register_100", "BOOL" },
  { "output_bit_register_101", "BOOL" },
  { "output_bit_register_102", "BOOL" },
  { "output_bit_register_103", "BOOL" },
  { "output_bit_register_104", "BOOL" },
  { "output_bit_register_105", "BOOL" },
  { "output_bit_register_106", "BOOL" },
  { "output_bit_register_107", "BOOL" },
  { "output_bit_register_108", "BOOL" },
  { "output_bit_register_109", "BOOL" },
  { "output_bit_register_110", "BOOL" },
  { "output_bit_register_111", "BOOL" },
  { "output_bit_register_112", "BOOL" },
  { "output_bit_register_113", "BOOL" },
  { "output_bit_register_114", "BOOL" },
  { "output_bit_register_115", "BOOL" },
  { "output_bit_register_116", "BOOL" },
  { "output_bit_register_117", "BOOL" },
  { "output_bit_register_118", "BOOL" },
  { "output_bit_register_119", "BOOL" },
  { "output_bit_register_120", "BOOL" },
  { "output_bit_register_121", "BOOL" },
  { "output_bit_register_122", "BOOL" },
  { "output_bit_register_123", "BOOL" },
  { "output_bit_register_124", "BOOL" },
  { "output_bit_register_125", "BOOL" },
  { "output_bit_register_126", "BOOL" },
  { "output_bit_register_127", "BOOL" },
  { "output_int_register_0", "INT32" },
  { "output_int_register_1", "INT32" },
  { "output_int_register_2", "INT32" },
  { "output_int_register_3", "INT32" },
  { "output_int_register_4", "INT32" },
  { "output_int_register_5", "INT32" },
  { "output_int_register_6", "INT32" },
  { "output_int_register_7", "INT32" },
  { "output_int_register_8", "INT32" },
  { "output_int_register_9", "INT32" },
  { "output_int_register_10", "INT32" },
  { "output_int_register_11", "INT32" },
  { "output_int_register_12", "INT32" },
  { "output_int_register_13", "INT32" },
  { "output_int_register_14", "INT32" },
  { "output_int_register_15", "INT32" },
  { "output_int_register_16", "INT32" },
  { "output_int_register_17", "INT32" },
  { "output_int_register_18", "INT32" },
  { "output_int_register_19", "INT32" },
  { "output_int_register_20", "INT32" },
  { "output_int_register_21", "INT32" },
  { "output_int_register_22", "INT32" },
  { "output_int_register_23", "INT32" },
  { "output_int_register_24", "INT32" },
  { "output_int_register_25", "INT32" },
  { "output_int_register_26", "INT32" },
  { "output_int_register_27", "INT32" },
  { "output_int_register_28", "INT32" },
  { "output_int_register_29", "INT32" },
  { "output_int_register_30", "INT32" },
  { "output_int_register_31", "INT32" },
  { "output_int_register_32", "INT32" },
  { "output_int_register_33", "INT32" },
  { "output_int_register_34", "INT32" },
  { "output_int_register_35", "INT32" },
  { "output_int_register_36", "INT32" },
  { "output_int_register_37", "INT32" },
  { "output_int_register_38", "INT32" },
  { "output_int_register_39", "INT32" },
  { "output_int_register_40", "INT32" },
  { "output_int_register_41", "INT32" },
  { "output_int_register_42", "INT32" },
  { "output_int_register_43", "INT32" },
  { "output_int_register_44", "INT32" },
  { "output_int_register_45", "INT32" },
  { "output_int_register_46", "INT32" },
  { "output_int_register_47", "INT32" },
  { "output_double_register_0", "DOUBLE" },
  { "output_double_register_1", "DOUBLE" },
  { "output_double_register_2", "DOUBLE" },
  { "output_double_register_3", "DOUBLE" },
  { "output_double_register_4", "DOUBLE" },
  { "output_double_register_5", "DOUBLE" },
  { "output_double_register_6", "DOUBLE" },
  { "output_double_register_7", "DOUBLE" },
  { "output_double_register_8", "DOUBLE" },
  { "output_double_register_9", "DOUBLE" },
  { "output_double_register_10", "DOUBLE" },
  { "output_double_register_11", "DOUBLE" },
  { "output_double_register_12", "DOUBLE" },
  { "output_double_register_13", "DOUBLE" },
  { "output_double_register_14", "DOUBLE" },
  { "output_double_register_15", "DOUBLE" },
  { "output_double_register_16", "DOUBLE" },
  { "output_double_register_17", "DOUBLE" },
  { "output_double_register_18", "DOUBLE" },
  { "output_double_register_19", "DOUBLE" },
  { "output_double_register_20", "DOUBLE" },
  { "output_double_register_21", "DOUBLE" },
  { "output_double_register_22", "DOUBLE" },
  { "output_double_register_23", "DOUBLE" },
  { "output_double_register_24", "DOUBLE" },
  { "output_double_register_25", "DOUBLE" },
  { "output_double_register_26", "DOUBLE" },
  { "output_double_register_27", "DOUBLE" },
  { "output_double_register_28", "DOUBLE" },
  { "output_double_register_29", "DOUBLE" },
  { "output_double_register_30", "DOUBLE" },
  { "output_double_register_31", "DOUBLE" },
  { "output_double_register_32", "DOUBLE" },
  { "output_double_register_33", "DOUBLE" },
  { "output_double_register_34", "DOUBLE" },
  { "output_double_register_35", "DOUBLE" },
  { "output_double_register_36", "DOUBLE" },
  { "output_double_register_37", "DOUBLE" },
  { "output_double_register_38", "DOUBLE" },
  { "output_double_register_39", "DOUBLE" },
  { "output_double_register_40", "DOUBLE" },
  { "output_double_register_41", "DOUBLE" },
  { "output_double_register_42", "DOUBLE" },
  { "output_double_register_43", "DOUBLE" },
  { "output_double_register_44", "DOUBLE" },
  { "output_double_register_45", "DOUBLE" },
  { "output_double_register_46", "DOUBLE" },
  { "output_double_register_47", "DOUBLE" },
  { "actual_robot_energy_consumed", "DOUBLE" },
  { "actual_robot_braking_energy_dissipated", "DOUBLE" },
  { "encoder0_raw", "INT32" },
  { "encoder1_raw", "INT32" },
  { "euromap67_input_bits", "UINT32" },
  { "euromap67_output_bits", "UINT32" },
  { "euromap67_24V_voltage", "DOUBLE" },
  { "euromap67_24V_current", "DOUBLE" },
  { "tool_mode", "UINT32" },
  { "tool_analog_input_types", "UINT32" },
  { "tool_analog_input0", "DOUBLE" },
  { "tool_analog_input1", "DOUBLE" },
  { "tool_output_voltage", "INT32" },
  { "tool_output_current", "DOUBLE" },
  { "tool_temperature", "DOUBLE" },
  { "tool_output_mode", "UINT8" },
  { "tool_digital_output0_mode", "UINT8" },
  { "tool_digital_output1_mode", "UINT8" },
  { "tcp_force_scalar", "DOUBLE" },
  { "joint_position_deviation_ratio", "DOUBLE" },
  { "collision_detection_ratio", "DOUBLE" },
  { "ft_raw_wrench", "VECTOR6D" },
  { "wrench_calc_from_currents", "VECTOR6D" },
  { "payload", "DOUBLE" },
  { "payload_cog", "VECTOR3D" },
  { "payload_inertia", "VECTOR6D" },
  { "script_control_line", "UINT32" },
  { "time_scale_source", "INT32" },
  { "target_gravity", "VECTOR3D" },
  { "target_base_acceleration", "VECTOR6D" },
  { "control_step", "UINT64" },
  { "target_base_wrench", "VECTOR6D" },

  // NOT IN OFFICIAL DOCS
  { "tool_digital_output_mask", "UINT8" },
  { "tool_digital_output", "UINT8" },
};
// clang-format on

// Mimics a robot's answer to a recipe setup request: the data type of every requested field, or
// "NOT_FOUND" for fields the robot doesn't know.
std::vector<std::string> variableTypesFor(const std::vector<std::string>& recipe)
{
  std::vector<std::string> types;
  types.reserve(recipe.size());
  for (const auto& name : recipe)
  {
    const auto it = g_variable_types.find(name);
    types.push_back(it == g_variable_types.end() ? "NOT_FOUND" : it->second);
  }
  return types;
}

std::string joinStrings(const std::vector<std::string>& strings, const std::string& delimiter = ",")
{
  std::string result;
  for (const auto& string : strings)
  {
    if (!result.empty())
    {
      result += delimiter;
    }
    result += string;
  }
  return result;
}

bool allVariablesFound(const std::vector<std::string>& types)
{
  return std::find(types.begin(), types.end(), "NOT_FOUND") == types.end();
}

// Unlike a client, the server side knows the data types up front, so it applies them itself right
// after allocating the package.
std::unique_ptr<rtde_interface::DataPackage> makeTypedDataPackage(const std::vector<std::string>& recipe,
                                                                  const std::vector<std::string>& types)
{
  auto package = std::make_unique<test::TestableDataPackage>(recipe);
  package->initEmpty(types);
  return package;
}
}  // namespace

RTDEServer::RTDEServer(const int port) : server_(port)
{
  start_time_ = std::chrono::steady_clock::now();
  server_.setMessageCallback(std::bind(&RTDEServer::messageCallback, this, std::placeholders::_1, std::placeholders::_2,
                                       std::placeholders::_3));
  server_.setConnectCallback(std::bind(&RTDEServer::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&RTDEServer::disconnectionCallback, this, std::placeholders::_1));
  server_.setMaxClientsAllowed(1);
  server_.start();
}

RTDEServer::~RTDEServer()
{
  stopSendingDataPackages();
}

void RTDEServer::queueTextMessageBeforeVersionReply(const std::string& message)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  pending_text_messages_.push_back(message);
}

void RTDEServer::setHighestAcceptedProtocolVersion(const uint16_t highest_accepted)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  highest_accepted_protocol_version_ = highest_accepted;
}

std::vector<uint16_t> RTDEServer::requestedProtocolVersions()
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  return requested_protocol_versions_;
}

void RTDEServer::setAcceptStart(const bool accept)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  accept_start_ = accept;
}

void RTDEServer::setAcceptPause(const bool accept)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  accept_pause_ = accept;
}

void RTDEServer::queueTextMessageBeforeSetupOutputs(const std::string& message)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  pending_setup_outputs_text_messages_.push_back(message);
}

void RTDEServer::queueTextMessageBeforeSetupInputs(const std::string& message)
{
  std::lock_guard<std::mutex> lock(negotiation_mutex_);
  pending_setup_inputs_text_messages_.push_back(message);
}

void RTDEServer::sendTextMessage(const socket_t filedescriptor, const std::string& message)
{
  const std::string source = "fake_rtde_server";
  const uint8_t warning_level = 1;

  comm::PackageSerializer serializer;
  uint8_t send_buffer[4096];
  const size_t payload_size = 2 * sizeof(uint8_t) + message.size() + source.size() + sizeof(warning_level);
  size_t send_size = rtde_interface::PackageHeader::serializeHeader(
      send_buffer, rtde_interface::PackageType::RTDE_TEXT_MESSAGE, static_cast<uint16_t>(payload_size));
  send_size += serializer.serialize(send_buffer + send_size, static_cast<uint8_t>(message.size()));
  send_size += serializer.serialize(send_buffer + send_size, message);
  send_size += serializer.serialize(send_buffer + send_size, static_cast<uint8_t>(source.size()));
  send_size += serializer.serialize(send_buffer + send_size, source);
  send_size += serializer.serialize(send_buffer + send_size, warning_level);

  size_t written = 0;
  server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
}

void RTDEServer::connectionCallback(const socket_t filedescriptor)
{
  client_socket_ = filedescriptor;
  URCL_LOG_INFO("Client connected to RTDE server on FD %d", filedescriptor);
}
void RTDEServer::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_INFO("Client disconnected from RTDE server on FD %d", filedescriptor);
  stopSendingDataPackages();
}
void RTDEServer::messageCallback([[maybe_unused]] const socket_t filedescriptor, char* buffer, int nbytesrecv)
{
  comm::BinParser bp(reinterpret_cast<uint8_t*>(buffer), nbytesrecv);
  rtde_interface::PackageHeader::_package_size_type size;
  rtde_interface::PackageType type;
  bp.parse(size);
  bp.parse(type);

  switch (type)
  {
    case rtde_interface::PackageType::RTDE_REQUEST_PROTOCOL_VERSION:
    {
      uint16_t requested_version = 0;
      bp.parse(requested_version);
      bool accepted;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        requested_protocol_versions_.push_back(requested_version);
        accepted = requested_version <= highest_accepted_protocol_version_;
      }
      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_REQUEST_PROTOCOL_VERSION, sizeof(uint8_t));
      send_size += serializer.serialize(send_buffer + send_size, accepted);

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
      break;
    }
    case rtde_interface::PackageType::RTDE_GET_URCONTROL_VERSION:
    {
      // The client only asks once, so every queued message has to go out now for it to see them all
      std::deque<std::string> text_messages;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        text_messages.swap(pending_text_messages_);
      }
      for (const std::string& text_message : text_messages)
      {
        sendTextMessage(filedescriptor, text_message);
      }

      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_GET_URCONTROL_VERSION, 4 * sizeof(uint32_t));
      uint32_t version = 10;
      send_size += serializer.serialize(send_buffer + send_size, version);  // major
      send_size += serializer.serialize(send_buffer + send_size, version);  // minor
      send_size += serializer.serialize(send_buffer + send_size, version);  // bugfix
      send_size += serializer.serialize(send_buffer + send_size, version);  // build

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      std::string unexpected_message;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        if (!pending_setup_outputs_text_messages_.empty())
        {
          unexpected_message = pending_setup_outputs_text_messages_.front();
          pending_setup_outputs_text_messages_.pop_front();
        }
      }
      if (!unexpected_message.empty())
      {
        sendTextMessage(filedescriptor, unexpected_message);
        break;
      }

      bp.parse(output_frequency_);
      URCL_LOG_DEBUG("Frequency is set to %f", output_frequency_);
      std::string variable_names_str;
      bp.parseRemainder(variable_names_str);
      output_recipe_ = splitString(variable_names_str);
      const std::vector<std::string> variable_types = variableTypesFor(output_recipe_);
      const std::string variable_types_str = joinStrings(variable_types);

      {
        std::lock_guard<std::mutex> data_lock(output_data_mutex_);
        output_data_package_.reset();
        if (allVariablesFound(variable_types))
        {
          output_data_package_ = makeTypedDataPackage(output_recipe_, variable_types);
        }
      }

      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
          static_cast<uint16_t>(variable_types_str.length() + sizeof(uint8_t)));
      uint8_t recipe_id = 1;
      send_size += serializer.serialize(send_buffer + send_size, recipe_id);
      send_size += serializer.serialize(send_buffer + send_size, variable_types_str);

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
      URCL_LOG_INFO("Output recipe set");
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
    {
      // The client sends the input recipe once, so every queued unexpected reply has to go out now.
      std::deque<std::string> unexpected_messages;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        unexpected_messages.swap(pending_setup_inputs_text_messages_);
      }
      if (!unexpected_messages.empty())
      {
        for (const std::string& unexpected_message : unexpected_messages)
        {
          sendTextMessage(filedescriptor, unexpected_message);
        }
        break;
      }

      std::string variable_names_str;
      bp.parseRemainder(variable_names_str);
      input_recipe_ = splitString(variable_names_str);
      const std::vector<std::string> variable_types = variableTypesFor(input_recipe_);
      const std::string variable_types_str = joinStrings(variable_types);

      input_data_package_.reset();
      if (allVariablesFound(variable_types))
      {
        input_data_package_ = makeTypedDataPackage(input_recipe_, variable_types);
      }

      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS,
          static_cast<uint16_t>(variable_types_str.length() + sizeof(uint8_t)));
      uint8_t recipe_id = 1;
      send_size += serializer.serialize(send_buffer + send_size, recipe_id);
      send_size += serializer.serialize(send_buffer + send_size, variable_types_str);

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);

      URCL_LOG_INFO("Input recipe set with %zu variables.", input_recipe_.size());
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_START:
    {
      bool accepted;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        accepted = accept_start_;
      }
      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_START, sizeof(uint8_t));
      send_size += serializer.serialize(send_buffer + send_size, accepted);

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
      if (accepted)
      {
        startSendingDataPackages();
      }
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_PAUSE:
    {
      bool accepted;
      {
        std::lock_guard<std::mutex> lock(negotiation_mutex_);
        accepted = accept_pause_;
      }
      comm::PackageSerializer serializer;
      uint8_t send_buffer[4096];
      size_t send_size = 0;
      send_size += rtde_interface::PackageHeader::serializeHeader(
          send_buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_PAUSE, sizeof(uint8_t));
      send_size += serializer.serialize(send_buffer + send_size, accepted);

      size_t written = 0;
      server_.writeUnchecked(filedescriptor, send_buffer, send_size, written);
      if (accepted)
      {
        stopSendingDataPackages();
      }
      break;
    }
    case rtde_interface::PackageType::RTDE_DATA_PACKAGE:
    {
      if (input_data_package_ == nullptr)
      {
        throw std::runtime_error("Fake RTDE Server received a data package before input recipe was setup. This should "
                                 "not happen.");
      }
      input_data_package_->parseWith(bp);
      actOnInput();
      break;
    }
    case rtde_interface::PackageType::RTDE_TEXT_MESSAGE:
    {
      URCL_LOG_WARN("Received Text message which usually shouldn't be sent to the RTDE server.");
      break;
    }
    default:
    {
      URCL_LOG_WARN("Received unknown package type %d", static_cast<int>(type));
      break;
    }
  }
}

void RTDEServer::startSendingDataPackages()
{
  URCL_LOG_INFO("Start sending data.");
  send_loop_running_ = true;
  send_thread_ = std::thread(&RTDEServer::sendDataLoop, this);
}

void RTDEServer::stopSendingDataPackages()
{
  std::lock_guard<std::mutex> thread_lock(thread_control_mutex_);
  send_loop_running_ = false;
  if (send_thread_.joinable())
  {
    URCL_LOG_INFO("Stop sending data.");
    send_thread_.join();
  }
}

void RTDEServer::sendDataLoop()
{
  while (send_loop_running_)
  {
    if (output_data_package_ != nullptr)
    {
      std::lock_guard<std::mutex> data_lock(output_data_mutex_);
      double timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
      output_data_package_->setData("timestamp", timestamp);
      uint8_t buffer[65536];
      size_t size = output_data_package_->serializePackage(buffer);
      size_t written = 0;
      server_.write(client_socket_, buffer, size, written);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / output_frequency_));
  }
}

void RTDEServer::actOnInput()
{
  // This is not complete!
  if (std::find(input_recipe_.begin(), input_recipe_.end(), "speed_slider_mask") != input_recipe_.end())
  {
    double speed_slider_fraction = 0.0;
    input_data_package_->getData("speed_slider_fraction", speed_slider_fraction);
    std::lock_guard<std::mutex> data_lock(output_data_mutex_);
    if (output_data_package_ != nullptr)
    {
      output_data_package_->setData("target_speed_fraction", speed_slider_fraction);
    }
  }
}

void RTDEServer::setStartTime(const std::chrono::steady_clock::time_point& start_time)
{
  start_time_ = start_time;
}

}  // namespace urcl
