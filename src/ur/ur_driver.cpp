// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
//
// Many parts from this (Most of the URScript program) comes from the ur_modern_driver
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/ur/ur_driver.h"
#include "ur_client_library/control/script_reader.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/helpers.h"
#include "ur_client_library/primary/primary_parser.h"
#include "ur_client_library/helpers.h"
#include <memory>
#include <sstream>

#include <ur_client_library/ur/calibration_checker.h>

namespace urcl
{
static const std::string BEGIN_REPLACE("BEGIN_REPLACE");
static const std::string JOINT_STATE_REPLACE("JOINT_STATE_REPLACE");
static const std::string TIME_REPLACE("TIME_REPLACE");
static const std::string SERVO_J_REPLACE("SERVO_J_REPLACE");
static const std::string SERVER_IP_REPLACE("SERVER_IP_REPLACE");
static const std::string SERVER_PORT_REPLACE("SERVER_PORT_REPLACE");
static const std::string TRAJECTORY_PORT_REPLACE("TRAJECTORY_SERVER_PORT_REPLACE");
static const std::string SCRIPT_COMMAND_PORT_REPLACE("SCRIPT_COMMAND_SERVER_PORT_REPLACE");

UrDriver::~UrDriver()
{
  // This will return false if the external control script is not running.
  stopControl();
}

void UrDriver::init(const UrDriverConfiguration& config)
{
  robot_ip_ = config.robot_ip;
  non_blocking_read_ = config.non_blocking_read;
  servoj_gain_ = config.servoj_gain;
  servoj_lookahead_time_ = config.servoj_lookahead_time;
  handle_program_state_ = config.handle_program_state;
  in_headless_mode_ = config.headless_mode;
  socket_connection_attempts_ = config.socket_reconnect_attempts;
  socket_reconnection_timeout_ = config.socket_reconnection_timeout;
  rtde_initialization_attempts_ = config.rtde_initialization_attempts;
  rtde_initialization_timeout_ = config.rtde_initialization_timeout;
  force_mode_gain_scale_factor_ = config.force_mode_gain_scaling;
  force_mode_damping_factor_ = config.force_mode_damping;

  URCL_LOG_DEBUG("Initializing urdriver");
  URCL_LOG_DEBUG("Initializing RTDE client");
  rtde_client_.reset(
      new rtde_interface::RTDEClient(robot_ip_, notifier_, config.output_recipe_file, config.input_recipe_file));

  primary_client_.reset(new urcl::primary_interface::PrimaryClient(robot_ip_, notifier_));

  get_packet_timeout_ = non_blocking_read_ ? 0 : 100;

  initRTDE();
  setupReverseInterface(config.reverse_port);

  // Figure out the ip automatically if the user didn't provide it
  std::string local_ip = config.reverse_ip.empty() ? rtde_client_->getIP() : config.reverse_ip;

  trajectory_interface_.reset(new control::TrajectoryPointInterface(config.trajectory_port));
  control::ReverseInterfaceConfig script_command_config;
  script_command_config.port = config.script_command_port;
  script_command_config.robot_software_version = rtde_client_->getVersion();
  script_command_interface_.reset(new control::ScriptCommandInterface(script_command_config));

  startPrimaryClientCommunication();

  std::chrono::milliseconds timeout(1000);
  try
  {
    waitFor([this]() { return primary_client_->getConfigurationData() != nullptr; }, timeout);
  }
  catch (const TimeoutException&)
  {
    throw TimeoutException("Could not get configuration package within timeout, are you connected to the robot?",
                           timeout);
  }

  control::ScriptReader::DataDict data;
  data[JOINT_STATE_REPLACE] = std::to_string(control::ReverseInterface::MULT_JOINTSTATE);
  data[TIME_REPLACE] = std::to_string(control::TrajectoryPointInterface::MULT_TIME);
  std::ostringstream out;
  out << "lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;
  data[SERVO_J_REPLACE] = out.str();
  data[SERVER_IP_REPLACE] = local_ip;
  data[SERVER_PORT_REPLACE] = std::to_string(config.reverse_port);
  data[TRAJECTORY_PORT_REPLACE] = std::to_string(config.trajectory_port);
  data[SCRIPT_COMMAND_PORT_REPLACE] = std::to_string(config.script_command_port);

  robot_version_ = rtde_client_->getVersion();

  std::stringstream begin_replace;
  if (config.tool_comm_setup != nullptr)
  {
    if (robot_version_.major < 5)
    {
      throw ToolCommNotAvailable("Tool communication setup requested, but this robot version does not support using "
                                 "the tool communication interface. Please check your configuration.",
                                 VersionInformation::fromString("5.0.0"), robot_version_);
    }
    begin_replace << "set_tool_voltage("
                  << static_cast<std::underlying_type<ToolVoltage>::type>(config.tool_comm_setup->getToolVoltage())
                  << ")\n";
    begin_replace << "set_tool_communication(" << "True" << ", " << config.tool_comm_setup->getBaudRate() << ", "
                  << static_cast<std::underlying_type<Parity>::type>(config.tool_comm_setup->getParity()) << ", "
                  << config.tool_comm_setup->getStopBits() << ", " << config.tool_comm_setup->getRxIdleChars() << ", "
                  << config.tool_comm_setup->getTxIdleChars() << ")";
  }
  data[BEGIN_REPLACE] = begin_replace.str();

  data["ROBOT_SOFTWARE_VERSION"] = getVersion();

  script_reader_.reset(new control::ScriptReader());
  std::string prog = script_reader_->readScriptFile(config.script_file, data);

  if (in_headless_mode_)
  {
    full_robot_program_ = "stop program\n";
    full_robot_program_ += "def externalControl():\n";
    std::istringstream prog_stream(prog);
    std::string line;
    while (std::getline(prog_stream, line))
    {
      full_robot_program_ += "\t" + line + "\n";
    }
    full_robot_program_ += "end\n";
    sendRobotProgram();
  }
  else
  {
    script_sender_.reset(new control::ScriptSender(config.script_sender_port, prog));
    URCL_LOG_DEBUG("Created script sender");
  }

  if (!std::empty(config.calibration_checksum))
  {
    URCL_LOG_WARN("DEPRECATION NOTICE: Passing the calibration_checksum to the UrDriver's constructor has been "
                  "deprecated. Instead, use the checkCalibration(calibration_checksum) function separately. This "
                  "notice is for application developers using this library. If you are only using an application using "
                  "this library, you can ignore this message.");
    if (checkCalibration(config.calibration_checksum))
    {
      URCL_LOG_INFO("Calibration checked successfully.");
    }
    else
    {
      URCL_LOG_ERROR("The calibration parameters of the connected robot don't match the ones from the given kinematics "
                     "config file. Please be aware that this can lead to critical inaccuracies of tcp positions. Use "
                     "the ur_calibration tool to extract the correct calibration from the robot and pass that into the "
                     "description. See "
                     "[https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information] "
                     "for details.");
    }
  }
  URCL_LOG_DEBUG("Initialization done");
}

std::unique_ptr<rtde_interface::DataPackage> urcl::UrDriver::getDataPackage()
{
  // This can take one of two values, 0ms or 100ms. The large timeout is for when the robot is commanding the control
  // loop's timing (read is blocking). The zero timeout is for when the robot is sharing a control loop with
  // something else (combined_robot_hw)
  std::chrono::milliseconds timeout(get_packet_timeout_);

  return rtde_client_->getDataPackage(timeout);
}

bool UrDriver::writeJointCommand(const vector6d_t& values, const comm::ControlMode control_mode,
                                 const RobotReceiveTimeout& robot_receive_timeout)
{
  return reverse_interface_->write(&values, control_mode, robot_receive_timeout);
}

bool UrDriver::writeTrajectoryPoint(const vector6d_t& positions, const float acceleration, const float velocity,
                                    const bool cartesian, const float goal_time, const float blend_radius)
{
  return trajectory_interface_->writeTrajectoryPoint(&positions, acceleration, velocity, goal_time, blend_radius,
                                                     cartesian);
}

bool UrDriver::writeTrajectoryPoint(const vector6d_t& positions, const bool cartesian, const float goal_time,
                                    const float blend_radius)
{
  return trajectory_interface_->writeTrajectoryPoint(&positions, goal_time, blend_radius, cartesian);
}

bool UrDriver::writeTrajectorySplinePoint(const vector6d_t& positions, const vector6d_t& velocities,
                                          const vector6d_t& accelerations, const float goal_time)
{
  return trajectory_interface_->writeTrajectorySplinePoint(&positions, &velocities, &accelerations, goal_time);
}

bool UrDriver::writeTrajectorySplinePoint(const vector6d_t& positions, const vector6d_t& velocities,
                                          const float goal_time)
{
  return trajectory_interface_->writeTrajectorySplinePoint(&positions, &velocities, nullptr, goal_time);
}

bool UrDriver::writeTrajectorySplinePoint(const vector6d_t& positions, const float goal_time)
{
  return trajectory_interface_->writeTrajectorySplinePoint(&positions, nullptr, nullptr, goal_time);
}

bool UrDriver::writeTrajectoryControlMessage(const control::TrajectoryControlMessage trajectory_action,
                                             const int point_number, const RobotReceiveTimeout& robot_receive_timeout)
{
  return reverse_interface_->writeTrajectoryControlMessage(trajectory_action, point_number, robot_receive_timeout);
}

bool UrDriver::writeMotionPrimitive(const std::shared_ptr<control::MotionPrimitive> motion_instruction)
{
  return trajectory_interface_->writeMotionPrimitive(motion_instruction);
}

bool UrDriver::writeFreedriveControlMessage(const control::FreedriveControlMessage freedrive_action,
                                            const RobotReceiveTimeout& robot_receive_timeout)
{
  return reverse_interface_->writeFreedriveControlMessage(freedrive_action, robot_receive_timeout);
}

bool UrDriver::zeroFTSensor()
{
  if (getVersion().major < 5)
  {
    std::stringstream ss;
    ss << "Zeroing the Force-Torque sensor is only available for e-Series robots (Major version >= 5). This robot's "
          "version is "
       << getVersion();
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }
  else
  {
    if (script_command_interface_->clientConnected())
    {
      return script_command_interface_->zeroFTSensor();
    }
    else
    {
      URCL_LOG_WARN("Script command interface is not running. Falling back to sending plain script code. This will "
                    "only work, if the robot is in remote_control mode.");
      std::stringstream cmd;
      cmd << "sec tareSetup():" << std::endl << " zero_ftsensor()" << std::endl << "end";
      return sendScript(cmd.str());
    }
  }
}

bool UrDriver::setPayload(const float mass, const vector3d_t& cog)
{
  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->setPayload(mass, &cog);
  }
  else
  {
    URCL_LOG_WARN("Script command interface is not running. Falling back to sending plain script code. On e-Series "
                  "robots this will only work, if the robot is in remote_control mode.");
    std::stringstream cmd;
    cmd.imbue(std::locale::classic());  // Make sure, decimal divider is actually '.'
    cmd << "sec setup():" << std::endl
        << " set_payload(" << mass << ", [" << cog[0] << ", " << cog[1] << ", " << cog[2] << "])" << std::endl
        << "end";
    return sendScript(cmd.str());
  }
}

bool UrDriver::setToolVoltage(const ToolVoltage voltage)
{
  // Test that the tool voltage is either 0, 12 or 24.
  switch (voltage)
  {
    case ToolVoltage::OFF:
      break;
    case ToolVoltage::_12V:
      break;
    case ToolVoltage::_24V:
      break;
    default:
      std::stringstream ss;
      ss << "The tool voltage should be 0, 12 or 24. The tool voltage is " << toUnderlying(voltage);
      URCL_LOG_ERROR(ss.str().c_str());
      return false;
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->setToolVoltage(voltage);
  }
  else
  {
    URCL_LOG_WARN("Script command interface is not running. Falling back to sending plain script code. On e-Series "
                  "robots this will only work, if the robot is in remote_control mode.");
    std::stringstream cmd;
    cmd << "sec setup():" << std::endl << " set_tool_voltage(" << toUnderlying(voltage) << ")" << std::endl << "end";
    return sendScript(cmd.str());
  }
}
// Function for e-series robots (Needs both damping factor and gain scaling factor)
bool UrDriver::startForceMode(const vector6d_t& task_frame, const vector6uint32_t& selection_vector,
                              const vector6d_t& wrench, const unsigned int type, const vector6d_t& limits,
                              double damping_factor, double gain_scaling_factor)
{
  if (robot_version_.major < 5)
  {
    std::stringstream ss;
    ss << "Force mode gain scaling factor cannot be set on a CB3 robot.";
    URCL_LOG_ERROR(ss.str().c_str());
    VersionInformation req_version = VersionInformation::fromString("5.0.0.0");
    throw IncompatibleRobotVersion(ss.str(), req_version, robot_version_);
  }
  // Test that the type is either 1, 2 or 3.
  switch (type)
  {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    default:
      std::stringstream ss;
      ss << "The type should be 1, 2 or 3. The type is " << type;
      URCL_LOG_ERROR(ss.str().c_str());
      throw InvalidRange(ss.str().c_str());
  }
  for (unsigned int i = 0; i < selection_vector.size(); ++i)
  {
    if (selection_vector[i] > 1)
    {
      std::stringstream ss;
      ss << "The selection vector should only consist of 0's and 1's";
      URCL_LOG_ERROR(ss.str().c_str());
      throw InvalidRange(ss.str().c_str());
    }
  }

  if (damping_factor > 1 || damping_factor < 0)
  {
    std::stringstream ss;
    ss << "The force mode damping factor should be between 0 and 1, both inclusive.";
    URCL_LOG_ERROR(ss.str().c_str());
    throw InvalidRange(ss.str().c_str());
  }

  if (gain_scaling_factor > 2 || gain_scaling_factor < 0)
  {
    std::stringstream ss;
    ss << "The force mode gain scaling factor should be between 0 and 2, both inclusive.";
    URCL_LOG_ERROR(ss.str().c_str());
    throw InvalidRange(ss.str().c_str());
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->startForceMode(&task_frame, &selection_vector, &wrench, type, &limits,
                                                     damping_factor, gain_scaling_factor);
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to start Force mode.");
    return false;
  }
}

// Function for CB3 robots (CB3 robots cannot use gain scaling)
bool UrDriver::startForceMode(const vector6d_t& task_frame, const vector6uint32_t& selection_vector,
                              const vector6d_t& wrench, const unsigned int type, const vector6d_t& limits,
                              double damping_factor)
{
  if (robot_version_.major >= 5)
  {
    std::stringstream ss;
    ss << "You should also specify a force mode gain scaling factor to activate force mode on an e-series robot.";
    URCL_LOG_ERROR(ss.str().c_str());
    throw MissingArgument(ss.str(), "startForceMode", "gain_scaling_factor", 0.5);
  }
  // Test that the type is either 1, 2 or 3.
  switch (type)
  {
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    default:
      std::stringstream ss;
      ss << "The type should be 1, 2 or 3. The type is " << type;
      URCL_LOG_ERROR(ss.str().c_str());
      throw InvalidRange(ss.str().c_str());
  }
  for (unsigned int i = 0; i < selection_vector.size(); ++i)
  {
    if (selection_vector[i] > 1)
    {
      std::stringstream ss;
      ss << "The selection vector should only consist of 0's and 1's";
      URCL_LOG_ERROR(ss.str().c_str());
      throw InvalidRange(ss.str().c_str());
    }
  }

  if (damping_factor > 1 || damping_factor < 0)
  {
    std::stringstream ss;
    ss << "The force mode damping factor should be between 0 and 1, both inclusive.";
    URCL_LOG_ERROR(ss.str().c_str());
    throw InvalidRange(ss.str().c_str());
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->startForceMode(&task_frame, &selection_vector, &wrench, type, &limits,
                                                     damping_factor, 0);
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to start Force mode.");
    return false;
  }
}

bool UrDriver::startForceMode(const vector6d_t& task_frame, const vector6uint32_t& selection_vector,
                              const vector6d_t& wrench, const unsigned int type, const vector6d_t& limits)
{
  if (robot_version_.major < 5)
  {
    return startForceMode(task_frame, selection_vector, wrench, type, limits, force_mode_damping_factor_);
  }
  else
  {
    return startForceMode(task_frame, selection_vector, wrench, type, limits, force_mode_damping_factor_,
                          force_mode_gain_scale_factor_);
  }
}

bool UrDriver::endForceMode()
{
  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->endForceMode();
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to end Force mode.");
    return false;
  }
}

bool UrDriver::startToolContact()
{
  if (getVersion().major < 5)
  {
    std::stringstream ss;
    ss << "Tool contact is only available for e-Series robots (Major version >= 5). This robot's "
          "version is "
       << getVersion();
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->startToolContact();
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to enable tool contact mode.");
    return 0;
  }
}

bool UrDriver::endToolContact()
{
  if (getVersion().major < 5)
  {
    std::stringstream ss;
    ss << "Tool contact is only available for e-Series robots (Major version >= 5). This robot's "
          "version is "
       << getVersion();
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->endToolContact();
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to end tool contact mode.");
    return 0;
  }
}

bool UrDriver::setFrictionCompensation(const bool friction_compensation_enabled)
{
  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->setFrictionCompensation(friction_compensation_enabled);
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to set friction compensation.");
    return 0;
  }
}

bool UrDriver::ftRtdeInputEnable(const bool enabled, const double sensor_mass,
                                 const vector3d_t& sensor_measuring_offset, const vector3d_t& sensor_cog)
{
  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->ftRtdeInputEnable(enabled, sensor_mass, sensor_measuring_offset, sensor_cog);
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to set ft_rtde_input_enable.");
    return 0;
  }
}

bool UrDriver::writeKeepalive(const RobotReceiveTimeout& robot_receive_timeout)
{
  vector6d_t* fake = nullptr;
  return reverse_interface_->write(fake, comm::ControlMode::MODE_IDLE, robot_receive_timeout);
}

void UrDriver::startRTDECommunication()
{
  rtde_client_->start();
}

bool UrDriver::stopControl()
{
  vector6d_t* fake = nullptr;
  return reverse_interface_->write(fake, comm::ControlMode::MODE_STOPPED);
}

std::string UrDriver::readScriptFile(const std::string& filename)
{
  std::ifstream ifs;
  ifs.open(filename);
  if (!ifs)
  {
    std::stringstream ss;
    ss << "URScript file '" << filename << "' doesn't exists.";
    throw UrException(ss.str().c_str());
  }
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return content;
}

bool UrDriver::checkCalibration(const std::string& checksum)
{
  return primary_client_->checkCalibration(checksum);
}

rtde_interface::RTDEWriter& UrDriver::getRTDEWriter()
{
  return rtde_client_->getWriter();
}

bool UrDriver::sendScript(const std::string& program)
{
  if (primary_client_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary client initialized. "
                             "This should not happen.");
  }
  return primary_client_->sendScript(program);
}

bool UrDriver::sendRobotProgram()
{
  if (in_headless_mode_)
  {
    return sendScript(full_robot_program_);
  }
  else
  {
    URCL_LOG_ERROR("Tried to send robot program directly while not in headless mode");
    return false;
  }
}

std::vector<std::string> UrDriver::getRTDEOutputRecipe()
{
  return rtde_client_->getOutputRecipe();
}

void UrDriver::setKeepaliveCount(const uint32_t count)
{
  URCL_LOG_WARN("DEPRECATION NOTICE: Setting the keepalive count has been deprecated. Instead use the "
                "RobotReceiveTimeout, to set the timeout directly in the write commands. Please change your code to "
                "set the "
                "read timeout in the write commands directly. This keepalive count will overwrite the timeout passed "
                "to the write functions.");
// TODO: Remove 2027-05
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  reverse_interface_->setKeepaliveCount(count);
#pragma GCC diagnostic pop
}

void UrDriver::resetRTDEClient(const std::vector<std::string>& output_recipe,
                               const std::vector<std::string>& input_recipe, double target_frequency,
                               bool ignore_unavailable_outputs)
{
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip_, notifier_, output_recipe, input_recipe, target_frequency,
                                                    ignore_unavailable_outputs));
  initRTDE();
}

void UrDriver::resetRTDEClient(const std::string& output_recipe_filename, const std::string& input_recipe_filename,
                               double target_frequency, bool ignore_unavailable_outputs)
{
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip_, notifier_, output_recipe_filename, input_recipe_filename,
                                                    target_frequency, ignore_unavailable_outputs));
  initRTDE();
}

void UrDriver::initRTDE()
{
  if (!rtde_client_->init(socket_connection_attempts_, socket_reconnection_timeout_, rtde_initialization_attempts_,
                          rtde_initialization_timeout_))
  {
    throw UrException("Initialization of RTDE client went wrong.");
  }
}

void UrDriver::setupReverseInterface(const uint32_t reverse_port)
{
  auto rtde_frequency = rtde_client_->getTargetFrequency();
  auto step_time = std::chrono::milliseconds(static_cast<int>(1000 / rtde_frequency));
  control::ReverseInterfaceConfig config;
  config.step_time = step_time;
  config.port = reverse_port;
  config.handle_program_state = handle_program_state_;
  config.robot_software_version = robot_version_;
  reverse_interface_.reset(new control::ReverseInterface(config));
}

void UrDriver::startPrimaryClientCommunication()
{
  primary_client_->start(socket_connection_attempts_, socket_reconnection_timeout_);
}

std::deque<urcl::primary_interface::ErrorCode> UrDriver::getErrorCodes()
{
  return primary_client_->getErrorCodes();
}
}  // namespace urcl
