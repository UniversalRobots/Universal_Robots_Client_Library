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
#include "ur_client_library/exceptions.h"
#include "ur_client_library/primary/primary_parser.h"
#include <memory>
#include <sstream>

#include <ur_client_library/ur/calibration_checker.h>

namespace urcl
{
static const std::string BEGIN_REPLACE("{{BEGIN_REPLACE}}");
static const std::string JOINT_STATE_REPLACE("{{JOINT_STATE_REPLACE}}");
static const std::string TIME_REPLACE("{{TIME_REPLACE}}");
static const std::string SERVO_J_REPLACE("{{SERVO_J_REPLACE}}");
static const std::string SERVER_IP_REPLACE("{{SERVER_IP_REPLACE}}");
static const std::string SERVER_PORT_REPLACE("{{SERVER_PORT_REPLACE}}");
static const std::string TRAJECTORY_PORT_REPLACE("{{TRAJECTORY_SERVER_PORT_REPLACE}}");
static const std::string SCRIPT_COMMAND_PORT_REPLACE("{{SCRIPT_COMMAND_SERVER_PORT_REPLACE}}");
static const std::string FORCE_MODE_SET_DAMPING_REPLACE("{{FORCE_MODE_SET_DAMPING_REPLACE}}");
static const std::string FORCE_MODE_SET_GAIN_SCALING_REPLACE("{{FORCE_MODE_SET_GAIN_SCALING_REPLACE}}");

urcl::UrDriver::UrDriver(const std::string& robot_ip, const std::string& script_file,
                         const std::string& output_recipe_file, const std::string& input_recipe_file,
                         std::function<void(bool)> handle_program_state, bool headless_mode,
                         std::unique_ptr<ToolCommSetup> tool_comm_setup, const uint32_t reverse_port,
                         const uint32_t script_sender_port, int servoj_gain, double servoj_lookahead_time,
                         bool non_blocking_read, const std::string& reverse_ip, const uint32_t trajectory_port,
                         const uint32_t script_command_port, double force_mode_damping, double force_mode_gain_scaling)
  : servoj_time_(0.008)
  , servoj_gain_(servoj_gain)
  , servoj_lookahead_time_(servoj_lookahead_time)
  , handle_program_state_(handle_program_state)
  , robot_ip_(robot_ip)
{
  URCL_LOG_DEBUG("Initializing urdriver");
  URCL_LOG_DEBUG("Initializing RTDE client");
  rtde_client_.reset(new rtde_interface::RTDEClient(robot_ip_, notifier_, output_recipe_file, input_recipe_file));

  primary_stream_.reset(
      new comm::URStream<primary_interface::PrimaryPackage>(robot_ip_, urcl::primary_interface::UR_PRIMARY_PORT));
  secondary_stream_.reset(
      new comm::URStream<primary_interface::PrimaryPackage>(robot_ip_, urcl::primary_interface::UR_SECONDARY_PORT));
  secondary_stream_->connect();

  non_blocking_read_ = non_blocking_read;
  get_packet_timeout_ = non_blocking_read_ ? 0 : 100;

  if (!rtde_client_->init())
  {
    throw UrException("Initialization of RTDE client went wrong.");
  }

  rtde_frequency_ = rtde_client_->getMaxFrequency();
  servoj_time_ = 1.0 / rtde_frequency_;

  // Figure out the ip automatically if the user didn't provide it
  std::string local_ip = reverse_ip.empty() ? rtde_client_->getIP() : reverse_ip;

  std::string prog = readScriptFile(script_file);
  while (prog.find(JOINT_STATE_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(JOINT_STATE_REPLACE), JOINT_STATE_REPLACE.length(),
                 std::to_string(control::ReverseInterface::MULT_JOINTSTATE));
  }
  while (prog.find(TIME_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(TIME_REPLACE), TIME_REPLACE.length(),
                 std::to_string(control::TrajectoryPointInterface::MULT_TIME));
  }

  std::ostringstream out;
  out << "lookahead_time=" << servoj_lookahead_time_ << ", gain=" << servoj_gain_;
  while (prog.find(SERVO_J_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(SERVO_J_REPLACE), SERVO_J_REPLACE.length(), out.str());
  }

  while (prog.find(SERVER_IP_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(SERVER_IP_REPLACE), SERVER_IP_REPLACE.length(), local_ip);
  }

  while (prog.find(SERVER_PORT_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(SERVER_PORT_REPLACE), SERVER_PORT_REPLACE.length(), std::to_string(reverse_port));
  }

  while (prog.find(TRAJECTORY_PORT_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(TRAJECTORY_PORT_REPLACE), TRAJECTORY_PORT_REPLACE.length(), std::to_string(trajectory_port));
  }

  while (prog.find(SCRIPT_COMMAND_PORT_REPLACE) != std::string::npos)
  {
    prog.replace(prog.find(SCRIPT_COMMAND_PORT_REPLACE), SCRIPT_COMMAND_PORT_REPLACE.length(),
                 std::to_string(script_command_port));
  }

  while (prog.find(FORCE_MODE_SET_DAMPING_REPLACE) != std::string::npos)
  {
    if (force_mode_damping < 0 || force_mode_damping > 1)
    {
      std::stringstream ss;
      ss << "Force mode damping, should be between 0 and 1, but it is " << force_mode_damping
         << " setting it to default 0.5";
      URCL_LOG_ERROR(ss.str().c_str());
      force_mode_damping = 0.025;
    }
    prog.replace(prog.find(FORCE_MODE_SET_DAMPING_REPLACE), FORCE_MODE_SET_DAMPING_REPLACE.length(),
                 std::to_string(force_mode_damping));
  }

  while (prog.find(FORCE_MODE_SET_GAIN_SCALING_REPLACE) != std::string::npos)
  {
    if (robot_version_.major < 5)
    {
      // force_mode_set_gain_scaling is only available for e-series and is therefore removed, if the robot is not
      // e-series
      std::stringstream ss;
      ss << "force_mode_set_gain_scaling(" << FORCE_MODE_SET_GAIN_SCALING_REPLACE << ")";
      prog.replace(prog.find(ss.str()), ss.str().length(), "");
    }
    else
    {
      if (force_mode_gain_scaling < 0 || force_mode_gain_scaling > 2)
      {
        std::stringstream ss;
        ss << "Force mode gain scaling, should be between 0 and 2, but it is " << force_mode_gain_scaling
           << " setting it to default 0.5";
        URCL_LOG_ERROR(ss.str().c_str());
        force_mode_gain_scaling = 0.5;
      }
      prog.replace(prog.find(FORCE_MODE_SET_GAIN_SCALING_REPLACE), FORCE_MODE_SET_GAIN_SCALING_REPLACE.length(),
                   std::to_string(force_mode_gain_scaling));
    }
  }

  robot_version_ = rtde_client_->getVersion();

  std::stringstream begin_replace;
  if (tool_comm_setup != nullptr)
  {
    if (robot_version_.major < 5)
    {
      throw ToolCommNotAvailable("Tool communication setup requested, but this robot version does not support using "
                                 "the tool communication interface. Please check your configuration.",
                                 5, robot_version_.major);
    }
    begin_replace << "set_tool_voltage("
                  << static_cast<std::underlying_type<ToolVoltage>::type>(tool_comm_setup->getToolVoltage()) << ")\n";
    begin_replace << "set_tool_communication("
                  << "True"
                  << ", " << tool_comm_setup->getBaudRate() << ", "
                  << static_cast<std::underlying_type<Parity>::type>(tool_comm_setup->getParity()) << ", "
                  << tool_comm_setup->getStopBits() << ", " << tool_comm_setup->getRxIdleChars() << ", "
                  << tool_comm_setup->getTxIdleChars() << ")";
  }
  prog.replace(prog.find(BEGIN_REPLACE), BEGIN_REPLACE.length(), begin_replace.str());

  in_headless_mode_ = headless_mode;
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
    script_sender_.reset(new control::ScriptSender(script_sender_port, prog));
    URCL_LOG_DEBUG("Created script sender");
  }

  reverse_interface_.reset(new control::ReverseInterface(reverse_port, handle_program_state));
  trajectory_interface_.reset(new control::TrajectoryPointInterface(trajectory_port));
  script_command_interface_.reset(new control::ScriptCommandInterface(script_command_port));

  URCL_LOG_DEBUG("Initialization done");
}

urcl::UrDriver::UrDriver(const std::string& robot_ip, const std::string& script_file,
                         const std::string& output_recipe_file, const std::string& input_recipe_file,
                         std::function<void(bool)> handle_program_state, bool headless_mode,
                         std::unique_ptr<ToolCommSetup> tool_comm_setup, const std::string& calibration_checksum,
                         const uint32_t reverse_port, const uint32_t script_sender_port, int servoj_gain,
                         double servoj_lookahead_time, bool non_blocking_read, const std::string& reverse_ip,
                         const uint32_t trajectory_port, const uint32_t script_command_port, double force_mode_damping,
                         double force_mode_gain_scaling)
  : UrDriver(robot_ip, script_file, output_recipe_file, input_recipe_file, handle_program_state, headless_mode,
             std::move(tool_comm_setup), reverse_port, script_sender_port, servoj_gain, servoj_lookahead_time,
             non_blocking_read, reverse_ip, trajectory_port, script_command_port, force_mode_damping,
             force_mode_gain_scaling)
{
  URCL_LOG_WARN("DEPRECATION NOTICE: Passing the calibration_checksum to the UrDriver's constructor has been "
                "deprecated. Instead, use the checkCalibration(calibration_checksum) function separately. This "
                "notice is for application developers using this library. If you are only using an application using "
                "this library, you can ignore this message.");
  if (checkCalibration(calibration_checksum))
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

std::unique_ptr<rtde_interface::DataPackage> urcl::UrDriver::getDataPackage()
{
  // This can take one of two values, 0ms or 100ms. The large timeout is for when the robot is commanding the control
  // loop's timing (read is blocking). The zero timeout is for when the robot is sharing a control loop with
  // something else (combined_robot_hw)
  std::chrono::milliseconds timeout(get_packet_timeout_);

  return rtde_client_->getDataPackage(timeout);
}

bool UrDriver::writeJointCommand(const vector6d_t& values, const comm::ControlMode control_mode)
{
  return reverse_interface_->write(&values, control_mode);
}

bool UrDriver::writeTrajectoryPoint(const vector6d_t& values, const bool cartesian, const float goal_time,
                                    const float blend_radius)
{
  return trajectory_interface_->writeTrajectoryPoint(&values, goal_time, blend_radius, cartesian);
}

bool UrDriver::writeTrajectoryControlMessage(const control::TrajectoryControlMessage trajectory_action,
                                             const int point_number)
{
  return reverse_interface_->writeTrajectoryControlMessage(trajectory_action, point_number);
}

bool UrDriver::writeFreedriveControlMessage(const control::FreedriveControlMessage freedrive_action)
{
  return reverse_interface_->writeFreedriveControlMessage(freedrive_action);
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

bool UrDriver::startForceMode(const vector6d_t& task_frame, const vector6uint32_t& selection_vector,
                              const vector6d_t& wrench, const unsigned int type, const vector6d_t& limits)
{
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
      return false;
  }
  for (unsigned int i = 0; i < selection_vector.size(); ++i)
  {
    if (selection_vector[i] > 1)
    {
      URCL_LOG_ERROR("The selection vector should only consist of 0's and 1's");
      return false;
    }
  }

  if (script_command_interface_->clientConnected())
  {
    return script_command_interface_->startForceMode(&task_frame, &selection_vector, &wrench, type, &limits);
  }
  else
  {
    URCL_LOG_ERROR("Script command interface is not running. Unable to start Force mode.");
    return false;
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

bool UrDriver::writeKeepalive()
{
  vector6d_t* fake = nullptr;
  return reverse_interface_->write(fake, comm::ControlMode::MODE_IDLE);
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
  std::ifstream ifs(filename);
  std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

  return content;
}

bool UrDriver::checkCalibration(const std::string& checksum)
{
  if (primary_stream_ == nullptr)
  {
    throw std::runtime_error("checkCalibration() called without a primary interface connection being established.");
  }
  primary_interface::PrimaryParser parser;
  comm::URProducer<primary_interface::PrimaryPackage> prod(*primary_stream_, parser);
  prod.setupProducer();

  CalibrationChecker consumer(checksum);

  comm::INotifier notifier;

  comm::Pipeline<primary_interface::PrimaryPackage> pipeline(prod, &consumer, "Pipeline", notifier);
  pipeline.run();

  while (!consumer.isChecked())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  URCL_LOG_DEBUG("Got calibration information from robot.");
  return consumer.checkSuccessful();
}

rtde_interface::RTDEWriter& UrDriver::getRTDEWriter()
{
  return rtde_client_->getWriter();
}

bool UrDriver::sendScript(const std::string& program)
{
  if (secondary_stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. "
                             "This "
                             "should not happen.");
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = program + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (secondary_stream_->write(data, len, written))
  {
    URCL_LOG_DEBUG("Sent program to robot:\n%s", program_with_newline.c_str());
    return true;
  }
  URCL_LOG_ERROR("Could not send program to robot");
  return false;
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

void UrDriver::setKeepaliveCount(const uint32_t& count)
{
  reverse_interface_->setKeepaliveCount(count);
}
}  // namespace urcl
