// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Text 2.0 (the "License");
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
 * \author  Felix Exner exner@fzi.de
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#include <chrono>
#include <thread>

#include <ur_client_library/primary/primary_client.h>

#include <ur_client_library/comm/tcp_socket.h>

namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip, const std::string& calibration_checksum)
  : robot_ip_(robot_ip), port_(UR_PRIMARY_PORT), simulated_(false)
{
  stream_.reset(new comm::URStream<PrimaryPackage>(robot_ip_, port_));
  producer_.reset(new comm::URProducer<PrimaryPackage>(*stream_, parser_));
  producer_->setupProducer();

  consumer_.reset(new PrimaryConsumer());
  calibration_checker_.reset(new CalibrationChecker(calibration_checksum));
  consumer_->setKinematicsInfoHandler(calibration_checker_);

  pipeline_.reset(new comm::Pipeline<PrimaryPackage>(*producer_, consumer_.get(), "primary pipeline", notifier_));
  pipeline_->run();
  in_remote_control_ = false;
  running_ = true;

  rc_thread_ = std::thread(&PrimaryClient::checkRemoteLocalControl, this);

  std::chrono::duration<double> time_done(0);
  std::chrono::duration<double> timeout = std::chrono::seconds(1);
  std::chrono::duration<double> retry_period = std::chrono::milliseconds(10);
  while (!calibration_checker_->isChecked() || time_done < timeout)
  {
    std::this_thread::sleep_for(retry_period);
    time_done += retry_period;
  }
  if (calibration_checker_->checkSuccessful())
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

PrimaryClient::~PrimaryClient()
{
  URCL_LOG_DEBUG("Destructing primary client");
  stop();
}

bool PrimaryClient::sendScript(const std::string& script_code)
{
  if (stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = script_code + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (!in_remote_control_ && !simulated_)
  {
    URCL_LOG_ERROR("Not in remote control. Cannot send script whilst in local control");
    return false;
  }

  if (stream_->write(data, len, written))
  {
    URCL_LOG_INFO("Sent program to robot:\n%s", program_with_newline.c_str());
    return true;
  }
  URCL_LOG_ERROR("Could not send program to robot");
  return false;
}

bool PrimaryClient::configure()
{
  URCL_LOG_DEBUG("Reconnecting to stream on %s:%d", robot_ip_.c_str(), port_);
  pipeline_->stop();

  if (stream_->getState() == comm::SocketState::Connected)
    stream_->disconnect();

  bool res = stream_->connect();
  pipeline_->run();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return res;
}

void PrimaryClient::checkRemoteLocalControl()
{
  dashboard_client_.reset(new urcl::DashboardClient(robot_ip_));
  dashboard_client_->connect();
  bool old_val = dashboard_client_->sendAndReceive("is in remote control\n") == "true";
  while (running_)
  {
    if (dashboard_client_->getState() == comm::SocketState::Connected)
    {
      std::string answer = dashboard_client_->sendAndReceive("is in remote control\n");
      in_remote_control_ = answer == "true";

      if (old_val != in_remote_control_)
        configure();
    }
    else
    {
      URCL_LOG_INFO("Reconnecting to Dashboard Server");
      dashboard_client_->connect();
    }
    old_val = in_remote_control_;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void PrimaryClient::stop()
{
  running_ = false;
  if (rc_thread_.joinable())
  {
    rc_thread_.join();
  }
  pipeline_->stop();
  stream_->disconnect();
}

bool PrimaryClient::isInRemoteControl()
{
  return in_remote_control_;
}

std::shared_ptr<AdditionalInfo> PrimaryClient::getAdditionalInfo()
{
  return consumer_->additional_info_message_worker_->getData();
}

std::shared_ptr<CartesianInfo> PrimaryClient::getCartesianInfo()
{
  return consumer_->cartesian_info_message_worker_->getData();
}

std::shared_ptr<ForceModeData> PrimaryClient::getForceModeData()
{
  return consumer_->force_mode_data_message_worker_->getData();
}

std::shared_ptr<JointData> PrimaryClient::getJointData()
{
  return consumer_->joint_data_message_worker_->getData();
}

std::shared_ptr<RobotModeData> PrimaryClient::getRobotModeData()
{
  return consumer_->robot_mode_data_message_worker_->getData();
}

std::shared_ptr<KeyMessage> PrimaryClient::getKeyMessage()
{
  return consumer_->key_message_worker_->getData();
}

std::shared_ptr<ErrorCodeMessage> PrimaryClient::getErrorCodeMessage()
{
  return consumer_->error_code_message_worker_->getData();
}

std::shared_ptr<RuntimeExceptionMessage> PrimaryClient::getRuntimeExceptionMessage()
{
  return consumer_->runtime_exception_message_worker_->getData();
}

std::shared_ptr<TextMessage> PrimaryClient::getTextMessage()
{
  return consumer_->text_message_worker_->getData();
}

std::shared_ptr<VersionMessage> PrimaryClient::getVersionMessage()
{
  return consumer_->version_message_worker_->getData();
}

std::shared_ptr<GlobalVariablesSetupMessage> PrimaryClient::getGlobalVariablesSetupMessage()
{
  return consumer_->global_variables_setup_message_worker_->getData();
}

std::shared_ptr<CalibrationChecker> PrimaryClient::getCalibrationChecker()
{
  return calibration_checker_;
}

void PrimaryClient::setSimulated(bool value)
{
  simulated_ = value;
}

bool PrimaryClient::getSimulated()
{
  return simulated_;
}
}  // namespace primary_interface
}  // namespace urcl
