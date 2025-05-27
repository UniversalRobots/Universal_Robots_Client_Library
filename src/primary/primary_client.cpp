// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright © 2024-2025 Ocado Group
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

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/primary/robot_message.h>
#include <ur_client_library/primary/robot_state.h>
#include "ur_client_library/exceptions.h"
#include <ur_client_library/helpers.h>
#include <chrono>
namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip, comm::INotifier& notifier)
  : stream_(robot_ip, UR_PRIMARY_PORT)
{
  prod_.reset(new comm::URProducer<PrimaryPackage>(stream_, parser_));

  consumer_.reset(new PrimaryConsumer());
  consumer_->setErrorCodeMessageCallback(std::bind(&PrimaryClient::errorMessageCallback, this, std::placeholders::_1));

  // Configure multi consumer even though we only have one consumer as default, as this enables the user to add more
  // consumers after the object has been created
  std::vector<std::shared_ptr<comm::IConsumer<PrimaryPackage>>> consumers;
  consumers.push_back(consumer_);
  multi_consumer_.reset(new comm::MultiConsumer<PrimaryPackage>(consumers));

  pipeline_.reset(
      new comm::Pipeline<PrimaryPackage>(*prod_, multi_consumer_.get(), "PrimaryClient Pipeline", notifier_));
}

PrimaryClient::~PrimaryClient()
{
  URCL_LOG_INFO("Stopping primary client pipeline");
  pipeline_->stop();
}

void PrimaryClient::start(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  URCL_LOG_INFO("Starting primary client pipeline");
  pipeline_->init(max_num_tries, reconnection_time);
  pipeline_->run();
}

void PrimaryClient::stop()
{
  pipeline_->stop();
  stream_.close();
}

void PrimaryClient::addPrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->addConsumer(primary_consumer);
}

void PrimaryClient::removePrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->removeConsumer(primary_consumer);
}

void PrimaryClient::errorMessageCallback(ErrorCode& code)
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  error_code_queue_.push_back(code);
}

std::deque<ErrorCode> PrimaryClient::getErrorCodes()
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  std::deque<ErrorCode> error_codes;
  error_codes = error_code_queue_;
  error_code_queue_.clear();
  return error_codes;
}

bool PrimaryClient::sendScript(const std::string& program)
{
  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = program + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  const auto send_script_contents = [this, program_with_newline, data, len,
                                     &written](const std::string&& description) -> bool {
    if (stream_.write(data, len, written))
    {
      URCL_LOG_DEBUG("Sent program to robot:\n%s", program_with_newline.c_str());
      return true;
    }
    const std::string error_message = "Could not send program to robot: " + description;
    URCL_LOG_ERROR(error_message.c_str());
    return false;
  };

  if (send_script_contents("initial attempt"))
  {
    return true;
  }

  if (reconnectStream())
  {
    return send_script_contents("after reconnecting primary stream");
  }

  return false;
}

bool PrimaryClient::reconnectStream()
{
  URCL_LOG_DEBUG("Closing primary stream...");
  stream_.close();
  if (stream_.connect())
  {
    URCL_LOG_DEBUG("Primary stream connected");
    return true;
  }
  URCL_LOG_ERROR("Failed to reconnect primary stream!");
  return false;
}

bool PrimaryClient::checkCalibration(const std::string& checksum)
{
  std::shared_ptr<primary_interface::KinematicsInfo> kin_info = consumer_->getKinematicsInfo();
  while (kin_info == nullptr)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    kin_info = consumer_->getKinematicsInfo();
  }
  URCL_LOG_DEBUG("Got calibration information from robot.");

  return kin_info->toHash() == checksum;
}

void PrimaryClient::commandPowerOn(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("power on"))
  {
    throw UrException("Failed to send power on command to robot");
  }

  if (validate)
  {
    try
    {
      waitFor([this]() { return getRobotMode() == RobotMode::IDLE; }, timeout);
    }
    catch (const TimeoutException& ex)
    {
      throw TimeoutException("Robot did not power on within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandPowerOff(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("power off"))
  {
    throw UrException("Failed to send power off command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return getRobotMode() == RobotMode::POWER_OFF; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not power off within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandBrakeRelease(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("set robotmode run"))
  {
    throw UrException("Failed to send brake release command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return getRobotMode() == RobotMode::RUNNING; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not release the brakes within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandUnlockProtectiveStop(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("set unlock protective stop"))
  {
    throw UrException("Failed to send unlock protective stop command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return consumer_->getRobotModeData()->is_protective_stopped_ == false; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not unlock the protective stop within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandStop(const bool validate, const std::chrono::milliseconds timeout)
{
  std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
  if (robot_mode_data == nullptr)
  {
    throw UrException("Stopping a program while robot state is unknown. This should not happen");
  }

  if (!sendScript("stop program"))
  {
    throw UrException("Failed to send the command `stop program` to robot");
  }
  if (validate)
  {
    try
    {
      waitFor(
          [this]() {
            return !consumer_->getRobotModeData()->is_program_running_ &&
                   !consumer_->getRobotModeData()->is_program_paused_;
          },
          timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not stop the program within the given timeout", timeout);
    }
  }
}
std::shared_ptr<VersionInformation> PrimaryClient::getRobotVersion(bool blocking,
                                                                   const std::chrono::milliseconds timeout)
{
  if (blocking)
  {
    waitFor([this]() { return consumer_->getVersionInformation() != nullptr; }, timeout);
  }

  return consumer_->getVersionInformation();
}

RobotType PrimaryClient::getRobotType()
{
  std::shared_ptr<ConfigurationData> configuration_data = consumer_->getConfigurationData();
  if (configuration_data == nullptr)
  {
    return RobotType::UNDEFINED;
  }
  return static_cast<RobotType>(configuration_data->robot_type_);
}

}  // namespace primary_interface
}  // namespace urcl
