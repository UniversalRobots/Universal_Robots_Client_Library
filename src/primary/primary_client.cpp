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

#include <ur_client_library/primary/primary_client.h>

namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip) : robot_ip_(robot_ip)
{
  stream_.reset(new comm::URStream<PrimaryPackage>(robot_ip_, UR_PRIMARY_PORT));
  producer_.reset(new comm::URProducer<PrimaryPackage>(*stream_, parser_));
  producer_->setupProducer();

  // Configure consumer
  consumer_.reset(new PrimaryConsumer());
  consumer_->setKeyMessageCallback(std::bind(&PrimaryClient::keyMessageCallback, this, std::placeholders::_1));
  consumer_->setErrorCodeMessageCallback(std::bind(&PrimaryClient::errorMessageCallback, this, std::placeholders::_1));
  consumer_->setRuntimeExceptionMessageCallback(
      std::bind(&PrimaryClient::runtimeExceptionMessageCallback, this, std::placeholders::_1));

  // Configure multi consumer even though we only have one consumer as default, as this enables the user to add more
  // consumers after the object has been created
  std::vector<std::shared_ptr<comm::IConsumer<PrimaryPackage>>> consumers;
  consumers.push_back(consumer_);
  multi_consumer_.reset(new comm::MultiConsumer<PrimaryPackage>(consumers));

  pipeline_.reset(new comm::Pipeline<PrimaryPackage>(*producer_, multi_consumer_.get(), "primary pipeline", notifier_));
  pipeline_->run();
}

bool PrimaryClient::checkCalibration(const std::string& checksum) const
{
  return consumer_->checkCalibration(checksum);
}

bool PrimaryClient::sendScript(const std::string& script_code, const std::chrono::milliseconds timeout)
{
  if (stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
  if (robot_mode_data->robot_mode_ != toUnderlying(RobotMode::ROBOT_MODE_RUNNING))
  {
    URCL_LOG_ERROR("The robot should be running in order to send script commands to the robot");
    return false;
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = script_code + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (!stream_->write(data, len, written))
  {
    URCL_LOG_ERROR("Could not send program to robot");
    return false;
  }
  URCL_LOG_INFO("Sent program to robot:\n%s", program_with_newline.c_str());
  return waitForRobotFeedback(std::chrono::milliseconds(timeout));
}

bool PrimaryClient::sendSecondaryScript(const std::string& script_code, const std::chrono::milliseconds timeout)
{
  if (stream_ == nullptr)
  {
    throw std::runtime_error("Sending script to robot requested while there is no primary interface established. This "
                             "should not happen.");
  }

  std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
  if (robot_mode_data->robot_mode_ != toUnderlying(RobotMode::ROBOT_MODE_RUNNING))
  {
    URCL_LOG_ERROR("The robot should be running in order to send script commands to the robot");
    return false;
  }

  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  auto program_with_newline = script_code + '\n';

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  if (!stream_->write(data, len, written))
  {
    URCL_LOG_ERROR("Could not send program to robot");
    return false;
  }
  URCL_LOG_INFO("Sent secondary program to robot:\n%s", program_with_newline.c_str());
  return waitForRobotErrorFeedback(std::chrono::milliseconds(timeout));
}

void PrimaryClient::reconnect() const
{
  pipeline_->stop();

  if (stream_->getState() == comm::SocketState::Connected)
  {
    stream_->disconnect();
  }

  producer_->setupProducer();
  pipeline_->run();
}

void PrimaryClient::addPrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer)
{
  multi_consumer_->addConsumer(primary_consumer);
}

void PrimaryClient::removePrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer)
{
  multi_consumer_->removeConsumer(primary_consumer);
}

void PrimaryClient::keyMessageCallback(KeyMessage& msg)
{
  std::lock_guard<std::mutex> lk(robot_feedback_mutex_);
  robot_feedback_type_ = RobotFeedbackType::KeyMessage;
  key_message_.reset(new KeyMessage(msg));
  robot_feedback_cv_.notify_one();
}

void PrimaryClient::errorMessageCallback(ErrorCodeMessage& msg)
{
  std::lock_guard<std::mutex> lk(robot_feedback_mutex_);
  error_code_message_.reset(new ErrorCodeMessage(msg));
  robot_feedback_type_ = RobotFeedbackType::ErrorMessage;
  robot_feedback_cv_.notify_one();
}

void PrimaryClient::runtimeExceptionMessageCallback(RuntimeExceptionMessage& msg)
{
  std::lock_guard<std::mutex> lk(robot_feedback_mutex_);
  robot_feedback_type_ = RobotFeedbackType::RuntimeException;
  robot_feedback_cv_.notify_one();
}

bool PrimaryClient::waitForRobotFeedback(const std::chrono::milliseconds timeout)
{
  std::chrono::duration<double> time_left(timeout.count() / 1000.0);
  std::chrono::time_point start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout)
  {
    std::unique_lock<std::mutex> lk(robot_feedback_mutex_);
    if (robot_feedback_cv_.wait_for(lk, time_left) == std::cv_status::no_timeout)
    {
      switch (robot_feedback_type_)
      {
        case RobotFeedbackType::RuntimeException:
          URCL_LOG_ERROR("Above runtime exception message was received from the robot, after executing script.");
          return false;

        case RobotFeedbackType::ErrorMessage:
          // Robot is not in remote control
          if (error_code_message_->message_code_ == 210)
          {
            URCL_LOG_ERROR("The robot is in local control and it is therefore not possible to send script commands to "
                           "the robot.\nIf the robot is in remote control, you have connected to the robot while "
                           "it was in local control. In this case it is necessary to reconnect to the primary "
                           "interface, for that you can use the function reconnect().");
          }
          else
          {
            URCL_LOG_ERROR("Above error code message was received from the robot, after executing script.");
          }
          return false;

        case RobotFeedbackType::KeyMessage:
          if (key_message_->title_ == "PROGRAM_XXX_STARTED")
          {
            return true;
          }
          break;
        default:
          // This shouldn't happen, but in case it does we just continue the loop
          break;
      }
    }
    time_left = timeout - (std::chrono::steady_clock::now() - start);
  }
  URCL_LOG_ERROR("Timed out waiting for feedback from the robot, are you still connected to the robot?");
  return false;
}

bool PrimaryClient::waitForRobotErrorFeedback(const std::chrono::milliseconds timeout)
{
  std::chrono::duration<double> time_left(timeout.count() / 1000.0);
  std::chrono::time_point start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < timeout)
  {
    std::unique_lock<std::mutex> lk(robot_feedback_mutex_);
    if (robot_feedback_cv_.wait_for(lk, time_left) == std::cv_status::no_timeout)
    {
      switch (robot_feedback_type_)
      {
        case RobotFeedbackType::RuntimeException:
          URCL_LOG_ERROR("Above runtime exception message was received from the robot, after executing script.");
          return false;

        case RobotFeedbackType::ErrorMessage:
          // Robot is not in remote control
          if (error_code_message_->message_code_ == 210)
          {
            URCL_LOG_ERROR("The robot is in local control and it is therefore not possible to send script commands to "
                           "the robot.\nIf the robot is in remote control, you have connected to the robot while "
                           "it was in local control. In this case it is necessary to reconnect to the primary "
                           "interface, for that you can use the function reconnect().");
          }
          else
          {
            URCL_LOG_ERROR("Above error code message was received from the robot, after executing script.");
          }
          return false;

        case RobotFeedbackType::KeyMessage:
          break;
        default:
          // This shouldn't happen, but in case it does we just continue the loop
          break;
      }
    }
    time_left = timeout - (std::chrono::steady_clock::now() - start);
  }
  return true;
}

}  // namespace primary_interface
}  // namespace urcl