// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
//
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner exner@fzi.de
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED

#include "ur_client_library/primary/abstract_primary_consumer.h"

#include <functional>
#include <mutex>
#include <condition_variable>

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Primary consumer implementation
 *
 * This class implements an AbstractPrimaryConsumer such that it can consume all incoming primary
 * messages. The actual work for each package will be done in this class.
 */
class PrimaryConsumer : public AbstractPrimaryConsumer
{
public:
  PrimaryConsumer()
  {
  }
  virtual ~PrimaryConsumer() = default;

  /*!
   * \brief Consume a RobotMessage
   *
   * \param msg Robot message
   *
   * \returns True
   */
  virtual bool consume(RobotMessage& msg) override
  {
    return true;
  }

  /*!
   * \brief Consume a RobotState
   *
   * \param msg Robot state
   *
   * \returns True
   */
  virtual bool consume(RobotState& msg) override
  {
    return true;
  }

  /*!
   * \brief Handle a VersionMessage
   *
   * \param pkg VersionMessage
   *
   * \returns True
   */
  virtual bool consume(VersionMessage& pkg) override
  {
    return true;
  }

  /*!
   * \brief Handle a KinematicsInfo
   *
   * \param pkg KinematicsInfo
   *
   * \returns True
   */
  virtual bool consume(KinematicsInfo& pkg) override
  {
    return true;
  }

  /*!
   * \brief Handle an ErrorCodeMessage
   *
   * \param pkg ErrorCodeMessage
   *
   * \returns True
   */
  virtual bool consume(ErrorCodeMessage& pkg) override
  {
    urcl::primary_interface::ErrorCode code;
    code.message_code = pkg.message_code_;
    code.message_argument = pkg.message_argument_;
    code.report_level = pkg.report_level_;
    code.data_type = pkg.data_type_;
    code.data = pkg.data_;
    code.text = pkg.text_;
    code.timestamp = pkg.timestamp_;
    code.to_string = pkg.toString();

    const auto log_contents = "Logging an ErrorCodeMessage from the UR Controller Box: " + pkg.toString();

    switch (code.report_level)
    {
      case urcl::primary_interface::ReportLevel::DEBUG:
      case urcl::primary_interface::ReportLevel::DEVL_DEBUG:
      case urcl::primary_interface::ReportLevel::DEVL_INFO:
      case urcl::primary_interface::ReportLevel::DEVL_WARNING:
      case urcl::primary_interface::ReportLevel::DEVL_VIOLATION:
      case urcl::primary_interface::ReportLevel::DEVL_FAULT:
        URCL_LOG_DEBUG(log_contents.c_str());
        break;
      case urcl::primary_interface::ReportLevel::INFO:
        URCL_LOG_INFO(log_contents.c_str());
        break;
      case urcl::primary_interface::ReportLevel::WARNING:
        URCL_LOG_WARN(log_contents.c_str());
        break;
      default:
        // urcl::primary_interface::ReportLevel::VIOLATION:
        // urcl::primary_interface::ReportLevel::FAULT:
        URCL_LOG_ERROR(log_contents.c_str());
        break;
    }

    if (error_code_message_callback_ != nullptr)
    {
      error_code_message_callback_(code);
    }
    return true;
  }

  /*!
   * \brief Set callback function which will be triggered whenever error code messages are received
   *
   * \param callback_function Function handling the event information. The error code message received is passed to the
   * function.
   */
  void setErrorCodeMessageCallback(std::function<void(ErrorCode&)> callback_function)
  {
    error_code_message_callback_ = callback_function;
  }

private:
  std::function<void(ErrorCode&)> error_code_message_callback_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
