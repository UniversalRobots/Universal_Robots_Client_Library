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
#include "ur_client_library/primary/robot_state/robot_mode_data.h"
#include "ur_client_library/ur/datatypes.h"
#include "ur_client_library/ur/version_information.h"

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
    std::scoped_lock lock(version_information_mutex_);
    version_information_ = std::make_shared<VersionInformation>();
    version_information_->major = pkg.major_version_;
    version_information_->minor = pkg.minor_version_;
    version_information_->bugfix = pkg.svn_version_;
    version_information_->build = pkg.build_number_;
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
    URCL_LOG_DEBUG("%s", pkg.toString().c_str());
    kinematics_info_ = std::make_shared<KinematicsInfo>(pkg);
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

  virtual bool consume(RobotModeData& pkg) override
  {
    URCL_LOG_DEBUG("Robot mode is now %s", robotModeString(static_cast<RobotMode>(pkg.robot_mode_)).c_str());
    std::scoped_lock lock(robot_mode_mutex_);
    robot_mode_ = std::make_shared<RobotModeData>(pkg);
    return true;
  }

  virtual bool consume(ConfigurationData& pkg) override
  {
    std::scoped_lock lock(configuration_data_mutex_);
    configuration_data_ = std::make_shared<ConfigurationData>(pkg);
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

  /*!
   * \brief Get the kinematics info
   *
   * \returns Shared pointer to the kinematics info
   */
  std::shared_ptr<KinematicsInfo> getKinematicsInfo()
  {
    return kinematics_info_;
  }

  /*!
   * \brief Get the latest robot mode.
   *
   * The robot mode will be updated in the background. This will always show the latest received
   * robot mode independent of the time that has passed since receiving it.
   */
  std::shared_ptr<RobotModeData> getRobotModeData()
  {
    std::scoped_lock lock(robot_mode_mutex_);
    return robot_mode_;
  }

  /*!
   * \brief Get the latest version information.
   *
   * The version information will be updated in the background. This will always show the latest
   * received version information independent of the time that has passed since receiving it. If no
   * version information has been received yet, this will return a nullptr.
   */
  std::shared_ptr<VersionInformation> getVersionInformation()
  {
    std::scoped_lock lock(version_information_mutex_);
    return version_information_;
  }

  /*!
   * \brief Get the latest configuration data.
   *
   * The configuration data will be updated in the background. This will always show the latest
   * received configuration data independent of the time that has passed since receiving it. If no
   * configuration data has been received yet, this will return a nullptr.
   */
  std::shared_ptr<ConfigurationData> getConfigurationData()
  {
    std::scoped_lock lock(configuration_data_mutex_);
    return configuration_data_;
  }

private:
  std::function<void(ErrorCode&)> error_code_message_callback_;
  std::shared_ptr<KinematicsInfo> kinematics_info_;
  std::mutex robot_mode_mutex_;
  std::shared_ptr<RobotModeData> robot_mode_;
  std::mutex version_information_mutex_;
  std::shared_ptr<VersionInformation> version_information_;
  std::shared_ptr<ConfigurationData> configuration_data_;
  std::mutex configuration_data_mutex_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
