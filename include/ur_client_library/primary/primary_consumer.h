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
 * This class implements am AbstractPrimaryConsumer such that it can consume all incoming primary
 * messages. The actual work for each package will be done in this class.
 */
class PrimaryConsumer : public AbstractPrimaryConsumer
{
public:
  PrimaryConsumer() : calibration_package_received_(false), robot_mode_data_received_(false)
  {
  }
  virtual ~PrimaryConsumer() = default;

  /*!
   * \brief Consume a RobotMessage
   *
   * \param pkg Robot message
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
   * \param pkg Robot state
   *
   * \returns True
   */
  virtual bool consume(RobotState& msg) override
  {
    return true;
  }

  /*!
   * \brief Consume a ProgramStateMessage
   *
   * \param pkg Program state message
   *
   * \returns True
   */
  virtual bool consume(ProgramStateMessage& msg) override
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
    URCL_LOG_ERROR("---VersionMessage---\n %s", pkg.toString().c_str());
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
    URCL_LOG_INFO("---KinematicsInfo---\n %s", pkg.toString().c_str());

    std::unique_lock<std::mutex> lk(calibration_data_mutex_);
    expected_hash_ = pkg.toHash();
    calibration_package_received_ = true;
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
    std::stringstream out_ss;
    out_ss << "---ErrorCodeMessage---\n" << pkg.toString().c_str() << std::endl;
    switch (pkg.report_level_)
    {
      case ReportLevel::DEBUG:
      case ReportLevel::DEVL_DEBUG:
      {
        URCL_LOG_DEBUG("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::INFO:
      case ReportLevel::DEVL_INFO:
      {
        URCL_LOG_INFO("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::WARNING:
      {
        URCL_LOG_WARN("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::VIOLATION:
      case ReportLevel::FAULT:
      case ReportLevel::DEVL_VIOLATION:
      case ReportLevel::DEVL_FAULT:
      {
        URCL_LOG_ERROR("%s", out_ss.str().c_str());
        break;
      }
      default:
      {
        std::stringstream ss;
        ss << "Unknown report level: " << static_cast<int>(pkg.report_level_) << std::endl << out_ss.str();
        URCL_LOG_ERROR("%s", ss.str().c_str());
      }
    }

    if (error_code_message_callback_ != nullptr)
    {
      error_code_message_callback_(pkg);
    }
    return true;
  }

  /*!
   * \brief Handle a RuntimeExceptionMessage
   *
   * \param pkg RuntimeExceptionMessage
   *
   * \returns True.
   */
  virtual bool consume(RuntimeExceptionMessage& pkg) override
  {
    URCL_LOG_ERROR("---RuntimeExceptionMessage---\n %s", pkg.toString().c_str());

    if (runtime_exception_message_callback_ != nullptr)
    {
      runtime_exception_message_callback_(pkg);
    }
    return true;
  }

  /*!
   * \brief Handle a KeyMessage
   *
   * \param pkg keyMessage
   *
   * \returns True.
   */
  virtual bool consume(KeyMessage& pkg) override
  {
    URCL_LOG_INFO("---KeyMessage---\n %s", pkg.toString().c_str());

    if (key_message_callback_ != nullptr)
    {
      key_message_callback_(pkg);
    }
    return true;
  }

  /*!
   * \brief Handle a RobotModeData package
   *
   * \param pkg RobotModeData
   *
   * \returns True
   */
  virtual bool consume(RobotModeData& pkg) override
  {
    std::lock_guard<std::mutex> lk(robot_mode_data_mutex_);
    robot_mode_data_.reset(new RobotModeData(pkg));
    robot_mode_data_received_ = true;
    robot_mode_data_cv_.notify_one();
    return true;
  }

  /*!
   * \brief Handle a TextMessage
   *
   * \param pkg TextMessage
   *
   * \returns True
   */
  virtual bool consume(TextMessage& pkg) override
  {
    URCL_LOG_INFO("---TextMessage---\n %s", pkg.toString().c_str());
    return true;
  }

  /*!
   * \brief Checks if the kinematics hash from the robot matches the expected kinematics hash
   *
   * \param expected_hash The expected kinematic hash
   *
   * \returns True if the robot's calibration checksum matches the one given to the checker. False
   * if it doesn't match
   */
  bool checkCalibration(const std::string& expected_hash)
  {
    while (!calibration_package_received_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::lock_guard<std::mutex> lk(calibration_data_mutex_);
    return expected_hash_ == expected_hash;
  }

  /*!
   * \brief Get the robot mode data package
   *
   * \param wait_for_new_package if true the function will wait until a new robot mode data package is received over the
   * primary interface, if false it will return the latest package received
   *
   * \returns RobotModeData
   */
  std::shared_ptr<RobotModeData> getRobotModeData(bool wait_for_new_package = false)
  {
    // Make sure we have received at least one package if we are not waiting for a package
    while (!robot_mode_data_received_ && wait_for_new_package == false)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (wait_for_new_package)
    {
      // Robot mode data is published with 10 Hz, so 500 milliseconds, should be plenty of time to wait for a new
      // package
      std::chrono::milliseconds timeout(500);
      std::unique_lock<std::mutex> lk(robot_mode_data_mutex_);
      if (robot_mode_data_cv_.wait_for(lk, timeout) == std::cv_status::no_timeout)
      {
        return robot_mode_data_;
      }
      URCL_LOG_WARN("Failed to receive new robot mode data package within the timeout, are you still connected to the "
                    "robot? Returning the last received package");
    }

    std::lock_guard<std::mutex> lk(robot_mode_data_mutex_);
    return robot_mode_data_;
  }

  /*!
   * \brief Set callback function which will be triggered whenever runtime exception messages are received
   *
   * \param callback_function Function handling the event information. The runtime exception message received is passed
   * to the function.
   */
  void setRuntimeExceptionMessageCallback(std::function<void(RuntimeExceptionMessage&)> callback_function)
  {
    runtime_exception_message_callback_ = callback_function;
  }

  /*!
   * \brief Set callback function which will be triggered whenever key messages are received
   *
   * \param callback_function Function handling the event information. The key message received is passed to the
   * function.
   */
  void setKeyMessageCallback(std::function<void(KeyMessage&)> callback_function)
  {
    key_message_callback_ = callback_function;
  }

  /*!
   * \brief Set callback function which will be triggered whenever error code messages are received
   *
   * \param callback_function Function handling the event information. The error code message received is passed to the
   * function.
   */
  void setErrorCodeMessageCallback(std::function<void(ErrorCodeMessage&)> callback_function)
  {
    error_code_message_callback_ = callback_function;
  }

private:
  std::function<void(RuntimeExceptionMessage&)> runtime_exception_message_callback_;
  std::function<void(KeyMessage&)> key_message_callback_;
  std::function<void(ErrorCodeMessage&)> error_code_message_callback_;

  std::string expected_hash_;
  std::mutex calibration_data_mutex_;

  std::shared_ptr<RobotModeData> robot_mode_data_;
  std::mutex robot_mode_data_mutex_;
  std::condition_variable robot_mode_data_cv_;

  // Variables used to ensure that a package of the type has been received
  bool calibration_package_received_;
  bool robot_mode_data_received_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
