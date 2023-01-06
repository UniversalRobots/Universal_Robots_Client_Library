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
#ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED

#include <ur_client_library/primary/primary_parser.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/ur/calibration_checker.h>
#include <ur_client_library/primary/primary_consumer.h>
#include <ur_client_library/ur/dashboard_client.h>

namespace urcl
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip, const std::string& calibration_checksum);
  // virtual ~PrimaryClient() = default;
  virtual ~PrimaryClient();

  /*!
   * \brief Sends a custom script program to the robot.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param script_code URScript code that shall be executed by the robot.
   *
   * \returns true on successful upload, false otherwise.
   */
  bool sendScript(const std::string& script_code);

  /*!
   * \brief Configures the primary client
   *
   * Creates a connection to the stream and sets up producer, consumer and pipeline
   */
  bool configure();

  /*!
   * \brief Checks if the robot is in local or remote control
   *
   * Checks for package with error code determining if robot is in remote or local control
   */
  void checkRemoteLocalControl();

  /*!
   * \brief Stops the primary client
   *
   * Closes the thread checking for remote or local control
   */
  void stop();

  /*!
   * \brief Returns whether the robot is in remote or local control
   *
   * Returns whether the robot is in remote or local control
   */
  bool isInRemoteControl();

  /*!
   * \brief Returns latest AdditionalInfo message
   *
   * Returns latest AdditionalInfo message
   */
  std::shared_ptr<AdditionalInfo> getAdditionalInfo();

  /*!
   * \brief Returns latest CartesianInfo message
   *
   * Returns latest CartesianInfo message
   */
  std::shared_ptr<CartesianInfo> getCartesianInfo();

  /*!
   * \brief Returns latest ForceModeData message
   *
   * Returns latest ForceModeData message
   */
  std::shared_ptr<ForceModeData> getForceModeData();

  /*!
   * \brief Returns latest JointData message
   *
   * Returns latest JointData message
   */
  std::shared_ptr<JointData> getJointData();

  /*!
   * \brief Returns latest RobotModeData message
   *
   * Returns latest RobotModeData message
   */
  std::shared_ptr<RobotModeData> getRobotModeData();

  /*!
   * \brief Returns latest KeyMessage message
   *
   * Returns latest KeyMessage message
   */
  std::shared_ptr<KeyMessage> getKeyMessage();

  /*!
   * \brief Returns latest ErrorCodeMessage message
   *
   * Returns latest ErrorCodeMessage message
   */
  std::shared_ptr<ErrorCodeMessage> getErrorCodeMessage();

  /*!
   * \brief Returns latest RuntimeExceptionMessage message
   *
   * Returns latest RuntimeExceptionMessage message
   */
  std::shared_ptr<RuntimeExceptionMessage> getRuntimeExceptionMessage();

  /*!
   * \brief Returns latest TextMessage message
   *
   * Returns latest TextMessage message
   */
  std::shared_ptr<TextMessage> getTextMessage();

  /*!
   * \brief Returns latest VersionMessage message
   *
   * Returns latest VersionMessage message
   */
  std::shared_ptr<VersionMessage> getVersionMessage();

  /*!
   * \brief Returns latest GlobalVariablesSetupMessage message
   *
   * Returns latest GlobalVariablesSetupMessage message
   */
  std::shared_ptr<GlobalVariablesSetupMessage> getGlobalVariablesSetupMessage();

  /*!
   * \brief Returns Calibration checker
   *
   * Returns Calibration checker
   */
  std::shared_ptr<CalibrationChecker> getCalibrationChecker();

  /*!
   * \brief Set whether the robot is simulated or real
   */
  void setSimulated(bool value);

  /*!
   * \brief Check if the robot is simulated or real
   *
   * \returns true if robot is simulated, false otherwise.
   */
  bool getSimulated();

private:
  std::string robot_ip_;
  int port_;
  PrimaryParser parser_;
  comm::INotifier notifier_;
  bool connected_;
  bool simulated_;
  std::atomic<bool> running_, in_remote_control_;
  std::unique_ptr<PrimaryConsumer> consumer_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> producer_;
  std::unique_ptr<comm::URStream<PrimaryPackage>> stream_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;
  std::unique_ptr<urcl::DashboardClient> dashboard_client_;
  std::shared_ptr<CalibrationChecker> calibration_checker_;
  std::thread rc_thread_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
