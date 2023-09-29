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
#include <ur_client_library/primary/primary_consumer.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/pipeline.h>

#include <mutex>
#include <condition_variable>
#include <chrono>

namespace urcl
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip);
  virtual ~PrimaryClient() = default;

  /*!
   * \brief Checks if the kinematics information in the used model fits the actual robot.
   *
   * \param checksum Hash of the used kinematics information
   *
   * \returns True if the robot's calibration checksum matches the one given to the checker. False
   * if it doesn't match
   */
  bool checkCalibration(const std::string& checksum) const;

  /*!
   * \brief Sends a custom script program to the robot.
   *
   * This function will wait until feedback is received from the robot. This could be either error feedback or that the
   * script has started running.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param script_code URScript code that shall be executed by the robot.
   * \param timeout Time to wait for feedback from the robot
   *
   * \returns true if the scripts starts running successfully, false otherwise.
   */
  bool sendScript(const std::string& script_code,
                  const std::chrono::milliseconds timeout = std::chrono::milliseconds(500));

  /*!
   * \brief Sends a secondary custom script program to the robot.
   *
   * The UR robot supports executing secondary script programs alongside a running program. This function is
   * for executing secondary script programs alongside a running program and it will wait for error feedback
   * if any is received, but it will not wait for the script to start running.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param script_code URScript code that shall be executed by the robot.
   * \param timeout Time to wait for feedback from the robot
   *
   * \returns true if no error feedback is received within the timeout, false otherwise.
   */
  bool sendSecondaryScript(const std::string& script_code,
                           const std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

  /*!
   * \brief Reconnect to the primary interface, this is necessary if you switch from local to remote control, in order
   * to be able to send script code to the robot
   */
  void reconnect() const;

  /*!
   * \brief Adds a primary consumer to the list of consumers
   *
   * \param primary_consumer Primary consumer that should be added to the list
   */
  void addPrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer);

  /*!
   * \brief Remove a primary consumer from the list of consumers
   *
   * \param primary_consumer Primary consumer that should be removed from the list
   */
  void removePrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer);

private:
  // Used internally to determine the type of feedback received from the robot, when executing script code
  enum class RobotFeedbackType : uint8_t
  {
    ErrorMessage = 0,
    RuntimeException = 1,
    KeyMessage = 2
  };

  // The function is called whenever a key message is received
  void keyMessageCallback(KeyMessage& msg);

  // The function is called whenever an error code message is received
  void errorMessageCallback(ErrorCodeMessage& msg);

  // The function is called whenever a runtime exception message is received
  void runtimeExceptionMessageCallback(RuntimeExceptionMessage& msg);

  // Wait for feedback from the robot, returns true if the program starts running, false if an error is received or if
  // the function times out
  bool waitForRobotFeedback(const std::chrono::milliseconds timeout);

  // Wait for error feedback from the robot, returns true if no error is received within the timeout, false if an error
  // is received
  bool waitForRobotErrorFeedback(const std::chrono::milliseconds timeout);

  std::string robot_ip_;
  PrimaryParser parser_;

  std::shared_ptr<PrimaryConsumer> consumer_;
  std::unique_ptr<comm::MultiConsumer<PrimaryPackage>> multi_consumer_;

  comm::INotifier notifier_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> producer_;
  std::unique_ptr<comm::URStream<PrimaryPackage>> stream_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;

  std::mutex robot_feedback_mutex_;
  std::condition_variable robot_feedback_cv_;
  RobotFeedbackType robot_feedback_type_;
  std::unique_ptr<KeyMessage> key_message_;
  std::unique_ptr<ErrorCodeMessage> error_code_message_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED