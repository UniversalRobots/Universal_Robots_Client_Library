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

#ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED

#include <chrono>
#include <memory>
#include <deque>

#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/primary/abstract_primary_consumer.h>
#include <ur_client_library/primary/primary_consumer.h>
#include <ur_client_library/primary/primary_package.h>
#include <ur_client_library/primary/primary_parser.h>

namespace urcl
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip, comm::INotifier& notifier);
  ~PrimaryClient();

  /*!
   * \brief Adds a primary consumer to the list of consumers
   *
   * \param primary_consumer Primary consumer that should be added to the list
   */
  void addPrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer);

  /*!
   * \brief Remove a primary consumer from the list of consumers
   *
   * \param primary_consumer Primary consumer that should be removed from the list
   */
  void removePrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer);
  void start(const size_t max_connection_attempts = 0,
             const std::chrono::milliseconds reconnection_timeout = urcl::comm::TCPSocket::DEFAULT_RECONNECTION_TIME);
  void stop();

  /*!
   * \brief Retrieves previously raised error codes from PrimaryClient. After calling this, recorded errors will be
   * deleted.
   */
  std::deque<ErrorCode> getErrorCodes();

  /*!
   * \brief Sends a custom script program to the robot.
   *
   * The given code must be valid according the UR Scripting Manual.
   *
   * \param program URScript code that shall be executed by the robot.
   *
   * \returns true on successful upload, false otherwise.
   */
  bool sendScript(const std::string& program);

  bool checkCalibration(const std::string& checksum);

  /*!
   * \brief Commands the robot to power on.
   *
   * \param validate If true, the function will block until the robot is powered on or the timeout
   * passed by.
   * \param timeout The maximum time to wait for the robot to confirm the power on command.
   *
   * \throws urcl::UrException if the command cannot be sent to the robot.
   * \throws urcl::TimeoutException if the robot doesn't power on within the given timeout.
   */
  void commandPowerOn(const bool validate = true, const std::chrono::milliseconds timeout = std::chrono::seconds(30));

  /*!
   * \brief Commands the robot to power off.
   *
   * \param validate If true, the function will block until the robot is powered off or the timeout
   * passed by.
   * \param timeout The maximum time to wait for the robot to confirm the power off command.
   *
   * \throws urcl::UrException if the command cannot be sent to the robot.
   * \throws urcl::TimeoutException if the robot doesn't power off within the given timeout.
   */
  void commandPowerOff(const bool validate = true, const std::chrono::milliseconds timeout = std::chrono::seconds(30));

  /*!
   * \brief Commands the robot to release the brakes
   *
   * \param validate If true, the function will block until the robot is running or the timeout
   * passed by.
   * \param timeout The maximum time to wait for the robot to confirm it is running.
   *
   * \throws urcl::UrException if the command cannot be sent to the robot.
   * \throws urcl::TimeoutException if the robot doesn't release the brakes within the given
   * timeout.
   */
  void commandBrakeRelease(const bool validate = true,
                           const std::chrono::milliseconds timeout = std::chrono::seconds(30));

  /*!
   * \brief Commands the robot to unlock the protective stop
   *
   * \param validate If true, the function will block until the protective stop is released or the
   * timeout passed by.
   * \param timeout The maximum time to wait for the robot to confirm it is no longer protective
   * stopped.
   *
   * \throws urcl::UrException if the command cannot be sent to the robot.
   * \throws urcl::TimeoutException if the robot doesn't unlock the protective stop within the
   * given timeout.
   */
  void commandUnlockProtectiveStop(const bool validate = true,
                                   const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));

  /*!
   * /brief Stop execution of a running or paused program
   *
   * \param validate If true, the function will block until the robot has stopped or the timeout
   * passed by.
   * \param timeout The maximum time to wait for the robot to stop the program.
   *
   * \throws urcl::UrException if the command cannot be sent to the robot.
   * \throws urcl::TimeoutException if the robot doesn't stop the program within the given timeout.
   */
  void commandStop(const bool validate = true, const std::chrono::milliseconds timeout = std::chrono::seconds(2));

  /*!
   * \brief Get the latest robot mode.
   *
   * The robot mode will be updated in the background. This will always show the latest received
   * robot mode independent of the time that has passed since receiving it.
   */
  RobotMode getRobotMode()
  {
    std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
    if (robot_mode_data == nullptr)
    {
      return RobotMode::UNKNOWN;
    }
    return static_cast<RobotMode>(consumer_->getRobotModeData()->robot_mode_);
  }

  /*!
   * \brief Get the robot's software version as Major.Minor.Bugfix
   *
   * This function by default blocks until a VersionMessage has been received and returns that
   * version information. If there is an older version message that has been received, this is
   * returned directly.
   *
   * \param blocking If true, the function will block until there is a valid version information
   * received by the client or the timeout passed by.
   * \param timeout The maximum time to wait for a valid version message.
   *
   * \throws urcl::TimeoutException if no message was received until the given timeout passed by.
   *
   */
  std::shared_ptr<VersionInformation>
  getRobotVersion(bool wait_for_message = true, const std::chrono::milliseconds timeout = std::chrono::seconds(2));

  /*!
   * \brief Get the latest robot mode data.
   *
   * The robot's mode data will be updated in the background. This will always show the latest received
   * state independent of the time that has passed since receiving it. The return value of this
   * will be a nullptr if no data has been received yet.
   */
  std::shared_ptr<RobotModeData> getRobotModeData()
  {
    return consumer_->getRobotModeData();
  }

  /*!
   * \brief Query if the robot is protective stopped.
   *
   * The robot's protective_stop state will be updated in the background. This will always show the latest received
   * state independent of the time that has passed since receiving it.
   *
   * \throws UrException when no robot mode data has been received, yet.
   */
  bool isRobotProtectiveStopped()
  {
    std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
    if (robot_mode_data == nullptr)
    {
      throw UrException("Robot mode data is a nullptr. Probably it hasn't been received, yet.");
    }
    return robot_mode_data->is_protective_stopped_;
  }

  /*!
   * \brief Get the latest configuration data
   *
   * The configuration data will be updated in the background. This will always show the latest
   * received configuration data independent of the time that has passed since receiving it. If no
   * configuration data has been received yet, this will return a nullptr.
   */
  std::shared_ptr<ConfigurationData> getConfigurationData()
  {
    return consumer_->getConfigurationData();
  }

  /*!
   * \brief Get the Robot type
   *
   * If no robot type data has been received yet, this will return UNDEFINED.
   */
  RobotType getRobotType();

private:
  /*!
   * \brief Reconnects the primary stream used to send program to the robot.
   *
   * Only for use in headless mode, as it replaces the use of the URCaps program.
   *
   * \returns true of on successful reconnection, false otherwise
   */
  bool reconnectStream();

  // The function is called whenever an error code message is received
  void errorMessageCallback(ErrorCode& code);

  PrimaryParser parser_;
  std::shared_ptr<PrimaryConsumer> consumer_;
  std::unique_ptr<comm::MultiConsumer<PrimaryPackage>> multi_consumer_;

  comm::INotifier notifier_;

  comm::URStream<PrimaryPackage> stream_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> prod_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;

  std::mutex error_code_queue_mutex_;
  std::deque<ErrorCode> error_code_queue_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
