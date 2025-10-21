// Copyright 2025 Universal Robots A/S
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

#pragma once

#include <chrono>
#include <string>
#include <variant>

#include <ur_client_library/exceptions.h>

namespace urcl
{

struct DashboardResponse
{
  bool ok = false;
  std::string message;
  std::unordered_map<std::string, std::variant<std::string, int, bool>> data;
};

class DashboardClientImpl
{
public:
  DashboardClientImpl() = default;
  DashboardClientImpl(const std::string& host) : host_(host) {};

  virtual ~DashboardClientImpl() = default;

  /*!
   * \brief Sends a command through the socket and waits for an answer.
   *
   * \param command Command that will be sent to the server.
   *
   * \throws UrException if no response was read from the dashboard server
   *
   * \returns Answer as received by the server cut off any trailing newlines.
   */
  virtual std::string sendAndReceive(const std::string& command) = 0;

  /*!
   * \brief Opens a connection to the dashboard server on the host as specified in the constructor.
   *
   * \param max_num_tries Maximum number of connection attempts before counting the connection as
   * failed. Unlimited number of attempts when set to 0.
   * \param reconnection_time time in between connection attempts to the server
   *
   * \returns True on successful connection, false otherwise.
   */
  virtual bool connect(const size_t max_num_tries = 0,
                       const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10)) = 0;

  /*!
   * \brief Makes sure no connection to the dashboard server is held inside the object.
   */
  virtual void disconnect() = 0;

  /*!
   * \brief Gets the configured receive timeout. If receive timeout is unconfigured "normal" socket timeout of 1 second
   * will be returned
   *
   * \returns configured receive timeout
   */
  virtual timeval getConfiguredReceiveTimeout() const = 0;

  /*!
   * \brief Sets the receive timeout for the socket.
   *
   * \param timeout The timeout to be set
   */
  virtual void setReceiveTimeout(const timeval& timeout) {};

  /*!
   * \brief Sends command and verifies that a valid answer is received.
   *
   * \param command Command that will be sent to the server.
   * \param expected_response_pattern Expected response as a regex string.
   * \param payload Optional payload to be sent with the command.
   *
   * \return True if the reply to the command is as expected
   */
  virtual bool sendRequest(const std::string& command, const std::string& expeted_response_pattern = "",
                           const std::string& payload = "") = 0;

  /*!
   * \brief Sends command and compare it with the expected answer
   *
   * \param command Command that will be sent to the server.
   * \param expected Expected response as a regex string
   * \param payload Optional payload to be sent with the command.
   *
   * \throws UrException if the received answer does not match the expected one.
   *
   * \return Answer string as received by the server
   */
  virtual std::string sendRequestString(const std::string& command, const std::string& expected = "",
                                        const std::string& payload = "") = 0;

  /*!
   * \brief Send Power off command
   */
  virtual DashboardResponse commandPowerOff() = 0;

  /*!
   * \brief Send Power on command
   *
   * \param timeout Timeout in seconds - The robot might take some time to boot before this call can
   * be made successfully.
   */
  virtual DashboardResponse commandPowerOn(const std::chrono::duration<double> timeout = std::chrono::seconds(300)) = 0;

  /*!
   * \brief Send Brake release command
   */
  virtual DashboardResponse commandBrakeRelease() = 0;

  /*!
   * \brief Send Load program command
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_name': std::string
   *
   * \param program_file_name The urp program file name with the urp extension
   */
  virtual DashboardResponse commandLoadProgram(const std::string& program_file_name) = 0;

  /*!
   * \brief Send Load installation command
   *
   * Stores the following entries in the data field:
   *
   *  - 'installation_name': std::string
   *
   * \param installation_file_name The installation file name with the installation extension
   */
  virtual DashboardResponse commandLoadInstallation(const std::string& installation_file_name) = 0;

  /*!
   * \brief Send Play program command
   */
  virtual DashboardResponse commandPlay() = 0;

  /*!
   * \brief Send Pause program command
   */
  virtual DashboardResponse commandPause() = 0;

  /*!
   * \brief Send resume command
   *
   * This does only exist for PolyScope X, for e-Series robots "play" is used also for resuming a
   * paused program.
   */
  virtual DashboardResponse commandResume() = 0;

  /*!
   * \brief Send Stop program command
   */
  virtual DashboardResponse commandStop() = 0;

  /*!
   * \brief Send Close popup command
   */
  virtual DashboardResponse commandClosePopup() = 0;

  /*!
   * \brief Send Close safety popup command
   */
  virtual DashboardResponse commandCloseSafetyPopup() = 0;

  /*!
   * \brief Send Restart Safety command
   */
  virtual DashboardResponse commandRestartSafety() = 0;

  /*!
   * \brief Send Unlock Protective stop popup command
   */
  virtual DashboardResponse commandUnlockProtectiveStop() = 0;

  /*!
   * \brief Send Shutdown command
   */
  virtual DashboardResponse commandShutdown() = 0;

  /*!
   * \brief Send Quit command
   */
  virtual DashboardResponse commandQuit() = 0;

  /*!
   * \brief Send Running command
   *
   * Stores the following entries in the data field:
   *
   *   - 'running': bool
   */
  virtual DashboardResponse commandRunning() = 0;

  /*!
   * \brief Send "Is program saved" request command
   *
   * Stores the following entries in the data field:
   *
   *   - 'saved': bool
   */
  virtual DashboardResponse commandIsProgramSaved() = 0;

  /*!
   * \brief Send "Is in remote control" query command
   *
   * Stores the following entries in the data field:
   *
   *   - 'remote_control': bool
   *
   * \throws an UrException when called on CB3 robots
   */
  virtual DashboardResponse commandIsInRemoteControl() = 0;

  /*!
   * \brief Send popup command
   *
   * \param popup_text The text to be shown in the popup
   */
  virtual DashboardResponse commandPopup(const std::string& popup_text) = 0;

  /*!
   * \brief Send text to log
   *
   * \param log_text The text to be sent to the log
   */
  virtual DashboardResponse commandAddToLog(const std::string& log_text) = 0;

  /*!
   * \brief Get Polyscope version
   *
   * Stores the following entries in the data field:
   *
   *   - 'polyscope_version': std::string
   */
  virtual DashboardResponse commandPolyscopeVersion() = 0;

  /*!
   * \brief Get Robot model
   *
   * Stores the following entries in the data field:
   *
   *   - 'robot_model': std::string
   */
  virtual DashboardResponse commandGetRobotModel() = 0;

  /*!
   * \brief Get Serial number
   *
   * Stores the following entries in the data field:
   *
   *   - 'serial_number': std::string
   */
  virtual DashboardResponse commandGetSerialNumber() = 0;

  /*!
   * \brief Get Robot mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'robot_mode': std::string
   */
  virtual DashboardResponse commandRobotMode() = 0;

  /*!
   * \brief Get Loaded Program
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_name': std::string
   */
  virtual DashboardResponse commandGetLoadedProgram() = 0;

  /*!
   * \brief Get Safety mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'safety_mode': std::string
   */
  virtual DashboardResponse commandSafetyMode() = 0;

  /*!
   * \brief Get Safety status
   *
   * Stores the following entries in the data field:
   *
   *   - 'safety_status': std::string
   */
  virtual DashboardResponse commandSafetyStatus() = 0;

  /*!
   * \brief Get Program state
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_state': std::string
   *   - 'program_name': std::string
   */
  virtual DashboardResponse commandProgramState() = 0;

  /*!
   * \brief Get Operational mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'operational_mode': std::string
   *
   * \throws an UrException when called on CB3 robots
   */
  virtual DashboardResponse commandGetOperationalMode() = 0;

  /*!
   * \brief Send Set operational mode command (Only available for e-series)
   *
   * \param operational_mode The operational mode to set on the robot
   *
   * \throws an UrException when called on CB3 robots
   */
  virtual DashboardResponse commandSetOperationalMode(const std::string& operational_mode) = 0;

  /*!
   * \brief Send Clear operational mode command
   *
   * \throws an UrException when called on CB3 robots
   */
  virtual DashboardResponse commandClearOperationalMode() = 0;

  /*!
   * \brief Send Set user role command (Only available for CB3)
   *
   * \param user_role The user role to set on the robot
   *
   * \throws an UrException when called on e-series robots
   */
  virtual DashboardResponse commandSetUserRole(const std::string& user_role) = 0;

  /*!
   * \brief Send Get user role command (Only available for CB3)
   *
   *
   * \throws an UrException when called on e-series robots
   */
  virtual DashboardResponse commandGetUserRole() = 0;

  /*!
   * \brief Send Generate flight report command
   *
   * \param report_type The report type to set for the flight report
   */
  virtual DashboardResponse commandGenerateFlightReport(const std::string& report_type) = 0;

  /*!
   * \brief Send Generate support file command
   *
   * \param dir_path The path to the directory of an already existing directory location inside the programs directory,
   * where the support file is saved
   */
  virtual DashboardResponse commandGenerateSupportFile(const std::string& dir_path) = 0;

  /*!
   * \brief Flush the polyscope log to the log_history.txt file
   */
  virtual DashboardResponse commandSaveLog() = 0;

  const VersionInformation& getPolyscopeVersion() const
  {
    return polyscope_version_;
  }

protected:
  virtual void assertHasCommand(const std::string& command) const = 0;

  VersionInformation polyscope_version_;
  std::string host_;
};
}  // namespace urcl
