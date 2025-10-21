// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------
#ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
#define UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED

#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/ur/version_information.h>
#include <ur_client_library/ur/dashboard_client_implementation.h>

namespace urcl
{
/*!
 * \brief This class is a wrapper around the dashboard server.
 *
 * For every Dashboard command there exists a wrapper function that will send the request and wait
 * for the server's response.
 *
 * For documentation about the dashboard server, please see
 *  - https://www.universal-robots.com/articles/ur/dashboard-server-cb-series-port-29999/
 *  - https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/
 */
class DashboardClient
{
public:
  enum class ClientPolicy
  {
    G5,
    POLYSCOPE_X
  };

  /*!
   * \brief Constructor that shall be used by default
   *
   * \param host IP address of the robot
   */
  DashboardClient(const std::string& host, const ClientPolicy client_policy = ClientPolicy::G5);
  DashboardClient() = delete;
  virtual ~DashboardClient() = default;

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
                       const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10));

  /*!
   * \brief Makes sure no connection to the dashboard server is held inside the object.
   */
  void disconnect();

  /*!
   * \brief Sends a command through the socket and waits for an answer.
   *
   * \param command Command that will be sent to the server.
   *
   * \throws UrException if no response was read from the dashboard server
   *
   * \returns Answer as received by the server cut off any trailing newlines.
   */
  std::string sendAndReceive(const std::string& command);

  /*!
   * \brief Sends command and compare it with the expected answer
   *
   * \param command Command that will be sent to the server.
   * \param expected Expected response as a regex string
   *
   * \return True if the reply to the command is as expected
   */
  bool sendRequest(const std::string& command, const std::string& expected = "");

  /*!
   * \brief Sends command and compare it with the expected answer
   *
   * \param command Command that will be sent to the server.
   * \param expected Expected response as a regex string
   *
   * \throws UrException if the received answer does not match the expected one.
   *
   * \return Answer string as received by the server
   */
  std::string sendRequestString(const std::string& command, const std::string& expected);

  /*!
   * \brief brief Sends a command and wait until it returns the expected answer
   *
   * \param command Command that will be sent to the server
   * \param expected Expected replay
   * \param timeout Timeout to wait before the command is considered failed.
   *
   *  \return True if the reply was as expected within the timeout time
   */
  bool waitForReply(const std::string& command, const std::string& expected,
                    std::chrono::duration<double> timeout = std::chrono::seconds(30));

  /*!
   * \brief Keep Sending the requesting Command and wait until it returns the expected answer.
   *
   * \param requestCommand Request command that will be sent to the server
   * \param requestExpectedResponse The expected reply to the request
   * \param waitRequest The status request
   * \param waitExpectedResponse The expected reply on the status
   * \param timeout Timeout before the command is ultimately considered failed
   * \param retry_period Retries will be done with this period
   *
   * \return True when both the requested command was receive with the expected reply as well as the resulting status
   * also is as expected within the timeout time
   */
  bool retryCommand(const std::string& requestCommand, const std::string& requestExpectedResponse,
                    const std::string& waitRequest, const std::string& waitExpectedResponse,
                    const std::chrono::duration<double> timeout,
                    const std::chrono::duration<double> retry_period = std::chrono::seconds(1));

  /*!
   * \brief Send Power off command
   *
   * \return True succeeded
   */
  bool commandPowerOff();

  /*!
   * \brief Send Power off command
   *
   */
  DashboardResponse commandPowerOffWithResponse();

  /*!
   * \brief Send Power on command
   *
   * \param timeout Timeout in seconds - The robot might take some time to boot before this call can
   * be made successfully.
   *
   * \return True succeeded
   */
  bool commandPowerOn(const std::chrono::duration<double> timeout = std::chrono::seconds(300));

  /*!
   * \brief Send Power on command
   *
   * \param timeout Timeout in seconds - The robot might take some time to boot before this call can
   * be made successfully.
   *
   */
  DashboardResponse commandPowerOnWithResponse(const std::chrono::duration<double> timeout = std::chrono::seconds(300));

  /*!
   * \brief Send Brake release command
   *
   * \return True succeeded
   */
  bool commandBrakeRelease();

  /*!
   * \brief Send Brake release command
   */
  DashboardResponse commandBrakeReleaseWithResponse();

  /*!
   * \brief Send Load program command
   *
   * \param program_file_name The urp program file name with the urp extension
   *
   * \return True succeeded
   */
  bool commandLoadProgram(const std::string& program_file_name);

  /*!
   * \brief Send Load program command and wait for a response
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_name': std::string
   *
   * \param program_file_name The urp program file name with the urp extension
   */
  DashboardResponse commandLoadProgramWithResponse(const std::string& program_file_name);

  /*!
   * \brief Send Load installation command
   *
   * \param installation_file_name The installation file name with the installation extension
   *
   * \return True succeeded
   */
  bool commandLoadInstallation(const std::string& installation_file_name);

  /*!
   * \brief Send Load installation command
   *
   * \param installation_file_name The installation file name with the installation extension
   */
  DashboardResponse commandLoadInstallationWithResponse(const std::string& installation_file_name);

  /*!
   * \brief Send Play program command
   *
   * \return True succeeded
   */
  bool commandPlay();

  /*!
   * \brief Send Play program command
   */
  DashboardResponse commandPlayWithResponse();

  /*!
   * \brief Send Pause program command
   *
   * \return True succeeded
   */
  bool commandPause();

  /*!
   * \brief Send Pause program command
   */
  DashboardResponse commandPauseWithResponse();

  /*!
   * \brief Send resume command
   *
   * This does only exist for PolyScope X, for e-Series robots "play" is used also for resuming a
   * paused program.
   *
   * \return True succeeded
   */
  bool commandResume();

  /*!
   * \brief Send resume command
   *
   * This does only exist for PolyScope X, for e-Series robots "play" is used also for resuming a
   * paused program.
   */
  DashboardResponse commandResumeWithResponse();

  /*!
   * \brief Send Stop program command
   *
   * \return True succeeded
   */
  bool commandStop();

  /*!
   * \brief Send Stop program command
   */
  DashboardResponse commandStopWithResponse();

  /*!
   * \brief Send Stop program command
   * \return True succeeded
   */
  bool commandClosePopup();

  /*!
   * \brief Send Close popup command
   */
  DashboardResponse commandClosePopupWithResponse();

  /*!
   * \brief Send Close safety popup command
   *
   * \return True succeeded
   */
  bool commandCloseSafetyPopup();

  /*!
   * \brief Send Close safety popup command
   */
  DashboardResponse commandCloseSafetyPopupWithResponse();

  /*!
   * \brief Send Restart Safety command
   *
   * \return True succeeded
   */
  bool commandRestartSafety();

  /*!
   * \brief Send Restart Safety command
   */
  DashboardResponse commandRestartSafetyWithResponse();

  /*!
   * \brief Send Unlock Protective stop popup command
   *
   * \return True succeeded
   */
  bool commandUnlockProtectiveStop();

  /*!
   * \brief Send Unlock Protective stop popup command
   */
  DashboardResponse commandUnlockProtectiveStopWithResponse();

  /*!
   * \brief Send Shutdown command
   *
   * \return True succeeded
   */
  bool commandShutdown();

  /*!
   * \brief Send Shutdown command
   */
  DashboardResponse commandShutdownWithResponse();

  /*!
   * \brief Send Quit command
   *
   * \return True succeeded
   */
  bool commandQuit();

  /*!
   * \brief Send Quit command
   */
  DashboardResponse commandQuitWithResponse();

  /*!
   * \brief Send Running command
   *
   * \return True succeeded
   */
  bool commandRunning();

  /*!
   * \brief Send Running command
   *
   * Stores the following entries in the data field:
   *
   *   - 'running': bool
   */
  DashboardResponse commandRunningWithResponse();

  /*!
   * \brief Send "Is program saved" request command
   *
   * \return True if the program is saved correctly
   */
  bool commandIsProgramSaved();

  /*!
   * \brief Send "Is program saved" request command
   *
   * Stores the following entries in the data field:
   *
   *   - 'saved': bool
   *   - 'program_name': std::string (PolyScope 5 only)
   */
  DashboardResponse commandIsProgramSavedWithResponse();

  /*!
   * \brief Send "Is in remote control" query command
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True if the robot is currently in remote control
   */
  bool commandIsInRemoteControl();

  /*!
   * \brief Send "Is in remote control" query command
   *
   * Stores the following entries in the data field:
   *
   *   - 'remote_control': bool
   */
  DashboardResponse commandIsInRemoteControlWithResponse();

  /*!
   * \brief Send popup command
   *
   * \param popup_text The text to be shown in the popup
   *
   * \return True succeeded
   */
  bool commandPopup(const std::string& popup_text);

  /*!
   * \brief Send popup command
   */
  DashboardResponse commandPopupWithResponse(const std::string& popup_text);

  /*!
   * \brief Send text to log
   *
   * \param log_text The text to be sent to the log
   *
   * \return True succeeded
   */
  bool commandAddToLog(const std::string& log_text);

  /*!
   * \brief Send text to log
   *
   * \param log_text The text to be sent to the log
   */
  DashboardResponse commandAddToLogWithResponse(const std::string& log_text);

  /*!
   * \brief Get Polyscope version
   *
   * \param polyscope_version The string for the polyscope version number returned
   *
   * \return True succeeded
   */
  bool commandPolyscopeVersion(std::string& polyscope_version);

  /*!
   * \brief Get Polyscope version
   *
   * Stores the following entries in the data field:
   *
   *   - 'polyscope_version': std::string
   */
  DashboardResponse commandPolyscopeVersionWithResponse();

  /*!
   * \brief Get Robot model
   *
   * \param robot_model The string for the robot model returned
   *
   * \return True succeeded
   */
  bool commandGetRobotModel(std::string& robot_model);

  /*!
   * \brief Get Robot model
   *
   * Stores the following entries in the data field:
   *
   *   - 'robot_model': std::string
   */
  DashboardResponse commandGetRobotModelWithResponse();

  /*!
   * \brief Get Serial number
   *
   * \param serial_number The serial number of the robot returned
   *
   * \return True succeeded
   */
  bool commandGetSerialNumber(std::string& serial_number);

  /*!
   * \brief Get Serial number
   *
   * Stores the following entries in the data field:
   *
   *   - 'serial_number': std::string
   */
  DashboardResponse commandGetSerialNumberWithResponse();

  /*!
   * \brief Get Robot mode
   *
   * \param robot_mode The mode of the robot returned
   *
   * \return True succeeded
   */
  bool commandRobotMode(std::string& robot_mode);

  /*!
   * \brief Get Robot mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'robot_mode': std::string
   */
  DashboardResponse commandRobotModeWithResponse();

  /*!
   * \brief Get Loaded Program
   *
   * \param loaded_program The path to the loaded program
   *
   * \return True succeeded
   */
  bool commandGetLoadedProgram(std::string& loaded_program);

  /*!
   * \brief Get Loaded Program
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_name': std::string
   */
  DashboardResponse commandGetLoadedProgramWithResponse();

  /*!
   * \brief Get Safety mode
   *
   * \param safety_mode The safety mode of the robot returned
   *
   * \return True succeeded
   */
  bool commandSafetyMode(std::string& safety_mode);

  /*!
   * \brief Get Safety mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'safety_mode': std::string
   */
  DashboardResponse commandSafetyModeWithResponse();

  /*!
   * \brief Get Safety status
   *
   * \param safety_status The safety status of the robot returned
   *
   * \return True succeeded
   */
  bool commandSafetyStatus(std::string& safety_status);

  /*!
   * \brief Get Safety status
   *
   * Stores the following entries in the data field:
   *
   *   - 'safety_status': std::string
   */
  DashboardResponse commandSafetyStatusWithResponse();

  /*!
   * \brief Get Program state
   *
   * \param program_state The program state of the robot returned
   *
   * \return True succeeded
   */
  bool commandProgramState(std::string& program_state);

  /*!
   * \brief Get Program state
   *
   * Stores the following entries in the data field:
   *
   *   - 'program_state': std::string
   *   - 'program_name': std::string
   */
  DashboardResponse commandProgramStateWithResponse();

  /*!
   * \brief Get Operational mode
   *
   * \param operational_mode The operational mode of the robot returned (Only available for e-series)
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True succeeded
   */
  bool commandGetOperationalMode(std::string& operational_mode);

  /*!
   * \brief Get Operational mode
   *
   * Stores the following entries in the data field:
   *
   *   - 'operational_mode': std::string
   */
  DashboardResponse commandGetOperationalModeWithResponse();

  /*!
   * \brief Send Set operational mode command (Only available for e-series)
   *
   * \param operational_mode The operational mode to set on the robot
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True succeeded
   */
  bool commandSetOperationalMode(const std::string& operational_mode);

  /*!
   * \brief Send Set operational mode command (Only available for e-series)
   *
   * \param operational_mode The operational mode to set on the robot
   */
  DashboardResponse commandSetOperationalModeWithResponse(const std::string& operational_mode);

  /*!
   * \brief Send Clear operational mode command
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True succeeded
   */
  bool commandClearOperationalMode();

  /*!
   * \brief Send Clear operational mode command
   *
   * \throws an UrException when called on CB3 robots
   */
  DashboardResponse commandClearOperationalModeWithResponse();

  /*!
   * \brief Send Set user role command (Only available for CB3)
   *
   * \param user_role The user role to set on the robot
   *
   * \throws an UrException when called on e-series robots
   *
   * \return True succeeded
   */
  bool commandSetUserRole(const std::string& user_role);

  /*!
   * \brief Send Set user role command (Only available for CB3)
   *
   * \throws an UrException when called on e-series robots
   */
  DashboardResponse commandSetUserRoleWithResponse(const std::string& user_role);

  /*!
   * \brief Send Get user role command (Only available for CB3)
   *
   * \param user_role The user role on the robot
   *
   * \throws an UrException when called on e-series robots
   *
   * \return True succeeded
   */
  bool commandGetUserRole(std::string& user_role);

  /*!
   * \brief Send Get user role command (Only available for CB3)
   *
   * Stores the following entries in the data field:
   *
   *   - 'user_role': std::string
   *
   * \throws an UrException when called on e-series robots
   */
  DashboardResponse commandGetUserRoleWithResponse();

  /*!
   * \brief Send Generate flight report command
   *
   * \note This may take a long time to run.
   *
   * \param report_type The report type to set for the flight report
   *
   * \return True succeeded
   */
  bool commandGenerateFlightReport(const std::string& report_type);

  /*!
   * \brief Send Generate flight report command
   *
   * \note This may take a long time to run.
   *
   * \param report_type The report type to set for the flight report
   */
  DashboardResponse commandGenerateFlightReportWithResponse(const std::string& report_type);

  /*!
   * \brief Send Generate support file command
   *
   * \note This may take a long time to run.
   *
   * \param dir_path The path to the directory of an already existing directory location inside the programs directory,
   * where the support file is saved
   *
   * \return True succeeded
   */
  bool commandGenerateSupportFile(const std::string& dir_path);

  /*!
   * \brief Send Generate support file command
   *
   * \note This may take a long time to run.
   *
   * \param dir_path The path to the directory of an already existing directory location inside the programs directory,
   * where the support file is saved
   */
  DashboardResponse commandGenerateSupportFileWithResponse(const std::string& dir_path);

  /*!
   * \brief Flush the polyscope log to the log_history.txt file
   *
   * \return True succeeded
   */
  bool commandSaveLog();

  /*!
   * \brief Flush the polyscope log to the log_history.txt file
   */
  DashboardResponse commandSaveLogWithResponse();

  /*!
   * \brief Makes sure that the dashboard_server's version is above the required version
   *
   * \param e_series_min_ver SW version for e-Series
   * \param cb3_min_ver SW version for cb3
   * \param required_call The dashboard call that should be checked
   *
   * \throws UrException if the robot's version isn't large enough
   */
  void assertVersion(const std::string& e_series_min_ver, const std::string& cb3_min_ver,
                     const std::string& required_call);

  /*!
   * \brief Gets the configured receive timeout. If receive timeout is unconfigured "normal" socket timeout of 1 second
   * will be returned
   *
   * \returns configured receive timeout
   */
  timeval getConfiguredReceiveTimeout() const;

  /*!
   * \brief Setup Receive timeout used for this socket.
   *
   * \param timeout Timeout used for setting things up
   */
  void setReceiveTimeout(const timeval& timeout);

protected:
  std::shared_ptr<DashboardClientImpl> impl_;
};
}  // namespace urcl
#endif  // ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
