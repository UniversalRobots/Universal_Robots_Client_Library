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

namespace urcl
{
/*!
 * \brief This class is a wrapper around the dashboard server.
 *
 * For every Dashboard command there exists a wrapper function that will send the request and wait
 * for the server's response.
 *
 * For documentation about the dashboard server, please see
 *  - https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-cb-series-port-29999-15690/
 *  - https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
 */
class DashboardClient : public comm::TCPSocket
{
public:
  /*!
   * \brief Constructor that shall be used by default
   *
   * \param host IP address of the robot
   */
  DashboardClient(const std::string& host);
  DashboardClient() = delete;
  virtual ~DashboardClient() = default;

  static constexpr int DASHBOARD_SERVER_PORT = 29999;

  /*!
   * \brief Opens a connection to the dashboard server on the host as specified in the constructor.
   *
   * \param max_num_tries Maximum number of connection attempts before counting the connection as
   * failed. Unlimited number of attempts when set to 0.
   * \param reconnection_time time in between connection attempts to the server
   *
   * \returns True on successful connection, false otherwise.
   */
  bool connect(const size_t max_num_tries = 0,
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
   * \param expected Expected response
   *
   * \return True if the reply to the command is as expected
   */
  bool sendRequest(const std::string& command, const std::string& expected);

  /*!
   * \brief Sends command and compare it with the expected answer
   *
   * \param command Command that will be sent to the server.
   * \param expected Expected response
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
   * \brief Send Power on command
   *
   * \param timeout Timeout in seconds - The robot might take some time to boot before this call can
   * be made successfully.
   *
   * \return True succeeded
   */
  bool commandPowerOn(const std::chrono::duration<double> timeout = std::chrono::seconds(300));

  /*!
   * \brief Send Brake release command
   *
   * \return True succeeded
   */
  bool commandBrakeRelease();

  /*!
   * \brief Send Load program command
   *
   * \param program_file_name The urp program file name with the urp extension
   *
   * \return True succeeded
   */
  bool commandLoadProgram(const std::string& program_file_name);

  /*!
   * \brief Send Load installation command
   *
   * \param installation_file_name The installation file name with the installation extension
   *
   * \return True succeeded
   */
  bool commandLoadInstallation(const std::string& installation_file_name);

  /*!
   * \brief Send Play program command
   *
   * \return True succeeded
   */
  bool commandPlay();

  /*!
   * \brief Send Pause program command
   *
   * \return True succeeded
   */
  bool commandPause();

  /*!
   * \brief Send Stop program command
   *
   * \return True succeeded
   */
  bool commandStop();

  /*!
   * \brief Send Close popup command
   *
   * \return True succeeded
   */
  bool commandClosePopup();

  /*!
   * \brief Send Close safety popup command
   *
   * \return True succeeded
   */
  bool commandCloseSafetyPopup();

  /*!
   * \brief Send Restart Safety command
   *
   * \return True succeeded
   */
  bool commandRestartSafety();

  /*!
   * \brief Send Unlock Protective stop popup command
   *
   * \return True succeeded
   */
  bool commandUnlockProtectiveStop();

  /*!
   * \brief Send Shutdown command
   *
   * \return True succeeded
   */
  bool commandShutdown();

  /*!
   * \brief Send Quit command
   *
   * \return True succeeded
   */
  bool commandQuit();

  /*!
   * \brief Send Running command
   *
   * \return True succeeded
   */
  bool commandRunning();

  /*!
   * \brief Send "Is program saved" request command
   *
   * \return True if the program is saved correctly
   */
  bool commandIsProgramSaved();

  /*!
   * \brief Send "Is in remote control" query command
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True if the robot is currently in remote control
   */
  bool commandIsInRemoteControl();

  /*!
   * \brief Send popup command
   *
   * \param popup_text The text to be shown in the popup
   *
   * \return True succeeded
   */
  bool commandPopup(const std::string& popup_text);

  /*!
   * \brief Send text to log
   *
   * \param log_text The text to be sent to the log
   *
   * \return True succeeded
   */
  bool commandAddToLog(const std::string& log_text);

  /*!
   * \brief Get Polyscope version
   *
   * \param polyscope_version The string for the polyscope version number returned
   *
   * \return True succeeded
   */
  bool commandPolyscopeVersion(std::string& polyscope_version);

  /*!
   * \brief Get Robot model
   *
   * \param robot_model The string for the robot model returned
   *
   * \return True succeeded
   */
  bool commandGetRobotModel(std::string& robot_model);

  /*!
   * \brief Get Serial number
   *
   * \param serial_number The serial number of the robot returned
   *
   * \return True succeeded
   */
  bool commandGetSerialNumber(std::string& serial_number);

  /*!
   * \brief Get Robot mode
   *
   * \param robot_mode The mode of the robot returned
   *
   * \return True succeeded
   */
  bool commandRobotMode(std::string& robot_mode);

  /*!
   * \brief Get Loaded Program
   *
   * \param loaded_program The path to the loaded program
   *
   * \return True succeeded
   */
  bool commandGetLoadedProgram(std::string& loaded_program);

  /*!
   * \brief Get Safety mode
   *
   * \param safety_mode The safety mode of the robot returned
   *
   * \return True succeeded
   */
  bool commandSafetyMode(std::string& safety_mode);

  /*!
   * \brief Get Safety status
   *
   * \param safety_status The safety status of the robot returned
   *
   * \return True succeeded
   */
  bool commandSafetyStatus(std::string& safety_status);

  /*!
   * \brief Get Program state
   *
   * \param program_state The program state of the robot returned
   *
   * \return True succeeded
   */
  bool commandProgramState(std::string& program_state);

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
   * \brief Send Clear operational mode command
   *
   * \throws an UrException when called on CB3 robots
   *
   * \return True succeeded
   */
  bool commandClearOperationalMode();

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
   * \brief Send Generate flight report command
   *
   * \param report_type The report type to set for the flight report
   *
   * \return True succeeded
   */
  bool commandGenerateFlightReport(const std::string& report_type);

  /*!
   * \brief Send Generate support file command
   *
   * \param dir_path The path to the directory of an already existing directory location inside the programs directory,
   * where the support file is saved
   *
   * \return True succeeded
   */
  bool commandGenerateSupportFile(const std::string& dir_path);

  /*!
   * \brief Flush the polyscope log to the log_history.txt file
   *
   * \return True succeeded
   */
  bool commandSaveLog();

private:
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
  bool send(const std::string& text);
  std::string read();
  void rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ");

  /*!
   * \brief Gets the configured receive timeout. If receive timeout is unconfigured "normal" socket timeout of 1 second
   * will be returned
   *
   * \returns configured receive timeout
   */
  timeval getConfiguredReceiveTimeout() const;

  VersionInformation polyscope_version_;
  std::string host_;
  int port_;
  std::mutex write_mutex_;
};
}  // namespace urcl
#endif  // ifndef UR_ROBOT_DRIVER_DASHBOARD_CLIENT_DASHBOARD_CLIENT_H_INCLUDED
