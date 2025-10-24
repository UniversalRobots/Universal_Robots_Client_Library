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

#include <ur_client_library/ur/dashboard_client_implementation.h>
#include <unordered_map>
#include "ur_client_library/ur/version_information.h"

namespace httplib
{
class Client;
}

namespace urcl
{

class DashboardClientImplX : public DashboardClientImpl
{
public:
  DashboardClientImplX() = delete;
  DashboardClientImplX(const std::string& host);

  ~DashboardClientImplX() override;

  static constexpr int DASHBOARD_SERVER_PORT = 29999;

  /*!
   * \brief Sends a command through the socket and waits for an answer.
   *
   * \param command Command that will be sent to the server.
   *
   * \throws UrException if no response was read from the dashboard server
   *
   * \returns Answer as received by the server cut off any trailing newlines.
   */
  std::string sendAndReceive(const std::string& command) override;

  /*!
   * \brief Checks whether there is a Robot API endpoint at the configured IP address
   *
   * To determine this, this function makes a test call to the "/system/v1/system-time/" endpoint.
   * If the given IP address doesn't correspond to a robot with the minimal required software
   * version, this call will fail.
   *
   * Note: Calling this isn't required for making dashboard calls for a PolyScope X robot.
   *
   * \returns True on successful test call
   */
  bool connect(const size_t max_num_tries = 0,
               const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10)) override;

  /*!
   * \brief This function call is effectively not doing anything.
   */
  void disconnect() override;

  /*!
   * \brief This function call is effectively not doing anything.
   */
  timeval getConfiguredReceiveTimeout() const override;

  bool sendRequest(const std::string& command, const std::string& expected_response_pattern = "",
                   const std::string& payload = "") override;
  std::string sendRequestString(const std::string& command, const std::string& expected = "",
                                const std::string& payload = "") override;
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

  DashboardResponse commandAddToLog(const std::string& log_text) override;
  DashboardResponse commandBrakeRelease() override;
  DashboardResponse commandClearOperationalMode() override;
  DashboardResponse commandClosePopup() override;
  DashboardResponse commandCloseSafetyPopup() override;
  DashboardResponse commandGenerateFlightReport(const std::string& report_type) override;
  DashboardResponse commandGenerateSupportFile(const std::string& dir_path) override;
  DashboardResponse commandGetLoadedProgram() override;
  DashboardResponse commandGetOperationalMode() override;
  DashboardResponse commandGetRobotModel() override;
  DashboardResponse commandGetSerialNumber() override;
  DashboardResponse commandGetUserRole() override;
  DashboardResponse commandIsInRemoteControl() override;
  DashboardResponse commandIsProgramSaved() override;
  DashboardResponse commandLoadInstallation(const std::string& installation_file_name) override;
  DashboardResponse commandLoadProgram(const std::string& program_file_name) override;
  DashboardResponse commandPause() override;
  DashboardResponse commandResume() override;
  DashboardResponse commandPlay() override;
  DashboardResponse commandPolyscopeVersion() override;
  DashboardResponse commandPopup(const std::string& popup_text) override;
  DashboardResponse commandPowerOff() override;
  DashboardResponse commandPowerOn(const std::chrono::duration<double> timeout = std::chrono::seconds(300)) override;
  DashboardResponse commandProgramState() override;
  DashboardResponse commandQuit() override;
  DashboardResponse commandRestartSafety() override;
  DashboardResponse commandRobotMode() override;
  DashboardResponse commandRunning() override;
  DashboardResponse commandSafetyMode() override;
  DashboardResponse commandSafetyStatus() override;
  DashboardResponse commandSaveLog() override;
  DashboardResponse commandSetOperationalMode(const std::string& operational_mode) override;
  DashboardResponse commandSetUserRole(const std::string& user_role) override;
  DashboardResponse commandShutdown() override;
  DashboardResponse commandStop() override;
  DashboardResponse commandUnlockProtectiveStop() override;

  void setReceiveTimeout(const timeval& timeout) override
  {
  }

protected:
  DashboardResponse put(const std::string& endpoint, const std::string& json_data);
  DashboardResponse get(const std::string& endpoint);
  virtual VersionInformation queryPolyScopeVersion();
  void assertHasCommand(const std::string& command) const override;

  const std::string base_url_ = "/universal-robots/robot-api";

  std::unique_ptr<httplib::Client> cli_;
};

}  // namespace urcl
