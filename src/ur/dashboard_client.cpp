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

#include <filesystem>
#include <iostream>
#include <regex>
#include <thread>
#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/exceptions.h>

#include <ur_client_library/ur/dashboard_client_implementation_g5.h>
#include <ur_client_library/ur/dashboard_client_implementation_x.h>

#ifndef _WIN32
#  include <unistd.h>
#endif  // !_WIN32

using namespace std::chrono_literals;

namespace urcl
{
DashboardClient::DashboardClient(const std::string& host, const ClientPolicy client_policy)
{
  if (client_policy == ClientPolicy::G5)
  {
    impl_ = std::make_shared<DashboardClientImplG5>(host);
  }
  else if (client_policy == ClientPolicy::POLYSCOPE_X)
  {
    impl_ = std::make_shared<DashboardClientImplX>(host);
  }
  else
  {
    throw UrException("Unknown DashboardClient policy passed to DashboardClient.");
  }
  URCL_LOG_INFO("DashboardClient created for host %s", host.c_str());
}

bool DashboardClient::connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  bool success = impl_->connect(max_num_tries, reconnection_time);
  return success;
}

void DashboardClient::disconnect()
{
  impl_->disconnect();
}

std::string DashboardClient::sendAndReceive(const std::string& text)
{
  return impl_->sendAndReceive(text);
}

bool DashboardClient::sendRequest(const std::string& command, const std::string& expected)
{
  return impl_->sendRequest(command, expected);
}

std::string DashboardClient::sendRequestString(const std::string& command, const std::string& expected)
{
  return impl_->sendRequestString(command, expected);
}

bool DashboardClient::waitForReply(const std::string& command, const std::string& expected,
                                   const std::chrono::duration<double> timeout)
{
  const std::chrono::duration<double> wait_period = 100ms;

  std::chrono::duration<double> time_done(0);
  std::string response;

  while (time_done < timeout)
  {
    // Send the request
    response = sendAndReceive(command);

    // Check if the response was as expected
    if (std::regex_match(response, std::regex(expected)))
    {
      return true;
    }

    // wait 100ms before trying again
    std::this_thread::sleep_for(wait_period);
    time_done += wait_period;
  }

  URCL_LOG_WARN("Did not got the expected \"%s\" response within the timeout. Last response was: \"%s\"",
                expected.c_str(), response.c_str());  // Is a warning here so retryCommand does not throw when retrying
  return false;
}

bool DashboardClient::retryCommand(const std::string& requestCommand, const std::string& requestExpectedResponse,
                                   const std::string& waitRequest, const std::string& waitExpectedResponse,
                                   const std::chrono::duration<double> timeout,
                                   const std::chrono::duration<double> retry_period)
{
  std::chrono::duration<double> time_done(0);
  do
  {
    impl_->sendRequest(requestCommand);
    time_done += retry_period;

    if (waitForReply(waitRequest, waitExpectedResponse, retry_period))
    {
      return true;
    }
  } while (time_done < timeout);
  return false;
}

bool DashboardClient::commandPowerOff()
{
  return commandPowerOffWithResponse().ok;
}

DashboardResponse DashboardClient::commandPowerOffWithResponse()
{
  return impl_->commandPowerOff();
}

bool DashboardClient::commandPowerOn(const std::chrono::duration<double> timeout)
{
  return commandPowerOnWithResponse(timeout).ok;
}

DashboardResponse DashboardClient::commandPowerOnWithResponse(const std::chrono::duration<double> timeout)
{
  return impl_->commandPowerOn(timeout);
}

bool DashboardClient::commandBrakeRelease()
{
  return commandBrakeReleaseWithResponse().ok;
}

DashboardResponse DashboardClient::commandBrakeReleaseWithResponse()
{
  return impl_->commandBrakeRelease();
}

bool DashboardClient::commandLoadProgram(const std::string& program_file_name)
{
  return commandLoadProgramWithResponse(program_file_name).ok;
}

DashboardResponse DashboardClient::commandLoadProgramWithResponse(const std::string& program_file_name)
{
  return impl_->commandLoadProgram(program_file_name);
}

bool DashboardClient::commandLoadInstallation(const std::string& installation_file_name)
{
  return commandLoadInstallationWithResponse(installation_file_name).ok;
}

DashboardResponse DashboardClient::commandLoadInstallationWithResponse(const std::string& installation_file_name)
{
  return impl_->commandLoadInstallation(installation_file_name);
}

bool DashboardClient::commandPlay()
{
  return commandPlayWithResponse().ok;
}
DashboardResponse DashboardClient::commandPlayWithResponse()
{
  return impl_->commandPlay();
}

bool DashboardClient::commandPause()
{
  return commandPauseWithResponse().ok;
}

DashboardResponse DashboardClient::commandPauseWithResponse()
{
  return impl_->commandPause();
}

bool DashboardClient::commandResume()
{
  return commandResumeWithResponse().ok;
}

DashboardResponse DashboardClient::commandResumeWithResponse()
{
  return impl_->commandResume();
}

bool DashboardClient::commandStop()
{
  return commandStopWithResponse().ok;
}

DashboardResponse DashboardClient::commandStopWithResponse()
{
  return impl_->commandStop();
}

bool DashboardClient::commandClosePopup()
{
  return commandClosePopupWithResponse().ok;
}

DashboardResponse DashboardClient::commandClosePopupWithResponse()
{
  return impl_->commandClosePopup();
}

bool DashboardClient::commandCloseSafetyPopup()
{
  return commandCloseSafetyPopupWithResponse().ok;
}

DashboardResponse DashboardClient::commandCloseSafetyPopupWithResponse()
{
  return impl_->commandCloseSafetyPopup();
}

bool DashboardClient::commandRestartSafety()
{
  return commandRestartSafetyWithResponse().ok;
}

DashboardResponse DashboardClient::commandRestartSafetyWithResponse()
{
  return impl_->commandRestartSafety();
}

bool DashboardClient::commandUnlockProtectiveStop()
{
  return commandUnlockProtectiveStopWithResponse().ok;
}

DashboardResponse DashboardClient::commandUnlockProtectiveStopWithResponse()
{
  return impl_->commandUnlockProtectiveStop();
}

bool DashboardClient::commandShutdown()
{
  return commandShutdownWithResponse().ok;
}

DashboardResponse DashboardClient::commandShutdownWithResponse()
{
  return impl_->commandShutdown();
}

bool DashboardClient::commandQuit()
{
  return commandQuitWithResponse().ok;
}

DashboardResponse DashboardClient::commandQuitWithResponse()
{
  return impl_->commandQuit();
}

bool DashboardClient::commandRunning()
{
  auto response = commandRunningWithResponse();
  return response.ok && std::get<bool>(response.data["running"]);
}

DashboardResponse DashboardClient::commandRunningWithResponse()
{
  return impl_->commandRunning();
}

bool DashboardClient::commandIsProgramSaved()
{
  auto response = impl_->commandIsProgramSaved();
  return response.ok && std::get<bool>(response.data["saved"]);
}

DashboardResponse DashboardClient::commandIsProgramSavedWithResponse()
{
  return impl_->commandIsProgramSaved();
}

bool DashboardClient::commandIsInRemoteControl()
{
  auto response = impl_->commandIsInRemoteControl();
  return response.ok && std::get<bool>(response.data["remote_control"]);
}

DashboardResponse DashboardClient::commandIsInRemoteControlWithResponse()
{
  return impl_->commandIsInRemoteControl();
}

bool DashboardClient::commandPopup(const std::string& popup_text)
{
  return commandPopupWithResponse(popup_text).ok;
}

DashboardResponse DashboardClient::commandPopupWithResponse(const std::string& popup_text)
{
  return impl_->commandPopup(popup_text);
}

bool DashboardClient::commandAddToLog(const std::string& log_text)
{
  return commandAddToLogWithResponse(log_text).ok;
}

DashboardResponse DashboardClient::commandAddToLogWithResponse(const std::string& log_text)
{
  return impl_->commandAddToLog(log_text);
}

bool DashboardClient::commandPolyscopeVersion(std::string& polyscope_version)
{
  DashboardResponse response = impl_->commandPolyscopeVersion();
  if (response.ok)
  {
    polyscope_version = std::get<std::string>(response.data["polyscope_version"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandPolyscopeVersionWithResponse()
{
  return impl_->commandPolyscopeVersion();
}

bool DashboardClient::commandGetRobotModel(std::string& robot_model)
{
  DashboardResponse response = impl_->commandGetRobotModel();
  if (response.ok)
  {
    robot_model = std::get<std::string>(response.data["robot_model"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandGetRobotModelWithResponse()
{
  return impl_->commandGetRobotModel();
}

bool DashboardClient::commandGetSerialNumber(std::string& serial_number)
{
  DashboardResponse response = impl_->commandGetSerialNumber();
  if (response.ok)
  {
    serial_number = std::get<std::string>(response.data["serial_number"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandGetSerialNumberWithResponse()
{
  return impl_->commandGetSerialNumber();
}

bool DashboardClient::commandRobotMode(std::string& robot_mode)
{
  DashboardResponse response = impl_->commandRobotMode();
  if (response.ok)
  {
    robot_mode = std::get<std::string>(response.data["robot_mode"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandRobotModeWithResponse()
{
  return impl_->commandRobotMode();
}

bool DashboardClient::commandGetLoadedProgram(std::string& loaded_program)
{
  DashboardResponse response = impl_->commandGetLoadedProgram();
  if (response.ok)
  {
    loaded_program = std::get<std::string>(response.data["program_name"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandGetLoadedProgramWithResponse()
{
  return impl_->commandGetLoadedProgram();
}

bool DashboardClient::commandSafetyMode(std::string& safety_mode)
{
  DashboardResponse response = impl_->commandSafetyMode();
  if (response.ok)
  {
    safety_mode = std::get<std::string>(response.data["safety_mode"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandSafetyModeWithResponse()
{
  return impl_->commandSafetyMode();
}

bool DashboardClient::commandSafetyStatus(std::string& safety_status)
{
  DashboardResponse response = impl_->commandSafetyStatus();
  if (response.ok)
  {
    safety_status = std::get<std::string>(response.data["safety_status"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandSafetyStatusWithResponse()
{
  return impl_->commandSafetyStatus();
}

bool DashboardClient::commandProgramState(std::string& program_state)
{
  DashboardResponse response = impl_->commandProgramState();
  if (response.ok)
  {
    program_state = std::get<std::string>(response.data["program_state"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandProgramStateWithResponse()
{
  return impl_->commandProgramState();
}

bool DashboardClient::commandGetOperationalMode(std::string& operational_mode)
{
  DashboardResponse response = impl_->commandGetOperationalMode();
  if (response.ok)
  {
    operational_mode = std::get<std::string>(response.data["operational_mode"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandGetOperationalModeWithResponse()
{
  return impl_->commandGetOperationalMode();
}

bool DashboardClient::commandSetOperationalMode(const std::string& operational_mode)
{
  return commandSetOperationalModeWithResponse(operational_mode).ok;
}

DashboardResponse DashboardClient::commandSetOperationalModeWithResponse(const std::string& operational_mode)
{
  return impl_->commandSetOperationalMode(operational_mode);
}

bool DashboardClient::commandClearOperationalMode()
{
  return commandClearOperationalModeWithResponse().ok;
}
DashboardResponse DashboardClient::commandClearOperationalModeWithResponse()
{
  return impl_->commandClearOperationalMode();
}

bool DashboardClient::commandSetUserRole(const std::string& user_role)
{
  return commandSetUserRoleWithResponse(user_role).ok;
}

DashboardResponse DashboardClient::commandSetUserRoleWithResponse(const std::string& user_role)
{
  return impl_->commandSetUserRole(user_role);
}

bool DashboardClient::commandGetUserRole(std::string& user_role)
{
  DashboardResponse response = impl_->commandGetUserRole();
  if (response.ok)
  {
    user_role = std::get<std::string>(response.data["user_role"]);
  }
  return response.ok;
}

DashboardResponse DashboardClient::commandGetUserRoleWithResponse()
{
  return impl_->commandGetUserRole();
}

bool DashboardClient::commandGenerateFlightReport(const std::string& report_type)
{
  return commandGenerateFlightReportWithResponse(report_type).ok;
}

DashboardResponse DashboardClient::commandGenerateFlightReportWithResponse(const std::string& report_type)
{
  return impl_->commandGenerateFlightReport(report_type);
}

bool DashboardClient::commandGenerateSupportFile(const std::string& dir_path)
{
  return commandGenerateSupportFileWithResponse(dir_path).ok;
}

DashboardResponse DashboardClient::commandGenerateSupportFileWithResponse(const std::string& dir_path)
{
  return impl_->commandGenerateSupportFile(dir_path);
}

bool DashboardClient::commandSaveLog()
{
  return commandSaveLogWithResponse().ok;
}

DashboardResponse DashboardClient::commandSaveLogWithResponse()
{
  return impl_->commandSaveLog();
}

void DashboardClient::assertVersion(const std::string& e_series_min_ver, const std::string& cb3_min_ver,
                                    const std::string& required_call)
{
  URCL_LOG_DEBUG("Asserting version for dashboard call '%s' with robot version %s", required_call.c_str(),
                 impl_->getPolyscopeVersion().toString().c_str());
  if (!impl_->getPolyscopeVersion().isESeries() && cb3_min_ver == "-")
  {
    std::stringstream ss;
    ss << "The dasboard call '" << required_call
       << "' is only available on e-series robots, but you seem to be running version " << impl_->getPolyscopeVersion();
    throw UrException(ss.str());
  }

  if (impl_->getPolyscopeVersion().isESeries() && e_series_min_ver == "-")
  {
    std::stringstream ss;
    ss << "The dasboard call '" << required_call
       << "' is only available on pre-e-series robots (5.x.y), but you seem to be running version "
       << impl_->getPolyscopeVersion();
    throw UrException(ss.str());
  }

  auto ref = impl_->getPolyscopeVersion().isESeries() ? VersionInformation::fromString(e_series_min_ver) :
                                                        VersionInformation::fromString(cb3_min_ver);
  if (ref > impl_->getPolyscopeVersion())
  {
    std::stringstream ss;
    ss << "Polyscope version " << impl_->getPolyscopeVersion() << " isn't recent enough to use dashboard call '"
       << required_call << "'";
    throw UrException(ss.str());
  }
}

timeval DashboardClient::getConfiguredReceiveTimeout() const
{
  return impl_->getConfiguredReceiveTimeout();
}

void DashboardClient::setReceiveTimeout(const timeval& timeout)
{
  impl_->setReceiveTimeout(timeout);
}
}  // namespace urcl
