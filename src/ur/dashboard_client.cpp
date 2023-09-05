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

#include <iostream>
#include <regex>
#include <thread>
#include <unistd.h>
#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/exceptions.h>

using namespace std::chrono_literals;

namespace urcl
{
DashboardClient::DashboardClient(const std::string& host) : host_(host), port_(DASHBOARD_SERVER_PORT)
{
}

void DashboardClient::rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
}

bool DashboardClient::connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  if (getState() == comm::SocketState::Connected)
  {
    URCL_LOG_ERROR("%s", "Socket is already connected. Refusing to reconnect.");
    return false;
  }
  bool ret_val = false;

  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval tv;

  while (not ret_val)
  {
    // The first read after connection can take more time.
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    TCPSocket::setReceiveTimeout(tv);
    try
    {
      if (TCPSocket::setup(host_, port_, max_num_tries, reconnection_time))
      {
        URCL_LOG_INFO("%s", read().c_str());
        ret_val = true;
      }
      else
      {
        return false;
      }
    }
    catch (const TimeoutException&)
    {
      URCL_LOG_WARN("Did not receive dashboard bootup message although connection was established. This should not "
                    "happen, please contact the package maintainers. Retrying anyway...");
    }
  }

  // Reset read timeout to configured socket timeout
  TCPSocket::setReceiveTimeout(configured_tv);

  std::string pv;
  commandPolyscopeVersion(pv);

  return ret_val;
}

void DashboardClient::disconnect()
{
  URCL_LOG_INFO("Disconnecting from Dashboard server on %s:%d", host_.c_str(), port_);
  TCPSocket::close();
}

bool DashboardClient::send(const std::string& text)
{
  size_t len = text.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(text.c_str());
  size_t written;
  return TCPSocket::write(data, len, written);
}

std::string DashboardClient::read()
{
  std::stringstream result;
  char character;
  size_t read_chars = 99;
  while (read_chars > 0)
  {
    if (!TCPSocket::read((uint8_t*)&character, 1, read_chars))
    {
      disconnect();
      throw TimeoutException("Did not receive answer from dashboard server in time. Disconnecting from dashboard "
                             "server.",
                             *recv_timeout_);
    }
    result << character;
    if (character == '\n')
    {
      break;
    }
  }
  return result.str();
}

std::string DashboardClient::sendAndReceive(const std::string& text)
{
  std::string command = text;
  if (text.back() != '\n')
  {
    command = text + "\n";
  }
  std::string response = "ERROR";
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (send(command))
  {
    response = read();
  }
  else
  {
    throw UrException("Failed to send request to dashboard server. Are you connected to the Dashboard Server?");
  }
  rtrim(response);

  return response;
}

bool DashboardClient::sendRequest(const std::string& command, const std::string& expected)
{
  URCL_LOG_DEBUG("Send Request: %s", command.c_str());
  std::string response = sendAndReceive(command);
  bool ret = std::regex_match(response, std::regex(expected));
  if (!ret)
  {
    throw UrException("Expected: " + expected + ", but received: " + response);
  }
  return ret;
}

std::string DashboardClient::sendRequestString(const std::string& command, const std::string& expected)
{
  URCL_LOG_DEBUG("Send Request: %s", command.c_str());
  std::string response = sendAndReceive(command);
  bool ret = std::regex_match(response, std::regex(expected));
  if (!ret)
  {
    throw UrException("Expected: " + expected + ", but received: " + response);
  }
  return response;
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
    sendRequest(requestCommand, requestExpectedResponse);
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
  assertVersion("5.0.0", "3.0", "power off");
  return sendRequest("power off", "Powering off") && waitForReply("robotmode", "Robotmode: POWER_OFF");
}

bool DashboardClient::commandPowerOn(const std::chrono::duration<double> timeout)
{
  assertVersion("5.0.0", "3.0", "power on");
  return retryCommand("power on", "Powering on", "robotmode", "Robotmode: IDLE", timeout);
}

bool DashboardClient::commandBrakeRelease()
{
  assertVersion("5.0.0", "3.0", "brake release");
  return sendRequest("brake release", "Brake releasing") && waitForReply("robotmode", "Robotmode: RUNNING");
}

bool DashboardClient::commandLoadProgram(const std::string& program_file_name)
{
  assertVersion("5.0.0", "1.4", "load <program>");
  return sendRequest("load " + program_file_name + "", "(?:Loading program: ).*(?:" + program_file_name + ").*") &&
         waitForReply("programState", "STOPPED " + program_file_name);
}

bool DashboardClient::commandLoadInstallation(const std::string& installation_file_name)
{
  assertVersion("5.0.0", "3.2", "load installation");
  return sendRequest("load installation " + installation_file_name,
                     "(?:Loading installation: ).*(?:" + installation_file_name + ").*");
}

bool DashboardClient::commandPlay()
{
  assertVersion("5.0.0", "1.4", "play");
  return sendRequest("play", "Starting program") && waitForReply("programState", "(?:PLAYING ).*");
}

bool DashboardClient::commandPause()
{
  assertVersion("5.0.0", "1.4", "pause");
  return sendRequest("pause", "Pausing program") && waitForReply("programState", "(?:PAUSED ).*");
}

bool DashboardClient::commandStop()
{
  assertVersion("5.0.0", "1.4", "stop");
  return sendRequest("stop", "Stopped") && waitForReply("programState", "(?:STOPPED ).*");
}

bool DashboardClient::commandClosePopup()
{
  assertVersion("5.0.0", "1.6", "close popup");
  return sendRequest("close popup", "closing popup");
}

bool DashboardClient::commandCloseSafetyPopup()
{
  assertVersion("5.0.0", "3.1", "close safety popup");
  return sendRequest("close safety popup", "closing safety popup");
}

bool DashboardClient::commandRestartSafety()
{
  assertVersion("5.1.0", "3.7", "restart safety");
  return sendRequest("restart safety", "Restarting safety") && waitForReply("robotmode", "Robotmode: POWER_OFF");
}

bool DashboardClient::commandUnlockProtectiveStop()
{
  assertVersion("5.0.0", "3.1", "unlock protective stop");
  return sendRequest("unlock protective stop", "Protective stop releasing");
}

bool DashboardClient::commandShutdown()
{
  assertVersion("5.0.0", "1.4", "shutdown");
  return sendRequest("shutdown", "Shutting down");
}

bool DashboardClient::commandQuit()
{
  assertVersion("5.0.0", "1.4", "quit");
  return sendRequest("quit", "Disconnected");
}

bool DashboardClient::commandRunning()
{
  assertVersion("5.0.0", "1.6", "running");
  return sendRequest("running", "Program running: true");
}

bool DashboardClient::commandIsProgramSaved()
{
  assertVersion("5.0.0", "1.8", "isProgramSaved");
  return sendRequest("isProgramSaved", "(?:true ).*");
}

bool DashboardClient::commandIsInRemoteControl()
{
  assertVersion("5.6.0", "-", "is in remote control");
  std::string response = sendAndReceive("is in remote control");
  bool ret = std::regex_match(response, std::regex("true"));
  return ret;
}

bool DashboardClient::commandPopup(const std::string& popup_text)
{
  assertVersion("5.0.0", "1.6", "popup");
  return sendRequest("popup " + popup_text, "showing popup");
}

bool DashboardClient::commandAddToLog(const std::string& log_text)
{
  assertVersion("5.0.0", "1.8", "addToLog");
  return sendRequest("addToLog " + log_text, "Added log message");
}

bool DashboardClient::commandPolyscopeVersion(std::string& polyscope_version)
{
  std::string expected = "(?:URSoftware ).*";
  polyscope_version = sendRequestString("PolyscopeVersion", expected);
  std::string version_string = polyscope_version.substr(polyscope_version.find(" ") + 1,
                                                        polyscope_version.find(" (") - polyscope_version.find(" ") - 1);
  polyscope_version_ = VersionInformation::fromString(version_string);
  return std::regex_match(polyscope_version, std::regex(expected));
}

bool DashboardClient::commandGetRobotModel(std::string& robot_model)
{
  assertVersion("5.6.0", "3.12", "get robot model");
  std::string expected = "(?:UR).*";
  robot_model = sendRequestString("get robot model", expected);
  return std::regex_match(robot_model, std::regex(expected));
}

bool DashboardClient::commandGetSerialNumber(std::string& serial_number)
{
  assertVersion("5.6.0", "3.12", "get serial number");
  std::string expected = "(?:20).*";
  serial_number = sendRequestString("get serial number", expected);
  return std::regex_match(serial_number, std::regex(expected));
}

bool DashboardClient::commandRobotMode(std::string& robot_mode)
{
  assertVersion("5.0.0", "1.6", "robotmode");
  std::string expected = "(?:Robotmode: ).*";
  robot_mode = sendRequestString("robotmode", expected);
  return std::regex_match(robot_mode, std::regex(expected));
}

bool DashboardClient::commandGetLoadedProgram(std::string& loaded_program)
{
  assertVersion("5.0.0", "1.6", "get loaded program");
  std::string expected = "(?:Loaded program: ).*";
  loaded_program = sendRequestString("get loaded program", expected);
  return std::regex_match(loaded_program, std::regex(expected));
}

bool DashboardClient::commandSafetyMode(std::string& safety_mode)
{
  assertVersion("5.0.0", "3.0", "safetymode");
  std::string expected = "(?:Safetymode: ).*";
  safety_mode = sendRequestString("safetymode", expected);
  return std::regex_match(safety_mode, std::regex(expected));
}

bool DashboardClient::commandSafetyStatus(std::string& safety_status)
{
  assertVersion("5.4.0", "3.11", "safetystatus");
  std::string expected = "(?:Safetystatus: ).*";
  safety_status = sendRequestString("safetystatus", expected);
  return std::regex_match(safety_status, std::regex(expected));
}

bool DashboardClient::commandProgramState(std::string& program_state)
{
  assertVersion("5.0.0", "1.8", "programState");
  std::string expected = "(?:).*";
  program_state = sendRequestString("programState", expected);
  return !std::regex_match(program_state, std::regex("(?:could not understand).*"));
}

bool DashboardClient::commandGetOperationalMode(std::string& operational_mode)
{
  assertVersion("5.6.0", "-", "get operational mode");
  std::string expected = "(?:).*";
  operational_mode = sendRequestString("get operational mode", expected);
  return !std::regex_match(operational_mode, std::regex("(?:could not understand).*"));
}

bool DashboardClient::commandSetOperationalMode(const std::string& operational_mode)
{
  assertVersion("5.0.0", "-", "set operational mode");
  return sendRequest("set operational mode " + operational_mode,
                     "(?:Operational mode ).*(?:" + operational_mode + ").*");
}

bool DashboardClient::commandClearOperationalMode()
{
  assertVersion("5.0.0", "-", "clear operational mode");
  return sendRequest("clear operational mode", "(?:No longer controlling the operational mode. ).*");
}

bool DashboardClient::commandSetUserRole(const std::string& user_role)
{
  assertVersion("-", "1.8", "setUserRole");
  return sendRequest("setUserRole " + user_role, "(?:Setting user role: ).*");
}

bool DashboardClient::commandGetUserRole(std::string& user_role)
{
  assertVersion("-", "1.8", "getUserRole");
  std::string expected = "(?:).*";
  user_role = sendRequestString("getUserRole", expected);
  return !std::regex_match(user_role, std::regex("(?:could not understand).*"));
}

bool DashboardClient::commandGenerateFlightReport(const std::string& report_type)
{
  assertVersion("5.8.0", "3.13", "generate flight report");
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval tv;
  tv.tv_sec = 180;
  tv.tv_usec = 0;
  TCPSocket::setReceiveTimeout(tv);  // Set timeout to 3 minutes as this command can take a long time to complete
  bool ret = sendRequest("generate flight report " + report_type, "(?:Flight Report generated with id:).*");
  TCPSocket::setReceiveTimeout(configured_tv);  // Reset to configured receive timeout
  return ret;
}

bool DashboardClient::commandGenerateSupportFile(const std::string& dir_path)
{
  assertVersion("5.8.0", "3.13", "generate support file");
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval tv;
  tv.tv_sec = 600;
  tv.tv_usec = 0;
  TCPSocket::setReceiveTimeout(tv);  // Set timeout to 10 minutes as this command can take a long time to complete
  bool ret = sendRequest("generate support file " + dir_path, "(?:Completed successfully:).*");
  TCPSocket::setReceiveTimeout(configured_tv);  // Reset to configured receive timeout
  return ret;
}

bool DashboardClient::commandSaveLog()
{
  assertVersion("5.0.0", "1.8", "save log");
  return sendRequest("saveLog", "Log saved to disk");
}

void DashboardClient::assertVersion(const std::string& e_series_min_ver, const std::string& cb3_min_ver,
                                    const std::string& required_call)
{
  if (!polyscope_version_.isESeries() && cb3_min_ver == "-")
  {
    std::stringstream ss;
    ss << "The dasboard call '" << required_call
       << "' is only available on e-series robots, but you seem to be running version " << polyscope_version_;
    throw UrException(ss.str());
  }

  if (polyscope_version_.isESeries() && e_series_min_ver == "-")
  {
    std::stringstream ss;
    ss << "The dasboard call '" << required_call
       << "' is only available on pre-e-series robots (5.x.y), but you seem to be running version "
       << polyscope_version_;
    throw UrException(ss.str());
  }

  auto ref = polyscope_version_.isESeries() ? VersionInformation::fromString(e_series_min_ver) :
                                              VersionInformation::fromString(cb3_min_ver);
  if (ref > polyscope_version_)
  {
    std::stringstream ss;
    ss << "Polyscope version " << polyscope_version_ << " isn't recent enough to use dashboard call '" << required_call
       << "'";
    throw UrException(ss.str());
  }
}

timeval DashboardClient::getConfiguredReceiveTimeout() const
{
  timeval tv;
  if (recv_timeout_ != nullptr)
  {
    tv = *recv_timeout_.get();
  }
  else
  {
    tv.tv_sec = 1;
    tv.tv_usec = 0;
  }
  return tv;
}

}  // namespace urcl
