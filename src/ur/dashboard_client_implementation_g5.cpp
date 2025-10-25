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

#include <filesystem>
#include <regex>
#include <sstream>
#include <thread>

#include "ur_client_library/helpers.h"
#include "ur_client_library/ur/version_information.h"
#include <ur_client_library/exceptions.h>
#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client_implementation_g5.h>

using namespace std::chrono_literals;

namespace urcl
{
std::unordered_map<std::string, DashboardCommandG5> DashboardClientImplG5::g_command_list{
  { "load",
    { "load <program.urp>", "(?:Loading program: ).*(?: <program.urp> ).*", VersionInformation::fromString("1.4"),
      VersionInformation::fromString("5.0.0") } },
  { "play",
    { "play", "Starting program", VersionInformation::fromString("1.4"), VersionInformation::fromString("5.0.0") } },
  { "stop", { "stop", "Stopped", VersionInformation::fromString("1.4"), VersionInformation::fromString("5.0.0") } },
  { "pause",
    { "pause", "Pausing program", VersionInformation::fromString("1.4"), VersionInformation::fromString("5.0.0") } },
  { "quit",
    { "quit", "Disconnected", VersionInformation::fromString("1.4"), VersionInformation::fromString("5.0.0") } },
  { "shutdown",
    { "shutdown", "Shutting down", VersionInformation::fromString("1.4"), VersionInformation::fromString("5.0.0") } },
  { "unlock protective stop",
    { "unlock protective stop", "Protective stop releasing", VersionInformation::fromString("3.1"),
      VersionInformation::fromString("5.0.0") } },
  { "running",
    { "running", "(?:Program running: ).*", VersionInformation::fromString("1.6"),
      VersionInformation::fromString("5.0.0") } },
  { "robotmode",
    { "robotmode", "(?:Robotmode: ).*", VersionInformation::fromString("1.6"),
      VersionInformation::fromString("5.0.0") } },
  { "get loaded program",
    { "get loaded program", "(?:Loaded program: ).*", VersionInformation::fromString("1.6"),
      VersionInformation::fromString("5.0.0") } },
  { "popup",
    { "popup <popup_text>", "showing popup", VersionInformation::fromString("1.6"),
      VersionInformation::fromString("5.0.0") } },
  { "close popup",
    { "close popup", "closing popup", VersionInformation::fromString("1.6"),
      VersionInformation::fromString("5.0.0") } },
  { "addToLog",
    { "addToLog <log_text>", "Added log message", VersionInformation::fromString("1.8"),
      VersionInformation::fromString("5.0.0") } },
  { "isProgramSaved",
    { "isProgramSaved", "(?:true|false).*", VersionInformation::fromString("1.8"),
      VersionInformation::fromString("5.0.0") } },
  { "programState",
    { "programState", "(STOPPED|PLAYING|PAUSED) .*", VersionInformation::fromString("1.8"),
      VersionInformation::fromString("5.0.0") } },
  { "PolyscopeVersion",
    { "PolyscopeVersion", "(?:URSoftware ).*", VersionInformation::fromString("1.8"),
      VersionInformation::fromString("5.0.0") } },
  { "version",
    { "version", "(\\d\\.\\d\\.\\d\\.\\d)", VersionInformation::fromString("5.0.0"),
      VersionInformation::fromString("5.0.0") } },
  { "setUserRole",
    { "setUserRole <role>", "Setting user role: <role>", VersionInformation::fromString("3.1.0"),
      VersionInformation::fromString("5.99.0") } },
  { "getUserRole",
    { "getUserRole", "(PROGRAMMER|OPERATOR|NONE|LOCKED|RESTRICTED)", VersionInformation::fromString("3.1.0"),
      VersionInformation::fromString("5.99.0") } },
  { "set operational mode",
    { "set operational mode <operational_mode>", "Operational mode (MANUAL|AUTOMATIC) is set",
      VersionInformation::fromString("3.99.0"), VersionInformation::fromString("5.0.0") } },
  { "get operational mode",
    { "get operational mode", "(NONE|MANUAL|AUTOMATIC)", VersionInformation::fromString("5.6.0"),
      VersionInformation::fromString("5.6.0") } },
  { "clear operational mode",
    { "clear operational mode", "(?:No longer controlling the operational mode.).*",
      VersionInformation::fromString("5.0.0"), VersionInformation::fromString("5.0.0") } },
  { "power on",
    { "power on", "Powering on", VersionInformation::fromString("3.0.0"), VersionInformation::fromString("5.0.0") } },
  { "power off",
    { "power off", "Powering off", VersionInformation::fromString("3.0.0"), VersionInformation::fromString("5.0.0") } },
  { "brake release",
    { "brake release", "Brake releasing", VersionInformation::fromString("3.0.0"),
      VersionInformation::fromString("5.0.0") } },
  { "safetymode",
    { "safetymode", "(?:Safetymode: ).*", VersionInformation::fromString("3.0"),
      VersionInformation::fromString("5.0.0") } },
  { "safetystatus",
    { "safetystatus", "(?:Safetystatus: ).*", VersionInformation::fromString("5.4.0"),
      VersionInformation::fromString("5.4.0") } },
  { "unlock protective stop",
    { "unlock protective stop", "Protective stop releasing", VersionInformation::fromString("3.1"),
      VersionInformation::fromString("5.0.0") } },
  { "close safety popup",
    { "close safety popup", "closing safety popup", VersionInformation::fromString("3.1"),
      VersionInformation::fromString("5.0.0") } },
  { "load installation",
    { "load installation <default.installation>", "(?:Loading installation: ).*(?: <default.installation> ).*",
      VersionInformation::fromString("3.2"), VersionInformation::fromString("5.0.0") } },
  { "restart safety",
    { "restart safety", "Restarting safety", VersionInformation::fromString("3.7"),
      VersionInformation::fromString("5.1.0") } },
  { "is in remote control",
    { "is in remote control", "(?:true|false)", VersionInformation::fromString("5.6.0"),
      VersionInformation::fromString("5.6.0") } },
  { "get serial number",
    { "get serial number", "(?:20).*", VersionInformation::fromString("3.12"),
      VersionInformation::fromString("5.6.0") } },
  { "get robot model",
    { "get robot model", "(?:UR).*", VersionInformation::fromString("3.12"),
      VersionInformation::fromString("5.6.0") } },
  { "generate flight report",
    { "generate flight report <report_type>", "(?:Generating flight report: ).*(?: <report_type> ).*",
      VersionInformation::fromString("3.13"), VersionInformation::fromString("5.8.0") } },
  { "generate support file",
    { "generate support file <dir_path>", "(?:Generating support file: ).*(?: <dir_path> ).*",
      VersionInformation::fromString("3.13"), VersionInformation::fromString("5.8.0") } },
  { "saveLog",
    { "saveLog", "Log saved to disk", VersionInformation::fromString("3.13"),
      VersionInformation::fromString("5.8.0") } },
};

DashboardClientImplG5::DashboardClientImplG5(const std::string& host) : DashboardClientImpl(host)
{
}
bool DashboardClientImplG5::send(const std::string& text)
{
  size_t len = text.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(text.c_str());
  size_t written;
  return TCPSocket::write(data, len, written);
}

std::string DashboardClientImplG5::read()
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

std::string DashboardClientImplG5::sendAndReceive(const std::string& text)
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

bool DashboardClientImplG5::connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  if (getState() == comm::SocketState::Connected)
  {
    URCL_LOG_ERROR("%s", "Socket is already connected. Refusing to reconnect.");
    return false;
  }
  bool ret_val = false;

  timeval configured_tv = getConfiguredReceiveTimeout();

  while (!ret_val)
  {
    timeval tv;
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

  polyscope_version_ = queryPolyScopeVersion();

  return ret_val;
}

void DashboardClientImplG5::disconnect()
{
  URCL_LOG_INFO("Disconnecting from Dashboard server on %s:%d", host_.c_str(), port_);
  TCPSocket::close();
}

void DashboardClientImplG5::rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
}

timeval DashboardClientImplG5::getConfiguredReceiveTimeout() const
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

VersionInformation DashboardClientImplG5::queryPolyScopeVersion()
{
  std::string response = sendAndReceive("PolyscopeVersion");
  std::string version_string = response.substr(response.find(" ") + 1, response.find(" (") - response.find(" ") - 1);
  return VersionInformation::fromString(version_string);
}

void DashboardClientImplG5::assertHasCommand(const std::string& command) const
{
  if (polyscope_version_ == VersionInformation::fromString("0.0.0"))
  {
    throw UrException("Polyscope version is not set. Has it been connected?");
  }
  auto it = g_command_list.find(command);
  if (it != g_command_list.end())
  {
    bool command_supported = false;
    if (polyscope_version_.isESeries())
    {
      command_supported = polyscope_version_ >= it->second.min_version_e_series;
    }
    else
    {
      command_supported = polyscope_version_ >= it->second.min_version_cb3;
    }
    if (!command_supported)
    {
      auto min_version = polyscope_version_.isESeries() ? it->second.min_version_e_series : it->second.min_version_cb3;
      throw VersionMismatch("Dashboard command '" + command + "' is not supported by this robot version.", min_version,
                            polyscope_version_);
    }
  }
  else
  {
    throw NotImplementedException("Dashboard command '" + command + "' is not known / implemented.");
  }
}

bool DashboardClientImplG5::sendRequest(const std::string& command_str, const std::string& expected_response_pattern,
                                        const std::string& payload)
{
  assertHasCommand(command_str);
  auto& command = g_command_list[command_str];
  std::string expected = expected_response_pattern.empty() ? command.response_pattern : expected_response_pattern;
  expected = replacePayload(expected, payload);
  std::string command_with_payload = replacePayload(command.command, payload);

  URCL_LOG_DEBUG("Send Request: %s", command_with_payload.c_str());
  std::string response = sendAndReceive(command_with_payload);
  URCL_LOG_DEBUG("Got Response: %s", response.c_str());
  bool ret = std::regex_match(response, std::regex(expected));
  if (!ret)
  {
    URCL_LOG_WARN("Expected: \"%s\", but received: \"%s\"", expected.c_str(), response.c_str());
  }
  return ret;
}

std::string DashboardClientImplG5::sendRequestString(const std::string& command_str,
                                                     const std::string& expected_response_pattern,
                                                     const std::string& payload)
{
  assertHasCommand(command_str);
  auto& command = g_command_list[command_str];
  std::string command_with_payload = replacePayload(command.command, payload);
  URCL_LOG_DEBUG("Send Request: %s", command_with_payload.c_str());
  std::string expected = expected_response_pattern.empty() ? command.response_pattern : expected_response_pattern;
  expected = replacePayload(expected, payload);
  std::string response = sendAndReceive(command_with_payload);
  bool ret = std::regex_match(response, std::regex(expected));
  if (!ret)
  {
    throw UnexpectedResponse("Expected: " + expected + ", but received: " + response);
  }
  return response;
}

bool DashboardClientImplG5::waitForReply(const std::string& command, const std::string& expected,
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

std::string DashboardClientImplG5::retryCommandString(const std::string& requestCommand,
                                                      const std::string& requestExpectedResponse,
                                                      const std::string& waitRequest,
                                                      const std::string& waitExpectedResponse,
                                                      const std::chrono::duration<double> timeout,
                                                      const std::chrono::duration<double> retry_period)
{
  std::chrono::duration<double> time_done(0);
  std::string response;
  do
  {
    response = sendRequestString(requestCommand, requestExpectedResponse);
    time_done += retry_period;

    if (waitForReply(waitRequest, waitExpectedResponse, retry_period))
    {
      return response;
    }
  } while (time_done < timeout);
  throw UrException("Failed to get expected response for command '" + requestCommand +
                    "' within the timeout. Last response was: " + response);
}

bool DashboardClientImplG5::retryCommand(const std::string& requestCommand, const std::string& requestExpectedResponse,
                                         const std::string& waitRequest, const std::string& waitExpectedResponse,
                                         const std::chrono::duration<double> timeout,
                                         const std::chrono::duration<double> retry_period)
{
  std::chrono::duration<double> time_done(0);
  do
  {
    sendRequest(requestCommand);
    time_done += retry_period;

    if (waitForReply(waitRequest, waitExpectedResponse, retry_period))
    {
      return true;
    }
  } while (time_done < timeout);
  return false;
}

DashboardResponse DashboardClientImplG5::commandPowerOff()
{
  DashboardResponse response;
  response.message = sendRequestString("power off");
  if (waitForReply("robotmode", "Robotmode: POWER_OFF"))
  {
    response.ok = true;
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandPowerOn(const std::chrono::duration<double> timeout)
{
  DashboardResponse response;
  try
  {
    response.message = retryCommandString("power on", "Powering on", "robotmode", "Robotmode: (IDLE|RUNNING)", timeout);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.message = e.what();
    response.ok = false;
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandBrakeRelease()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("brake release");
    response.ok = true;
    if (!waitForReply("robotmode", "Robotmode: RUNNING"))
    {
      response.ok = false;
      response.message = "Failed to release brakes. Expected 'robotmode' to be 'RUNNING', but did not receive it.";
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.message = e.what();
    response.ok = false;
  }

  return response;
}

DashboardResponse DashboardClientImplG5::commandLoadProgram(const std::string& program_file_name)
{
  // We load the program, which will fail if the program is not found or the requested file does
  // not contain a valid program. Afterwards, we wait until the program state is stopped with a
  // program named the same as the requested program. We cannot check the full file path here, but
  // the important thing is that the program state is stopped.
  DashboardResponse response;
  try
  {
    response.message =
        sendRequestString("load", "(?:Loading program: ).*(?:" + program_file_name + ").*", program_file_name);
    response.ok = true;
    response.data["program_name"] = program_file_name;
    if (!waitForReply("programState", "STOPPED " + std::filesystem::path{ program_file_name }.filename().string()))
    {
      response.ok = false;
      response.message = "Failed to load program. Expected 'programState' to be 'STOPPED " +
                         std::filesystem::path{ program_file_name }.filename().string() + "', but did not receive it.";
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandLoadInstallation(const std::string& installation_file_name)
{
  DashboardResponse response;
  try
  {
    response.message =
        sendRequestString("load installation", "(?:Loading installation: ).*(?:" + installation_file_name + ").*",
                          installation_file_name);
    response.data["installation_name"] = installation_file_name;
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }

  return response;
}

DashboardResponse DashboardClientImplG5::commandPlay()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("play");
    response.ok = true;
    if (!waitForReply("programState", "(?:PLAYING ).*"))
    {
      response.ok = false;
      response.message = "Failed to play program. Expected 'programState' to be 'PLAYING', but did not receive such a  "
                         "message.";
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }

  return response;
}

DashboardResponse DashboardClientImplG5::commandPause()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("pause");
    response.ok = true;
    if (!waitForReply("programState", "(?:PAUSED ).*"))
    {
      response.ok = false;
      response.message = "Failed to pause program. Expected 'programState' to be 'PAUSED', but didn't receive such a "
                         "message.";
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandResume()
{
  throw NotImplementedException("commandResume is not implemented for DashboardClientImplG5. Use commandPlay instead.");
}

DashboardResponse DashboardClientImplG5::commandStop()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("stop");
    response.ok = true;
    if (!waitForReply("programState", "(?:STOPPED ).*"))
    {
      response.message = "Failed to pause program. Expected 'programState' to be 'STOPPED', but didn't receive such a "
                         "message.";
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandClosePopup()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("close popup");
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }

  return response;
}

DashboardResponse DashboardClientImplG5::commandCloseSafetyPopup()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("close safety popup");
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandRestartSafety()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("restart safety");
    response.ok = true;
    if (!waitForReply("robotmode", "Robotmode: POWER_OFF"))
    {
      response.message = "Failed to power off the robot. Expected 'robotmode' to be 'POWER_OFF', but didn't receive "
                         "such a message.";
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandUnlockProtectiveStop()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("unlock protective stop");
    response.ok = true;
    if (!waitForReply("robotmode", "Robotmode: RUNNING"))
    {
      response.message = "Failed to power off the robot. Expected 'robotmode' to be 'RUNNING', but didn't receive such "
                         "a message.";
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandShutdown()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("shutdown");
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandQuit()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("quit");
    response.ok = true;
    disconnect();
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandRunning()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("running");
    response.ok = true;
    std::regex expected("(?:Program running: )(true)");
    if (std::regex_match(response.message, expected))
    {
      response.data["running"] = true;
    }
    else if (std::regex_match(response.message, std::regex("(?:Program running: )(false)")))
    {
      response.data["running"] = false;
    }
    else
    {
      URCL_LOG_WARN("Got neither true nor false as running state. This should not happen. Got: %s",
                    response.message.c_str());
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandIsProgramSaved()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("isProgramSaved");
    response.ok = false;
    std::regex expected("(false|true) (.*\\.urp)");
    std::smatch match;
    if (std::regex_search(response.message, match, expected))
    {
      response.ok = true;
      response.data["saved"] = parseBoolean(match[1]);
      response.data["program_name"] = match[2];
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandIsInRemoteControl()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("is in remote control");
    response.ok = true;
    if (std::regex_match(response.message, std::regex("true")))
    {
      response.data["remote_control"] = true;
    }
    else
    {
      response.data["remote_control"] = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandPopup(const std::string& popup_text)
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("popup", "", popup_text);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandAddToLog(const std::string& log_text)
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("addToLog", "", log_text);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandPolyscopeVersion()
{
  DashboardResponse response;
  try
  {
    std::string polyscope_version = sendRequestString("PolyscopeVersion");
    response.ok = true;
    const std::string expected("(?:URSoftware ).*");
    if (std::regex_match(polyscope_version, std::regex(expected)))
    {
      const std::string version_string = polyscope_version.substr(
          polyscope_version.find(" ") + 1, polyscope_version.find(" (") - polyscope_version.find(" ") - 1);
      polyscope_version_ = VersionInformation::fromString(version_string);
      response.data["polyscope_version"] = version_string;
      response.message = polyscope_version;
    }
    else
    {
      response.message = "Failed to get Polyscope version. Expected response to match regex: " + expected +
                         ", but got: " + polyscope_version;
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGetRobotModel()
{
  DashboardResponse response;
  try
  {
    std::string response_str = sendRequestString("get robot model");
    const std::string expected("(?:UR).*");
    if (std::regex_match(response_str, std::regex(expected)))
    {
      response.ok = true;
      response.message = response_str;
      response.data["robot_model"] = response_str;
    }
    else
    {
      response.message =
          "Failed to get robot model. Expected response to match regex: " + expected + ", but got: " + response_str;
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGetSerialNumber()
{
  DashboardResponse response;
  try
  {
    std::string response_str = sendRequestString("get serial number");
    const std::string expected = "(?:20).*";
    if (std::regex_match(response_str, std::regex(expected)))
    {
      response.ok = true;
      response.message = response_str;
      response.data["serial_number"] = response_str;  // Keep the pattern of saving return data in
      // data fields.
    }
    else
    {
      response.message =
          "Failed to get serial number. Expected response to match regex: " + expected + ", but got: " + response_str;
      response.ok = false;
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandRobotMode()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("robotmode");
    response.ok = true;
    const std::regex pattern("Robotmode: (.*)");
    std::smatch match;
    if (std::regex_search(response.message, match, pattern))
    {
      response.data["robot_mode"] = match[1];
    }
    else
    {
      throw UrException("Got a valid response to 'robotmode' request but couldn't parse the robot mode from the "
                        "response. Please contact the package maintainers about this.");
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGetLoadedProgram()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("get loaded program");
    response.ok = true;
    const std::string pattern("Loaded program: (.*)");
    std::smatch match;
    if (std::regex_search(response.message, match, std::regex(pattern)))
    {
      response.data["program_name"] = match[1];
    }
    else
    {
      throw UrException("Failed to get loaded program. Expected response to match regex: " + pattern +
                        ", but got: " + response.message);
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandSafetyMode()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("safetymode");
    response.ok = true;
    const std::string pattern("Safetymode: (.*)");
    std::smatch match;
    if (std::regex_search(response.message, match, std::regex(pattern)))
    {
      response.data["safety_mode"] = match[1];
    }
    else
    {
      throw UrException("Failed to get safety mode. Expected response to match regex: " + pattern +
                        ", but got: " + response.message);
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandSafetyStatus()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("safetystatus");
    response.ok = true;
    const std::string pattern("Safetystatus: (.*)");
    std::smatch match;
    if (std::regex_search(response.message, match, std::regex(pattern)))
    {
      response.data["safety_status"] = match[1];
    }
    else
    {
      throw UrException("Failed to get safety status. Expected response to match regex: " + pattern +
                        ", but got: " + response.message);
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandProgramState()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("programState");
    response.ok = true;
    const std::string pattern("(STOPPED|PLAYING|PAUSED) (.*)");
    std::smatch match;
    if (std::regex_search(response.message, match, std::regex(pattern)))
    {
      response.data["program_state"] = match[1];
      response.data["program_name"] = match[2];
    }
    else
    {
      throw UrException("Failed to get program state. Expected response to match regex: " + pattern +
                        ", but got: " + response.message);
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGetOperationalMode()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("get operational mode");
    response.ok = true;
    const std::string pattern("(MANUAL|AUTOMATIC|NONE)");
    std::smatch match;
    if (std::regex_search(response.message, match, std::regex(pattern)))
    {
      response.data["operational_mode"] = match[1];
    }
    else
    {
      throw UrException("Failed to get operational mode. Expected response to match regex: " + pattern +
                        ", but got: " + response.message);
    }
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandSetOperationalMode(const std::string& operational_mode)
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("set operational mode",
                                         "(?:Operational mode ).*(?:" + operational_mode + ").*", operational_mode);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandClearOperationalMode()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("clear operational mode");
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandSetUserRole(const std::string& user_role)
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("setUserRole", "", user_role);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGetUserRole()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("getUserRole");
    response.data["user_role"] = response.message;
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

DashboardResponse DashboardClientImplG5::commandGenerateFlightReport(const std::string& report_type)
{
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval tv;
  tv.tv_sec = 180;
  tv.tv_usec = 0;
  TCPSocket::setReceiveTimeout(tv);  // Set timeout to 3 minutes as this command can take a long time to complete
  DashboardResponse response;
  try
  {
    response.message =
        sendRequestString("generate flight report", "(?:Flight Report generated with id:).*", report_type);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  TCPSocket::setReceiveTimeout(configured_tv);  // Reset to configured receive timeout
  return response;
}

DashboardResponse DashboardClientImplG5::commandGenerateSupportFile(const std::string& dir_path)
{
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval tv;
  tv.tv_sec = 600;
  tv.tv_usec = 0;
  TCPSocket::setReceiveTimeout(tv);  // Set timeout to 10 minutes as this command can take a long time to complete
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("generate support file", "(?:Completed successfully:).*", dir_path);
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  TCPSocket::setReceiveTimeout(configured_tv);  // Reset to configured receive timeout
  return response;
}

DashboardResponse DashboardClientImplG5::commandSaveLog()
{
  DashboardResponse response;
  try
  {
    response.message = sendRequestString("saveLog");
    response.ok = true;
  }
  catch (const UnexpectedResponse& e)
  {
    response.ok = false;
    response.message = e.what();
  }
  return response;
}

std::string DashboardClientImplG5::replacePayload(const std::string& command, const std::string& payload)
{
  std::regex pattern("<.*>");
  return std::regex_replace(command, pattern, payload);
}

}  // namespace urcl
