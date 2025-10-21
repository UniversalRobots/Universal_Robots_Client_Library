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

#include <regex>
#include <string>
#include <thread>

#include "ur_client_library/ur/version_information.h"
#include <ur_client_library/exceptions.h>
#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client_implementation_x.h>

#include <urcl_3rdparty/nlohmann_json/json.hpp>
#include <urcl_3rdparty/httplib/httplib.h>
using json = nlohmann::json;

using namespace std::chrono_literals;

namespace urcl
{

DashboardClientImplX::DashboardClientImplX(const std::string& host) : DashboardClientImpl(host)
{
  cli_ = std::make_unique<httplib::Client>("http://" + host);
}

std::string DashboardClientImplX::sendAndReceive(const std::string& text)
{
  throw NotImplementedException("sendAndReceive is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  // The PolyScope X Robot API doesn't require any connection prior to making calls. However, this
  // check call will assure that the endpoint for making Robot API calls exist. This could fail if
  // the IP address is wrong or the robot at the IP doesn't have the necessary software version.
  // Quick check whether there is a dashboard client available at the given host.
  if (auto res = cli_->Get(base_url_ + "/system/v1/system-time/"))
  {
    return res->status == 200;
  }
  return false;
}

void DashboardClientImplX::disconnect()
{
  // Nothing to do here, since the Robot API doesn't keep any active connections.
  return;
}

timeval DashboardClientImplX::getConfiguredReceiveTimeout() const
{
  timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  return tv;
}

VersionInformation DashboardClientImplX::queryPolyScopeVersion()
{
  throw NotImplementedException("queryPolyScopeVersion is not implemented for DashboardClientImplX.");
}

void DashboardClientImplX::assertHasCommand(const std::string& command) const
{
  // Currently, there is only one set of implemented commands. Once the first software release has
  // been made with a Dashboard Server, following versions will support more commands, which is
  // when we might have to deal with that here.
}

bool DashboardClientImplX::sendRequest(const std::string& command_str, const std::string& expected_response_pattern,
                                       const std::string& payload)
{
  throw NotImplementedException("sendRequestis not implemented for DashboardClientImplX.");
}

std::string DashboardClientImplX::sendRequestString(const std::string& command_str,
                                                    const std::string& expected_response_pattern,
                                                    const std::string& payload)
{
  throw NotImplementedException("sendRequestString is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::waitForReply(const std::string& command, const std::string& expected,
                                        const std::chrono::duration<double> timeout)
{
  throw NotImplementedException("waitForReply is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::retryCommand(const std::string& requestCommand, const std::string& requestExpectedResponse,
                                        const std::string& waitRequest, const std::string& waitExpectedResponse,
                                        const std::chrono::duration<double> timeout,
                                        const std::chrono::duration<double> retry_period)
{
  throw NotImplementedException("retryCommand is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandPowerOff()
{
  return put("/robotstate/v1/state/", R"({"action": "POWER_OFF"})");
}

DashboardResponse DashboardClientImplX::commandPowerOn(const std::chrono::duration<double> timeout)
{
  return put("/robotstate/v1/state/", R"({"action": "POWER_ON"})");
}

DashboardResponse DashboardClientImplX::commandBrakeRelease()
{
  return put("/robotstate/v1/state/", R"({"action": "BRAKE_RELEASE"})");
}

DashboardResponse DashboardClientImplX::commandLoadProgram(const std::string& program_file_name)
{
  return put("/program/v1/load", R"({"programName": ")" + program_file_name + R"("})");
}

DashboardResponse DashboardClientImplX::commandLoadInstallation(const std::string& installation_file_name)
{
  throw NotImplementedException("commandLoadInstallation is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandPlay()

{
  return put("/program/v1/state", R"({"action": "play"})");
}

DashboardResponse DashboardClientImplX::commandPause()
{
  return put("/program/v1/state", R"({"action": "pause"})");
}

DashboardResponse DashboardClientImplX::commandResume()
{
  return put("/program/v1/state", R"({"action": "resume"})");
}

DashboardResponse DashboardClientImplX::commandStop()
{
  return put("/program/v1/state", R"({"action": "stop"})");
}

DashboardResponse DashboardClientImplX::commandClosePopup()
{
  throw NotImplementedException("commandClosePopup is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandCloseSafetyPopup()
{
  throw NotImplementedException("commandCloseSafetyPopup is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandRestartSafety()
{
  return put("/robotstate/v1/state/", R"({"action": "RESTART_SAFETY"})");
}

DashboardResponse DashboardClientImplX::commandUnlockProtectiveStop()
{
  return put("/robotstate/v1/state/", R"({"action": "UNLOCK_PROTECTIVE_STOP"})");
}

DashboardResponse DashboardClientImplX::commandShutdown()
{
  throw NotImplementedException("commandShutdown is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandQuit()
{
  // No active connection, therefore nothing to do.
  return DashboardResponse{ true, "Nothing to quit here. All is fine.", {} };
}

DashboardResponse DashboardClientImplX::commandRunning()
{
  throw NotImplementedException("commandRunning is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandIsProgramSaved()
{
  throw NotImplementedException("commandIsProgramSaved is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandIsInRemoteControl()
{
  throw NotImplementedException("commandIsInRemoteControl is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandPopup(const std::string& popup_text)
{
  throw NotImplementedException("commandPopup is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandAddToLog(const std::string& log_text)
{
  throw NotImplementedException("commandAddToLog is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandPolyscopeVersion()
{
  throw NotImplementedException("commandPolyscopeVersion is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGetRobotModel()
{
  throw NotImplementedException("commandGetRobotModel is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGetSerialNumber()
{
  throw NotImplementedException("commandGetSerialNumber is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandRobotMode()
{
  throw NotImplementedException("commandRobotMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGetLoadedProgram()
{
  throw NotImplementedException("commandGetLoadedProgram is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSafetyMode()
{
  throw NotImplementedException("commandSafetyMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSafetyStatus()
{
  throw NotImplementedException("commandSafetyStatus is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandProgramState()
{
  auto response = get("/program/v1/state");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["program_state"] = std::string(json_data["state"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandGetOperationalMode()
{
  throw NotImplementedException("commandGetOperationalMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSetOperationalMode(const std::string& operational_mode)
{
  throw NotImplementedException("commandSetOperationalMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandClearOperationalMode()
{
  throw NotImplementedException("commandClearOperationalMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSetUserRole(const std::string& user_role)
{
  throw NotImplementedException("commandSetUserRole is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGetUserRole()
{
  throw NotImplementedException("commandGetUserRole is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGenerateFlightReport(const std::string& report_type)
{
  throw NotImplementedException("commandGenerateFlightReport is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGenerateSupportFile(const std::string& dir_path)
{
  throw NotImplementedException("commandGenerateSupportFile is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSaveLog()
{
  throw NotImplementedException("commandSaveLog is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::put(const std::string& endpoint, const std::string& json_data)
{
  DashboardResponse response;
  if (auto res = cli_->Put(base_url_ + endpoint, json_data, "application/json"))
  {
    URCL_LOG_INFO(res->body.c_str());
    response.message = res->body;
    response.data["status_code"] = res->status;
    response.ok = res->status == 200;
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardResponse DashboardClientImplX::get(const std::string& endpoint)
{
  DashboardResponse response;
  if (auto res = cli_->Get(base_url_ + endpoint))
  {
    response.message = res->body;
    response.data["status_code"] = res->status;
    response.ok = res->status == 200;
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardClientImplX::~DashboardClientImplX()
{
  // We need to keep the implementation in the cpp file due to the unique_ptr of the incomplete
  // type httplib::Client.
}

}  // namespace urcl
