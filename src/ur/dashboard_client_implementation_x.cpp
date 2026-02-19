// Copyright 2025 Universal Robots A/S
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

#include <string>
#include <fstream>
#include <ios>

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

  // Some targets have changed between versions redirecting to the correct endpoint. For this to
  // work, we'll have to follow redirects, which is not the default for httplib.
  cli_->set_follow_location(true);
}

std::string DashboardClientImplX::sendAndReceive(const std::string& text)
{
  throw NotImplementedException("sendAndReceive is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::connect(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  std::string endpoint = base_url_ + "/openapi.json";
  // The PolyScope X Robot API doesn't require any connection prior to making calls. However, this
  // check call will assure that the endpoint for making Robot API calls exist. This could fail if
  // the IP address is wrong or the robot at the IP doesn't have the necessary software version.
  if (auto res = cli_->Get(endpoint))
  {
    if (res->status != 200)
    {
      URCL_LOG_ERROR("Received non-200 response code when connecting to Robot API: %d", res->status);
      return false;
    }
    auto db_res = handleHttpResult(res, false);
    auto json_data = json::parse(db_res.message);
    if (db_res.ok && json_data.contains("info") && json_data["info"].contains("version") &&
        json_data["info"]["version"].is_string())
    {
      robot_api_version_ = VersionInformation::fromString(json_data["info"]["version"]);
      URCL_LOG_DEBUG("Connected to Robot API version: %s", robot_api_version_.toString().c_str());
      return true;
    }
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
  return put("/robotstate/v1/state", R"({"action": "POWER_OFF"})");
}

DashboardResponse DashboardClientImplX::commandPowerOn(const std::chrono::duration<double> timeout)
{
  return put("/robotstate/v1/state", R"({"action": "POWER_ON"})");
}

DashboardResponse DashboardClientImplX::commandBrakeRelease()
{
  return put("/robotstate/v1/state", R"({"action": "BRAKE_RELEASE"})");
}

DashboardResponse DashboardClientImplX::commandLoadProgram(const std::string& program_file_name)
{
  std::string endpoint = "/program/v1/loaded";
  std::string program_key = "name";
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    endpoint = "/program/v1/load";
    program_key = "programName";
  }
  return put(endpoint, R"({")" + program_key + R"(": ")" + program_file_name + R"("})");
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
  return put("/robotstate/v1/state", R"({"action": "RESTART_SAFETY"})");
}

DashboardResponse DashboardClientImplX::commandUnlockProtectiveStop()
{
  return put("/robotstate/v1/state", R"({"action": "UNLOCK_PROTECTIVE_STOP"})");
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
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandIsInRemoteControl is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/system/v1/controlmode");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["mode"] = std::string(json_data["mode"]);
    if (std::string(json_data["mode"]) == "REMOTE")
    {
      response.data["remote_control"] = true;
    }
    else
    {
      response.data["remote_control"] = false;
    }
  }
  return response;
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
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandRobotMode is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/robotstate/v1/robotmode");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["robot_mode"] = std::string(json_data["mode"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandGetLoadedProgram()
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandGetLoadedProgram is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/program/v1/loaded");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["program_name"] = std::string(json_data["name"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandSafetyMode()
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandSafetyMode is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/robotstate/v1/safetymode");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["safety_mode"] = std::string(json_data["mode"]);
  }
  return response;
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
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandGetOperationalMode is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/system/v1/operationalmode");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["operational_mode"] = std::string(json_data["mode"]);
  }
  return response;
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

DashboardResponse DashboardClientImplX::commandGetProgramList()
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandGetProgramList is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/programs/v1/");
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    std::vector<ProgramInformation> programs;
    for (auto prog : json_data["programs"])
    {
      unsigned int last_modified = 0;
      if (!prog["lastModifiedDate"].is_null())
      {
        last_modified = static_cast<unsigned int>(prog["lastModifiedDate"]);
      }

      unsigned int last_saved = 0;
      if (!prog["lastSavedDate"].is_null())
      {
        last_saved = static_cast<unsigned int>(prog["lastSavedDate"]);
      }

      ProgramInformation pi(prog["createdDate"], prog["description"], last_modified, last_saved, prog["name"],
                            prog["programState"]);
      programs.push_back(pi);
    }
    response.data["programs"] = programs;
  }
  return response;
}

DashboardResponse DashboardClientImplX::performProgramUpload(
    const std::string& file_path,
    std::function<DashboardResponse(const std::string&, const httplib::UploadFormDataItems&)> upload_func)
{
  std::ifstream file(file_path, std::ios_base::in);
  if (!file.is_open())
  {
    DashboardResponse response;
    response.ok = false;
    response.message = "URPX File not found: " + file_path;
    URCL_LOG_ERROR(response.message.c_str());
    return response;
  }
  std::string line;
  std::string content;
  while (getline(file, line, '\n'))
  {
    content.append(line + '\n');
  }

  httplib::UploadFormDataItems form_data = { { "file", content, "filename", "text/plain" } };
  auto response = upload_func("/programs/v1/", form_data);
  auto json_data = json::parse(response.message);
  if (response.ok && json_data.contains("programName") && json_data["programName"].is_string())
  {
    response.data["program_name"] = std::string(json_data["programName"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandUploadProgram(const std::string& file_path)
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandUploadProgram is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  URCL_LOG_INFO("Uploading program from file: %s", file_path.c_str());
  return performProgramUpload(
      file_path, [this](const std::string& e, const httplib::UploadFormDataItems& f) { return post(e, f, true); });
}

DashboardResponse DashboardClientImplX::commandUpdateProgram(const std::string& file_path)
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandUpdateProgram is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  return performProgramUpload(
      file_path, [this](const std::string& e, const httplib::UploadFormDataItems& f) { return put(e, f); });
}

DashboardResponse DashboardClientImplX::commandDownloadProgram(const std::string& filename,
                                                               const std::string& save_path)
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandDownloadProgram is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  auto response = get("/programs/v1/" + filename, false);  // The json response is pretty long. Don't print it.
  if (response.ok)
  {
    std::ofstream save_file(save_path, std::ios_base::out);
    if (!save_file.is_open())
    {
      DashboardResponse response;
      response.ok = false;
      response.message = "Failed to open file for saving: " + save_path;
      URCL_LOG_ERROR(response.message.c_str());
      return response;
    }
    save_file << response.message;

    response.message = "Downloaded program to " + save_path;
  }
  else
  {
    URCL_LOG_ERROR("Failed to download program. Response message: %s", response.message.c_str());
  }
  return response;
}

DashboardResponse DashboardClientImplX::handleHttpResult(const httplib::Result& res, const bool debug)
{
  DashboardResponse response;
  if (debug)
  {
    URCL_LOG_INFO(res->body.c_str());
  }
  response.message = res->body;
  response.data["status_code"] = res->status;
  response.ok = res->status == 200;
  return response;
}

DashboardResponse DashboardClientImplX::post(const std::string& endpoint, const httplib::UploadFormDataItems& form_data,
                                             const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Post(base_url_ + endpoint, form_data))
  {
    response = handleHttpResult(res, debug);
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardResponse DashboardClientImplX::post(const std::string& endpoint, const std::string& json_data,
                                             const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Post(base_url_ + endpoint, json_data, "application/json"))
  {
    response = handleHttpResult(res, debug);
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardResponse DashboardClientImplX::put(const std::string& endpoint, const std::string& json_data, const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Put(base_url_ + endpoint, json_data, "application/json"))
  {
    response = handleHttpResult(res, debug);
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardResponse DashboardClientImplX::put(const std::string& endpoint, const httplib::UploadFormDataItems& form_data,
                                            const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Put(base_url_ + endpoint, form_data))
  {
    response = handleHttpResult(res, debug);
  }
  else
  {
    throw UrException("Error code: " + to_string(res.error()));
  }
  return response;
}

DashboardResponse DashboardClientImplX::get(const std::string& endpoint, const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Get(base_url_ + endpoint))
  {
    response = handleHttpResult(res, debug);
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
