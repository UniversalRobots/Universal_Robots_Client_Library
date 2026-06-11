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

#include <chrono>
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

  // cpp-httplib's default read_timeout is 300 seconds. Applied unchanged, this makes any
  // dashboard call hang for 5 minutes when the controller becomes unresponsive (e.g. network
  // partition, paused container in tests).
  //
  // The 10 s default for read/write is chosen to cover blocking calls that legitimately take
  // time on real hardware — brake_release, commandLoadProgram (read), commandUploadProgram
  // (write), commandUpdateProgram (write), commandDownloadProgram (read) — without forcing
  // each one to plumb its own timeout. commandPowerOn already accepts its own (longer)
  // timeout parameter. Callers needing different limits can override via setReceiveTimeout.
  cli_->set_connection_timeout(std::chrono::seconds(5));
  cli_->set_read_timeout(std::chrono::seconds(10));
  cli_->set_write_timeout(std::chrono::seconds(10));
}

void DashboardClientImplX::setReceiveTimeout(const timeval& timeout)
{
  recv_timeout_ = std::make_unique<timeval>(timeout);
  if (cli_)
  {
    cli_->set_read_timeout(std::chrono::seconds(timeout.tv_sec) + std::chrono::microseconds(timeout.tv_usec));
  }
}

void DashboardClientImplX::setSendTimeout(const timeval& timeout)
{
  send_timeout_ = std::make_unique<timeval>(timeout);
  if (cli_)
  {
    cli_->set_write_timeout(std::chrono::seconds(timeout.tv_sec) + std::chrono::microseconds(timeout.tv_usec));
  }
}

std::string DashboardClientImplX::sendAndReceive([[maybe_unused]] const std::string& text)
{
  throw NotImplementedException("sendAndReceive is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::connect([[maybe_unused]] const size_t max_num_tries,
                                   [[maybe_unused]] const std::chrono::milliseconds reconnection_time)
{
  // The initial openapi.json fetch can take significantly more time than steady-state
  // dashboard calls (larger payload, first-contact handshake). Mirror the G5 pattern:
  // temporarily extend the read timeout for setup, then restore the configured value.
  // The restore must run on every exit, including exceptions from json::parse or
  // VersionInformation::fromString — hence the catch(...) rethrow guard.
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval setup_tv;
  setup_tv.tv_sec = 10;
  setup_tv.tv_usec = 0;
  setReceiveTimeout(setup_tv);

  bool result = false;
  try
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
      }
      else
      {
        auto db_res = handleHttpResult(res, false);
        auto json_data = json::parse(db_res.message);
        if (db_res.ok && json_data.contains("info") && json_data["info"].contains("version") &&
            json_data["info"]["version"].is_string())
        {
          robot_api_version_ = VersionInformation::fromString(json_data["info"]["version"]);
          URCL_LOG_DEBUG("Connected to Robot API version: %s", robot_api_version_.toString().c_str());
          result = true;
        }
      }
    }
  }
  catch (...)
  {
    setReceiveTimeout(configured_tv);
    throw;
  }

  setReceiveTimeout(configured_tv);
  return result;
}

void DashboardClientImplX::disconnect()
{
  // Nothing to do here, since the Robot API doesn't keep any active connections.
  return;
}

timeval DashboardClientImplX::getConfiguredReceiveTimeout() const
{
  // If the caller has explicitly configured a receive timeout via setReceiveTimeout,
  // return that. Otherwise fall back to the constructor default. See the constructor
  // comment for the rationale on the 10 s default (covers brake_release / program
  // load/upload/download without per-method timeouts).
  timeval tv;
  if (recv_timeout_ != nullptr)
  {
    tv = *recv_timeout_;
  }
  else
  {
    tv.tv_sec = 10;
    tv.tv_usec = 0;
  }
  return tv;
}

timeval DashboardClientImplX::getConfiguredSendTimeout() const
{
  // Mirrors getConfiguredReceiveTimeout. Default of 10 s matches the constructor.
  timeval tv;
  if (send_timeout_ != nullptr)
  {
    tv = *send_timeout_;
  }
  else
  {
    tv.tv_sec = 10;
    tv.tv_usec = 0;
  }
  return tv;
}

VersionInformation DashboardClientImplX::queryPolyScopeVersion()
{
  throw NotImplementedException("queryPolyScopeVersion is not implemented for DashboardClientImplX.");
}

void DashboardClientImplX::assertHasCommand([[maybe_unused]] const std::string& command) const
{
  // Currently, there is only one set of implemented commands. Once the first software release has
  // been made with a Dashboard Server, following versions will support more commands, which is
  // when we might have to deal with that here.
}

bool DashboardClientImplX::sendRequest([[maybe_unused]] const std::string& command_str,
                                       [[maybe_unused]] const std::string& expected_response_pattern,
                                       [[maybe_unused]] const std::string& payload)
{
  throw NotImplementedException("sendRequestis not implemented for DashboardClientImplX.");
}

std::string DashboardClientImplX::sendRequestString([[maybe_unused]] const std::string& command_str,
                                                    [[maybe_unused]] const std::string& expected_response_pattern,
                                                    [[maybe_unused]] const std::string& payload)
{
  throw NotImplementedException("sendRequestString is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::waitForReply([[maybe_unused]] const std::string& command,
                                        [[maybe_unused]] const std::string& expected,
                                        [[maybe_unused]] const std::chrono::duration<double> timeout)
{
  throw NotImplementedException("waitForReply is not implemented for DashboardClientImplX.");
}

bool DashboardClientImplX::retryCommand([[maybe_unused]] const std::string& requestCommand,
                                        [[maybe_unused]] const std::string& requestExpectedResponse,
                                        [[maybe_unused]] const std::string& waitRequest,
                                        [[maybe_unused]] const std::string& waitExpectedResponse,
                                        [[maybe_unused]] const std::chrono::duration<double> timeout,
                                        [[maybe_unused]] const std::chrono::duration<double> retry_period)
{
  throw NotImplementedException("retryCommand is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandPowerOff()
{
  return put("/robotstate/v1/state", R"({"action": "POWER_OFF"})");
}

DashboardResponse DashboardClientImplX::commandPowerOn(const std::chrono::duration<double> timeout)
{
  // commandPowerOn can take significantly longer than steady-state dashboard calls (the robot
  // boots, runs self-checks, etc.). Bump the read timeout to the caller-supplied value for the
  // duration of the PUT, then restore — using the same save-bump-restore pattern as connect().
  // The restore must run on every exit, including exceptions from inside put(), hence the
  // catch(...) rethrow guard.
  timeval configured_tv = getConfiguredReceiveTimeout();
  timeval pwron_tv;
  pwron_tv.tv_sec = static_cast<time_t>(std::chrono::duration_cast<std::chrono::seconds>(timeout).count());
  pwron_tv.tv_usec = 0;
  setReceiveTimeout(pwron_tv);

  DashboardResponse response;
  try
  {
    response = put("/robotstate/v1/state", R"({"action": "POWER_ON"})");
  }
  catch (...)
  {
    setReceiveTimeout(configured_tv);
    throw;
  }

  setReceiveTimeout(configured_tv);
  return response;
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

DashboardResponse
DashboardClientImplX::commandLoadInstallation([[maybe_unused]] const std::string& installation_file_name)
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

DashboardResponse DashboardClientImplX::commandPopup([[maybe_unused]] const std::string& popup_text)
{
  throw NotImplementedException("commandPopup is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandAddToLog([[maybe_unused]] const std::string& log_text)
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

DashboardResponse DashboardClientImplX::commandSetOperationalMode([[maybe_unused]] const std::string& operational_mode)
{
  throw NotImplementedException("commandSetOperationalMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandClearOperationalMode()
{
  throw NotImplementedException("commandClearOperationalMode is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandSetUserRole([[maybe_unused]] const std::string& user_role)
{
  throw NotImplementedException("commandSetUserRole is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGetUserRole()
{
  throw NotImplementedException("commandGetUserRole is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGenerateFlightReport([[maybe_unused]] const std::string& report_type)
{
  throw NotImplementedException("commandGenerateFlightReport is not implemented for DashboardClientImplX.");
}

DashboardResponse DashboardClientImplX::commandGenerateSupportFile([[maybe_unused]] const std::string& dir_path)
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

DashboardResponse DashboardClientImplX::commandDownloadProgram(const std::string& program_name,
                                                               const std::string& save_path)
{
  if (robot_api_version_ < VersionInformation::fromString("3.1.4"))
  {
    throw NotImplementedException("commandDownloadProgram is not implemented for Robot API version < 3.1.4. Please "
                                  "upgrade the robot to PolyScope 10.12.0 or higher to use this command.");
  }
  if (program_name.size() == 0 || save_path.size() == 0)
  {
    std::string error = "Both program_name and save_path parameters should be populated.";
    error += program_name.size() == 0 ? " Program name is empty." : "";
    error += save_path.size() == 0 ? " Save path is empty." : "";
    URCL_LOG_ERROR(error.c_str());
    DashboardResponse response;
    response.ok = false;
    response.message = error;
    return response;
  }
  auto response = get("/programs/v1/" + program_name, false);  // The json response is pretty long. Don't print it.
  if (response.ok)
  {
    std::ofstream save_file(save_path, std::ios_base::out);
    if (!save_file.is_open())
    {
      DashboardResponse error_response;
      error_response.ok = false;
      error_response.message = "Failed to open file for saving: " + save_path;
      URCL_LOG_ERROR(error_response.message.c_str());
      return error_response;
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
