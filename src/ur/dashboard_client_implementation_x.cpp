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
#include <filesystem>
#include <fstream>
#include <ios>
#include <string>

#ifdef _WIN32
#  include <fcntl.h>
#  include <io.h>
#  include <share.h>
#  include <sys/stat.h>

// _mktemp_s only generates a unique name but does not open the file, so this
// wrapper adds the exclusive open to match the POSIX mkstemp contract.
static int mkstemp(char* templ)
{
  if (_mktemp_s(templ, std::strlen(templ) + 1) != 0)
  {
    return -1;
  }
  int fd = -1;
  _sopen_s(&fd, templ, _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _SH_DENYRW, _S_IREAD | _S_IWRITE);
  return fd;
}
#else
#  include <unistd.h>
#endif

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

std::unordered_map<std::string, RobotAPICommand> DashboardClientImplX::g_command_list = {
  { "get_loaded_program",
    { "/program/v1/loaded", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") } },
  { "get_program_list",
    { "/programs/v1", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") } },
  { "upload_program",
    { "/programs/v1", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") } },
  { "update_program",
    { "/programs/v1", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") } },
  { "download_program",
    { "/programs/v1/", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") } },
  { "robot_mode",
    { "/robotstate/v1/robotmode", VersionInformation::fromString("3.1.4"),
      VersionInformation::fromString("10.12.0") } },
  { "safety_mode",
    { "/robotstate/v1/safetymode", VersionInformation::fromString("3.1.4"),
      VersionInformation::fromString("10.12.0") } },
  { "get_operational_mode",
    { "/system/v1/operationalmode", VersionInformation::fromString("3.1.4"),
      VersionInformation::fromString("10.12.0") } },
  { "popup", { "/popup/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "close_popup",
    { "/popup/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "close_safety_popup",
    { "/popup/v1/safety", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  {
      "is_in_remote_control",
      { "/system/v1/controlmode", VersionInformation::fromString("3.1.4"), VersionInformation::fromString("10.12.0") },
  },
  { "PolyscopeVersion",
    { "/versions/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "get_robot_model",
    { "/system/v1/information", VersionInformation::fromString("5.0.107"),
      VersionInformation::fromString("10.14.0") } },
  { "get_serial_number",
    { "/system/v1/information", VersionInformation::fromString("5.0.107"),
      VersionInformation::fromString("10.14.0") } },
  { "shutdown",
    { "/system/v1/shutdown", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "add_to_log",
    { "/system/v1/log", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "generate_flight_report",
    { "/supportfiles/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "download_support_files",
    { "/supportfiles/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "set_operational_mode",
    { "/operational-mode/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } },
  { "clear_operational_mode",
    { "/operational-mode/v1", VersionInformation::fromString("5.0.107"), VersionInformation::fromString("10.14.0") } }
};

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
  cli_->set_read_timeout(std::chrono::seconds(recv_timeout_.tv_sec) + std::chrono::microseconds(recv_timeout_.tv_usec));
  cli_->set_write_timeout(std::chrono::seconds(send_timeout_.tv_sec) +
                          std::chrono::microseconds(send_timeout_.tv_usec));
}

void DashboardClientImplX::setReceiveTimeout(const timeval& timeout)
{
  recv_timeout_ = timeout;
  if (cli_)
  {
    cli_->set_read_timeout(std::chrono::seconds(timeout.tv_sec) + std::chrono::microseconds(timeout.tv_usec));
  }
}

void DashboardClientImplX::setSendTimeout(const timeval& timeout)
{
  send_timeout_ = timeout;
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
  std::string endpoint = base_url_ + "/openapi.json";
  // The PolyScope X Robot API doesn't require any connection prior to making calls. However, this
  // check call will assure that the endpoint for making Robot API calls exist. This could fail if
  // the IP address is wrong or the robot at the IP doesn't have the necessary software version.
  if (auto res = cli_->Get(endpoint))
  {
    if (res->status != 200)
    {
      is_connected_ = false;
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
      is_connected_ = true;
      return true;
    }
  }
  is_connected_ = false;
  return false;
}

void DashboardClientImplX::disconnect()
{
  is_connected_ = false;
  return;
}

timeval DashboardClientImplX::getConfiguredReceiveTimeout() const
{
  return recv_timeout_;
}

timeval DashboardClientImplX::getConfiguredSendTimeout() const
{
  return send_timeout_;
}

void DashboardClientImplX::assertHasCommand(const std::string& command)
{
  if (is_connected_ == false)
  {
    // connect will query the robot API version and set robot_api_version_ if successful.
    if (!connect())
    {
      throw UrException("Failed to connect to the robot API. Cannot assert command availability.");
    }
  }
  if (robot_api_version_ < g_command_list.at(command).robotAPIVersion)
  {
    std::stringstream ss;
    ss << "The command '" << command << "' requires Robot API version " << g_command_list.at(command).robotAPIVersion
       << " or higher. The connected robot has Robot API version " << robot_api_version_
       << ". Please upgrade the robot to PolyScope " << g_command_list.at(command).marketingVersion
       << " or higher to use this command.";
    throw NotImplementedException(ss.str());
  }
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
  // Preserve sub-second precision: duration_cast<seconds> truncates fractional values,
  // so go via microseconds (the smallest unit timeval can represent) and split.
  const auto pwron_us = std::chrono::duration_cast<std::chrono::microseconds>(timeout);
  timeval pwron_tv;
  pwron_tv.tv_sec = static_cast<long>(pwron_us.count() / 1'000'000);
  pwron_tv.tv_usec = static_cast<long>(pwron_us.count() % 1'000'000);
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
  assertHasCommand("close_popup");
  return del("/popup/v1");
}

DashboardResponse DashboardClientImplX::commandCloseSafetyPopup()
{
  assertHasCommand("close_safety_popup");
  return del("/popup/v1/safety");
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
  assertHasCommand("shutdown");
  const std::string endpoint = g_command_list["shutdown"].endpoint;
  return put(endpoint, "");
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
  assertHasCommand("is_in_remote_control");
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

DashboardResponse DashboardClientImplX::commandPopup(const std::string& popup_text, const std::string& title)
{
  assertHasCommand("popup");
  nlohmann::json payload = { { "title", title }, { "message", popup_text } };
  return post("/popup/v1", payload.dump());
}

DashboardResponse DashboardClientImplX::commandAddToLog(const std::string& log_text)
{
  assertHasCommand("add_to_log");
  const std::string endpoint = g_command_list["add_to_log"].endpoint;
  nlohmann::json payload = { { "message", log_text } };
  return post(endpoint, payload.dump());
}

DashboardResponse DashboardClientImplX::commandPolyscopeVersion()
{
  assertHasCommand("PolyscopeVersion");
  const std::string endpoint = g_command_list["PolyscopeVersion"].endpoint;
  auto response = get(endpoint);  // This returns both marketing version and baseline version
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["polyscope_version"] = std::string(json_data["marketingVersion"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandGetRobotModel()
{
  assertHasCommand("get_robot_model");
  const std::string endpoint = g_command_list["get_robot_model"].endpoint;
  auto response = get(endpoint);
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["robot_model"] = std::string(json_data["robotType"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandGetSerialNumber()
{
  assertHasCommand("get_serial_number");
  const std::string endpoint = g_command_list["get_serial_number"].endpoint;
  auto response = get(endpoint);
  auto json_data = json::parse(response.message);
  if (response.ok)
  {
    response.data["serial_number"] = std::string(json_data["serialNumber"]);
  }
  return response;
}

DashboardResponse DashboardClientImplX::commandRobotMode()
{
  assertHasCommand("robot_mode");
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
  assertHasCommand("get_loaded_program");
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
  assertHasCommand("safety_mode");
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
  assertHasCommand("get_operational_mode");
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
  assertHasCommand("generate_flight_report");

  if (!report_type.empty())
  {
    URCL_LOG_WARN("The report_type parameter is not used in the PolyScope X Robot API. Ignoring it.");
  }
  auto timeout = std::chrono::seconds(30);  // Default timeout for generating flight report

  timeval configured_tv = getConfiguredReceiveTimeout();
  // Preserve sub-second precision: duration_cast<seconds> truncates fractional values,
  // so go via microseconds (the smallest unit timeval can represent) and split.
  const auto pwron_us = std::chrono::duration_cast<std::chrono::microseconds>(timeout);
  timeval flightreport_tv;
  flightreport_tv.tv_sec = static_cast<long>(pwron_us.count() / 1'000'000);
  flightreport_tv.tv_usec = static_cast<long>(pwron_us.count() % 1'000'000);
  setReceiveTimeout(flightreport_tv);
  const std::string endpoint = g_command_list["generate_flight_report"].endpoint;

  DashboardResponse response;
  try
  {
    response = post(endpoint, "");
  }
  catch (...)
  {
    setReceiveTimeout(configured_tv);
    throw;
  }

  setReceiveTimeout(configured_tv);
  return response;
}

DashboardResponse DashboardClientImplX::commandDownloadSupportFiles(const std::string& save_path)
{
  assertHasCommand("download_support_files");
  const std::string endpoint = g_command_list["download_support_files"].endpoint;

  DashboardResponse response;

  // Place the temp file in the same directory as the destination so that the
  // later std::filesystem::rename stays on the same filesystem.
  std::filesystem::path dest_dir = std::filesystem::path(save_path).parent_path();
  if (dest_dir.empty())
  {
    dest_dir = ".";
  }

  bool http_ok = true;
  bool write_error = false;
  std::string error_body;

  // mkstemp atomically creates the temp file with O_CREAT|O_EXCL and a random suffix so
  // that a symlink pre-placed at the path is rejected rather than followed, preventing
  // local-privilege symlink attacks in shared directories such as /tmp.
  // On Windows the mkstemp wrapper above calls _mktemp_s + _sopen_s with _O_EXCL.
  std::string temp_save_path = (dest_dir / (std::filesystem::path(save_path).filename().string() + ".XXXXXX")).string();
  int tmp_fd = mkstemp(temp_save_path.data());

  if (tmp_fd < 0)
  {
    response.ok = false;
    response.message = "Failed to create temporary file for saving: " + save_path;
    URCL_LOG_ERROR("%s", response.message.c_str());
    return response;
  }

  auto write_chunk = [&tmp_fd](const char* data, size_t len) -> bool {
#ifndef _WIN32
    return write(tmp_fd, data, len) == static_cast<ssize_t>(len);
#else
    return _write(tmp_fd, data, static_cast<unsigned int>(len)) == static_cast<int>(len);
#endif
  };

  // Since support files can be rather large, we stream the response body to a file
  // instead of loading it all into memory.
  auto res = cli_->Get(
      base_url_ + endpoint,
      [&](const httplib::Response& r) -> bool {
        response.data["status_code"] = r.status;
        http_ok = (r.status >= 200 && r.status < 300);
        return true;  // always receive body: success body → file, error body → error_body
      },
      [&](const char* data, size_t data_length) -> bool {
        if (!http_ok)
        {
          error_body.append(data, data_length);
          return true;
        }
        if (!write_chunk(data, data_length))
        {
          write_error = true;
          return false;
        }
        return true;
      });

#ifndef _WIN32
  close(tmp_fd);
#else
  _close(tmp_fd);
#endif

  if (write_error)
  {
    response.ok = false;
    response.message = "Write error while streaming support files to: " + save_path;
    URCL_LOG_ERROR("%s", response.message.c_str());
    std::filesystem::remove(temp_save_path);
    return response;
  }

  if (!res)
  {
    response.ok = false;
    response.message = "HTTP request failed: " + httplib::to_string(res.error());
    URCL_LOG_ERROR("%s", response.message.c_str());
    std::filesystem::remove(temp_save_path);
    return response;
  }

  if (http_ok)
  {
    // std::filesystem::rename replaces an existing destination atomically on POSIX and
    // uses MoveFileExW(MOVEFILE_REPLACE_EXISTING) on Windows, so repeat downloads to
    // the same path work correctly on both platforms.
    std::error_code ec;
    std::filesystem::rename(temp_save_path, save_path, ec);
    if (ec)
    {
      std::filesystem::remove(temp_save_path);
      response.ok = false;
      response.message = "Failed to rename temporary file to final destination: " + ec.message();
      URCL_LOG_ERROR("%s", response.message.c_str());
      return response;
    }
    response.ok = true;
    response.message = "Downloaded support files to " + save_path;
  }
  else
  {
    response.ok = false;
    response.message = error_body;
    URCL_LOG_ERROR("Failed to download support files. Response message: %s", response.message.c_str());
    std::filesystem::remove(temp_save_path);
  }
  return response;
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
  assertHasCommand("get_program_list");
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
  assertHasCommand("upload_program");
  URCL_LOG_INFO("Uploading program from file: %s", file_path.c_str());
  return performProgramUpload(
      file_path, [this](const std::string& e, const httplib::UploadFormDataItems& f) { return post(e, f, true); });
}

DashboardResponse DashboardClientImplX::commandUpdateProgram(const std::string& file_path)
{
  assertHasCommand("update_program");
  return performProgramUpload(
      file_path, [this](const std::string& e, const httplib::UploadFormDataItems& f) { return put(e, f); });
}

DashboardResponse DashboardClientImplX::commandDownloadProgram(const std::string& program_name,
                                                               const std::string& save_path)
{
  assertHasCommand("download_program");
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
  if (res->status >= 200 && res->status < 300)
  {
    response.ok = true;
  }
  else
  {
    response.ok = false;
  }

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

DashboardResponse DashboardClientImplX::del(const std::string& endpoint, const bool debug)
{
  if (robot_api_version_.isEmpty())
  {
    connect();
  }
  DashboardResponse response;
  if (auto res = cli_->Delete(base_url_ + endpoint))
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
