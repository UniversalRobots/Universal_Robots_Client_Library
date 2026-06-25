// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright © 2024-2025 Ocado Group
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
// -- END LICENSE BLOCK ------------------------------------------------

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/primary/robot_message.h>
#include <ur_client_library/primary/robot_state.h>
#include "ur_client_library/exceptions.h"
#include <ur_client_library/helpers.h>
#include <ur_client_library/compile_options.h>

#include <chrono>
#include <iomanip>
#include <regex>
namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip, [[maybe_unused]] comm::INotifier& notifier)
  : stream_(robot_ip, UR_PRIMARY_PORT)
{
  parser_.setStrictMode(COMPILE_OPTIONS.PRIMARY_CLIENT_STRICT_PARSING);
  prod_.reset(new comm::URProducer<PrimaryPackage>(stream_, parser_));

  consumer_.reset(new PrimaryConsumer());
  consumer_->setErrorCodeMessageCallback(std::bind(&PrimaryClient::errorMessageCallback, this, std::placeholders::_1));
  consumer_->setKeyMessageCallback(std::bind(&PrimaryClient::keyMessageCallback, this, std::placeholders::_1));
  consumer_->setRuntimeExceptionCallback(
      std::bind(&PrimaryClient::runtimeExceptionCallback, this, std::placeholders::_1));

  // Configure multi consumer even though we only have one consumer as default, as this enables the user to add more
  // consumers after the object has been created
  std::vector<std::shared_ptr<comm::IConsumer<PrimaryPackage>>> consumers;
  consumers.push_back(consumer_);
  multi_consumer_.reset(new comm::MultiConsumer<PrimaryPackage>(consumers));

  pipeline_.reset(
      new comm::Pipeline<PrimaryPackage>(*prod_, multi_consumer_.get(), "PrimaryClient Pipeline", notifier_));
}

PrimaryClient::~PrimaryClient()
{
  URCL_LOG_INFO("Stopping primary client pipeline");
  pipeline_->stop();
}

void PrimaryClient::start(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  URCL_LOG_INFO("Starting primary client pipeline");
  pipeline_->init(max_num_tries, reconnection_time);
  pipeline_->run();
}

void PrimaryClient::stop()
{
  pipeline_->stop();
  stream_.close();
}

void PrimaryClient::addPrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->addConsumer(primary_consumer);
}

void PrimaryClient::removePrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->removeConsumer(primary_consumer);
}

void PrimaryClient::errorMessageCallback(ErrorCode& code)
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  error_code_queue_.push_back(code);
}

void PrimaryClient::keyMessageCallback(KeyMessage& msg)
{
  std::lock_guard<std::mutex> lock_guard(key_message_queue_mutex_);
  key_message_queue_.push_back(msg);
}

void PrimaryClient::runtimeExceptionCallback(RuntimeExceptionMessage& msg)
{
  std::scoped_lock lock(runtime_exception_mutex_);
  latest_runtime_exception_ = std::make_shared<primary_interface::RuntimeExceptionMessage>(msg);
}

std::deque<ErrorCode> PrimaryClient::getErrorCodes()
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  std::deque<ErrorCode> error_codes;
  error_codes = error_code_queue_;
  error_code_queue_.clear();
  return error_codes;
}

bool PrimaryClient::safetyModeAllowsExecution()
{
  SafetyMode mode = getSafetyMode();
  switch (mode)
  {
    case SafetyMode::NORMAL:
    case SafetyMode::REDUCED:
    case SafetyMode::RECOVERY:
    // Safety mode might be unknown, as it is only updated on changes.
    case SafetyMode::UNDEFINED_SAFETY_MODE:
      return true;

    default:
      return false;
  }
}

void PrimaryClient::sendScriptBlocking(const std::string& program, const std::string& script_name,
                                       const std::chrono::milliseconds start_timeout, const bool fail_on_warnings,
                                       const bool retry_on_readonly_interface)
{
  ScriptInfo script_info = prepareScript(program, script_name);

  RobotMode robot_mode = getRobotMode();
  std::chrono::milliseconds robot_mode_timeout(1000);
  auto start_time = std::chrono::system_clock::now();
  while (robot_mode == RobotMode::UNKNOWN)
  {
    auto now = std::chrono::system_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count() > robot_mode_timeout.count())
    {
      throw TimeoutException("Robot mode not received within timeout. ", robot_mode_timeout);
    }
    URCL_LOG_INFO("Robot mode not received yet, waiting for it to be received.");
    std::chrono::milliseconds update_period(100);
    std::this_thread::sleep_for(update_period);
    robot_mode = getRobotMode();
  }

  if (robot_mode != RobotMode::RUNNING)
  {
    throw RobotModeException("Script execution via primary interface", urcl::RobotMode::RUNNING, robot_mode);
  }

  if (!safetyModeAllowsExecution())
  {
    std::vector<urcl::SafetyMode> allowed_modes = { urcl::SafetyMode::NORMAL, urcl::SafetyMode::REDUCED,
                                                    urcl::SafetyMode::RECOVERY };
    allowed_modes.push_back(urcl::SafetyMode::UNDEFINED_SAFETY_MODE);  // Remove when safety mode gets updated
                                                                       // continuously

    throw SafetyModeException("Script execution via primary interface", allowed_modes, getSafetyMode());
  }

  try
  {
    sendScriptMonitorExecution(script_info, start_timeout, fail_on_warnings);
  }
  catch ([[maybe_unused]] const ReadOnlyInterfaceException& exc)
  {
    if (retry_on_readonly_interface)
    {
      URCL_LOG_INFO("Script execution failed due to the primary interface being read-only. Restarting primary "
                    "interface and retrying once.");
      stop();
      start();
      sendScriptMonitorExecution(script_info, start_timeout, fail_on_warnings);
    }
    else
    {
      throw;
    }
  }
}

void PrimaryClient::sendScriptMonitorExecution(const ScriptInfo& script_info, const std::chrono::milliseconds& timeout,
                                               const bool fail_on_warnings)
{
  // Clear runtime exception
  {
    std::scoped_lock lock(runtime_exception_mutex_);
    latest_runtime_exception_ = nullptr;
  }
  // Clear existing error codes
  getErrorCodes();
  // Clear key messages
  {
    std::scoped_lock lock(key_message_queue_mutex_);
    key_message_queue_.clear();
  }

  bool script_sent = sendScript(script_info.script_code);
  if (!script_sent)
  {
    throw StreamNotConnectedException("Script could not be sent to the robot. Ensure that the primary interface is "
                                      "connected.");
  }

  // No feedback from secondary programs, so we assume success
  if (script_info.script_type == ScriptTypes::SEC)
  {
    URCL_LOG_INFO("Script %s was determined to be a secondary program. Script was transferred successfully, but no "
                  "further feedback will be provided.",
                  script_info.script_name.c_str());
    return;
  }

  const auto script_start_time = std::chrono::system_clock::now();
  // Ignore start delay if it is 0
  bool script_started = timeout == std::chrono::milliseconds(0) ? true : false;
  // Error codes and key messages are produced by the same pipeline thread, but they live in two
  // separate queues that are drained independently in this loop. A warning ErrorCode and the
  // PROGRAM_XXX_STOPPED KeyMessage may therefore be visible to consecutive iterations rather than
  // to the same one. To avoid returning success before such a "straggler" warning/error has been
  // observed, we don't return immediately when STOPPED is seen. Instead we record that fact and
  // keep draining the error / runtime exception queues for a short grace period.
  bool program_stopped = false;
  std::chrono::system_clock::time_point program_stopped_time;
  const std::chrono::milliseconds post_stop_drain_period(100);
  while (true)
  {
    {
      std::scoped_lock lock(runtime_exception_mutex_);
      if (latest_runtime_exception_ != nullptr)
      {
        std::stringstream ss;
        ss << "Runtime exception occured during script execution."
           << "Runtime exception type: " << latest_runtime_exception_->text_ << "\n"
           << "Exception occured at line " << latest_runtime_exception_->line_number_ << ", column "
           << latest_runtime_exception_->column_number_ << "\n";
        // Line and column numbers should always be 1-based, but we check that they are greater
        // than 0 just to be sure before using them for indexing in the debug print below
        if (latest_runtime_exception_->line_number_ > 0 && latest_runtime_exception_->column_number_ > 0)
        {
          // Debug print for the user
          auto script_lines = splitString(script_info.script_code, "\n");
          size_t line_count = script_lines.size();
          size_t line_number_width = std::to_string(line_count).size();
          for (size_t i = 0; i < line_count; i++)
          {
            if (!script_lines[i].empty())
            {
              ss << std::setw(line_number_width) << (i + 1) << ": " << script_lines[i] << "\n";
            }
            if (static_cast<uint32_t>(i) == latest_runtime_exception_->line_number_ - 1)
            {
              uint32_t output_column =
                  latest_runtime_exception_->column_number_ - 1 + (static_cast<uint32_t>(line_number_width) + 2);
              for (uint32_t j = 0; j < output_column; j++)
              {
                ss << " ";
              }
              ss << "^<--- here\n";
            }
          }
        }
        throw RobotRuntimeException(ss.str());
      }
    }

    auto errors = getErrorCodes();
    if (errors.size() > 0)
    {
      bool is_error = false;
      bool is_warning = false;
      bool is_read_only = false;

      std::stringstream error_stream;
      error_stream << "Robot error codes received during script execution: \n";

      for (auto error : errors)
      {
        switch (error.report_level)
        {
          case ReportLevel::VIOLATION:
          case ReportLevel::FAULT:
          case ReportLevel::CRITICAL_FAULT:
            error_stream << "Code: " << error.to_string << ", severity: " << reportLevelString(error.report_level)
                         << "\n";
            is_error = true;
            break;
          case ReportLevel::WARNING:
            if (fail_on_warnings)
            {
              error_stream << "Code: " << error.to_string << ", severity: " << reportLevelString(error.report_level)
                           << "\n";
            }
            is_warning = true;
            break;
          case ReportLevel::DEBUG:
          case ReportLevel::INFO:
          case ReportLevel::DEVL_DEBUG:
          case ReportLevel::DEVL_INFO:
          case ReportLevel::DEVL_WARNING:
          case ReportLevel::DEVL_VIOLATION:
          case ReportLevel::DEVL_FAULT:
          case ReportLevel::DEVL_CRITICAL_FAULT:
            break;
        }
        if (error.message_code == 210)
        {
          // C210 means that the primary client is connected to a read-only primary interface,
          // which means that scripts cannot be executed. We check for this error code to give the
          // user a more specific error message in this case.
          is_error = true;
          is_read_only = true;
        }
      }
      if (is_error)
      {
        if (!is_read_only)
        {
          commandStop();
        }
        else
        {
          throw ReadOnlyInterfaceException("Script cannot be executed since primary client is connected to a read-only "
                                           "primary "
                                           "interface. If you have switched from local to remote mode recently, try "
                                           "reconnecting the "
                                           "primary client and send the script code again.");
        }
        throw RobotErrorCodeException(error_stream.str());
      }
      if (is_warning && fail_on_warnings)
      {
        throw RobotErrorCodeException(error_stream.str());
      }
    }

    // Copy out key messages
    std::deque<urcl::primary_interface::KeyMessage> key_messages;
    {
      std::scoped_lock lock(key_message_queue_mutex_);
      for (auto msg : key_message_queue_)
      {
        key_messages.push_back(msg);
      }
      key_message_queue_.clear();
    }
    if (key_messages.size() > 0)
    {
      for (auto message : key_messages)
      {
        if (message.title_ == "PROGRAM_XXX_STOPPED" && message.text_ == script_info.script_name)
        {
          if (!program_stopped)
          {
            URCL_LOG_DEBUG("Script with name %s reported as stopped. Draining residual error / "
                           "runtime exception messages for up to %lld ms before reporting success.",
                           script_info.script_name.c_str(), static_cast<long long>(post_stop_drain_period.count()));
            program_stopped = true;
            program_stopped_time = std::chrono::system_clock::now();
            // STOPPED implies the script was started, otherwise the controller could not have
            // stopped it. This avoids a spurious "not started within timeout" failure if the
            // STARTED message was never observed in this loop.
            script_started = true;
          }
        }
        else if (!script_started && message.title_ == "PROGRAM_XXX_STARTED" && message.text_ == script_info.script_name)
        {
          URCL_LOG_INFO("Script with name %s started", script_info.script_name.c_str());
          script_started = true;
        }
      }
    }

    // After STOPPED has been observed, give the pipeline a short grace period to deliver any
    // warning / fault / violation error codes that may have been parsed just before STOPPED but
    // ended up in the error queue after this iteration's getErrorCodes() snapshot. Only declare
    // success once that grace period elapsed without any reportable issue.
    if (program_stopped)
    {
      const auto now = std::chrono::system_clock::now();
      if (now - program_stopped_time >= post_stop_drain_period)
      {
        URCL_LOG_INFO("Script with name %s executed successfully", script_info.script_name.c_str());
        return;
      }
    }
    else
    {
      const auto current_time = std::chrono::system_clock::now();
      const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - script_start_time);

      if (!script_started && elapsed_time > timeout)
      {
        throw urcl::TimeoutException("Script with name " + script_info.script_name + " not started within timeout. ",
                                     timeout);
      }
    }
    std::chrono::milliseconds wait_period(10);
    std::this_thread::sleep_for(wait_period);
  }
}

std::vector<std::string> PrimaryClient::stripCommentsAndWhitespace(std::vector<std::string> split_script)
{
  std::vector<std::string> stripped_script;
  for (auto line : split_script)
  {
    for (auto c : line)
    {
      if (!isspace(c))
      {
        if (c == '#')
        {
          break;
        }
        else
        {
          stripped_script.push_back(line);
          break;
        }
      }
    }
  }
  return stripped_script;
}

std::string PrimaryClient::truncateScriptName(const std::string candidate_name)
{
  std::string final_name = candidate_name;
  // Limit script name length to 31, to ensure backwards compatibility
  if (final_name.size() > 31)
  {
    final_name = final_name.substr(0, 31);
    URCL_LOG_WARN("Given script name was too long, and has been truncated. New script name is: %s", final_name.c_str());
  }

  return final_name;
}

ScriptInfo PrimaryClient::prepareScript(std::string script, std::string script_name)
{
  // Split the given script in to separate lines
  std::vector<std::string> split_script = splitString(script, "\n");

  // Remove all comments and white-space-only lines
  std::vector<std::string> stripped_script = stripCommentsAndWhitespace(split_script);

  if (stripped_script.size() == 0)
  {
    throw ScriptCodeSyntaxException("Script is empty after stripping comments and whitespace.");
  }

  // Use given script name or create one
  int64_t current_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  // Assign name according to inputs
  std::string actual_script_name = script_name.empty() ? "script_" + std::to_string(current_time) : script_name;

  ScriptTypes actual_script_type = urcl::primary_interface::ScriptTypes::DEF;
  // Is the script wrapped in a function definition? If not add one
  if (stripped_script[0].substr(0, 4).find("def ") == script.npos &&
      stripped_script[0].substr(0, 4).find("sec ") == script.npos)
  {
    // Check that the final name is not too long
    actual_script_name = truncateScriptName(actual_script_name);
    std::string definition = "def " + actual_script_name + "():";
    std::string end = "end";
    // Add indentation to the existing script code
    for (std::size_t i = 0; i < stripped_script.size(); i++)
    {
      stripped_script[i] = "  " + stripped_script[i];
    }
    // Add function definition and end statement to the stripped script lines vector
    stripped_script.insert(stripped_script.begin(), definition);
    stripped_script.push_back(end);
  }
  // Otherwise extract script name and type from function
  else
  {
    size_t name_end = stripped_script[0].find("(");
    if (name_end == stripped_script[0].npos)
    {
      throw urcl::ScriptCodeSyntaxException("Function definition detected in script, but a '(' could not be found. "
                                            "Definition is invalid.");
    }
    std::string name_in_script = stripped_script[0].substr(4, name_end - 4);
    if (stripped_script[0].substr(0, 4).find("def ") != stripped_script[0].npos)
    {
      actual_script_type = ScriptTypes::DEF;
    }
    else
    {
      actual_script_type = ScriptTypes::SEC;
    }
    // Check that the script name is not too long, replace it, if it is
    actual_script_name = truncateScriptName(name_in_script);
    if (actual_script_name.size() != name_in_script.size())
    {
      stripped_script[0].replace(stripped_script[0].find(name_in_script), name_in_script.size(), actual_script_name);
    }
  }

  // Validate script_name
  static const std::regex valid_name(R"(^[A-Za-z_][A-Za-z0-9_]*$)");
  if (!std::regex_match(actual_script_name, valid_name))
  {
    throw ScriptCodeSyntaxException("Invalid script name: '" + actual_script_name +
                                    "'. Can only contain letters, numbers and underscores. First character "
                                    "must be a letter or underscore.");
  }

  if (stripped_script.back().substr(0, 3).find("end") == script.npos)
  {
    throw ScriptCodeSyntaxException("Script contains either function definition or secondary process "
                                    "definition, "
                                    "but no 'end' term. Script is invalid.");
  }

  // Concatenate all the script lines in to the final script
  std::string prepared_script = "";
  for (auto line : stripped_script)
  {
    prepared_script.append(line + "\n");
  }

  // Return final script code as well as the name of the script as it will be exectuted
  return ScriptInfo(actual_script_name, prepared_script, actual_script_type);
}

bool PrimaryClient::sendScript(const std::string& program)
{
  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.

  auto program_with_newline = program + "\n";

  size_t len = program_with_newline.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_with_newline.c_str());
  size_t written;

  const auto send_script_contents = [this, program_with_newline, data, len,
                                     &written](const std::string&& description) -> bool {
    if (stream_.write(data, len, written))
    {
      URCL_LOG_DEBUG("Sent program to robot:\n%s", program_with_newline.c_str());
      return true;
    }
    const std::string error_message = "Could not send program to robot: " + description;
    URCL_LOG_ERROR(error_message.c_str());
    return false;
  };

  if (send_script_contents("initial attempt"))
  {
    return true;
  }

  if (reconnectStream())
  {
    return send_script_contents("after reconnecting primary stream");
  }

  return false;
}

bool PrimaryClient::reconnectStream()
{
  URCL_LOG_DEBUG("Closing primary stream...");
  stream_.close();
  if (stream_.connect())
  {
    URCL_LOG_DEBUG("Primary stream connected");
    return true;
  }
  URCL_LOG_ERROR("Failed to reconnect primary stream!");
  return false;
}

bool PrimaryClient::checkCalibration(const std::string& checksum)
{
  std::shared_ptr<primary_interface::KinematicsInfo> kin_info = consumer_->getKinematicsInfo();
  while (kin_info == nullptr)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    kin_info = consumer_->getKinematicsInfo();
  }
  URCL_LOG_DEBUG("Got calibration information from robot.");

  return kin_info->toHash() == checksum;
}

void PrimaryClient::commandPowerOn(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("power on"))
  {
    throw UrException("Failed to send power on command to robot");
  }

  if (validate)
  {
    try
    {
      waitFor(
          [this]() {
            const auto mode = getRobotMode();
            return mode == RobotMode::IDLE || mode == RobotMode::RUNNING;
          },
          timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not power on within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandPowerOff(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("power off"))
  {
    throw UrException("Failed to send power off command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return getRobotMode() == RobotMode::POWER_OFF; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not power off within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandBrakeRelease(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("set robotmode run"))
  {
    throw UrException("Failed to send brake release command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return getRobotMode() == RobotMode::RUNNING; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not release the brakes within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandUnlockProtectiveStop(const bool validate, const std::chrono::milliseconds timeout)
{
  if (!sendScript("set unlock protective stop"))
  {
    throw UrException("Failed to send unlock protective stop command to robot");
  }
  if (validate)
  {
    try
    {
      waitFor([this]() { return consumer_->getRobotModeData()->is_protective_stopped_ == false; }, timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not unlock the protective stop within the given timeout", timeout);
    }
  }
}

void PrimaryClient::commandStop(const bool validate, const std::chrono::milliseconds timeout)
{
  std::shared_ptr<RobotModeData> robot_mode_data = consumer_->getRobotModeData();
  if (robot_mode_data == nullptr)
  {
    throw UrException("Stopping a program while robot state is unknown. This should not happen");
  }

  if (!sendScript("stop program"))
  {
    throw UrException("Failed to send the command `stop program` to robot");
  }
  if (validate)
  {
    try
    {
      waitFor(
          [this]() {
            return !consumer_->getRobotModeData()->is_program_running_ &&
                   !consumer_->getRobotModeData()->is_program_paused_;
          },
          timeout);
    }
    catch (const TimeoutException&)
    {
      throw TimeoutException("Robot did not stop the program within the given timeout", timeout);
    }
  }
}
std::shared_ptr<VersionInformation> PrimaryClient::getRobotVersion(bool blocking,
                                                                   const std::chrono::milliseconds timeout)
{
  if (blocking)
  {
    waitFor([this]() { return consumer_->getVersionInformation() != nullptr; }, timeout);
  }

  return consumer_->getVersionInformation();
}

RobotType PrimaryClient::getRobotType()
{
  std::shared_ptr<ConfigurationData> configuration_data = consumer_->getConfigurationData();
  if (configuration_data == nullptr)
  {
    return RobotType::UNDEFINED;
  }
  return static_cast<RobotType>(configuration_data->robot_type_);
}

RobotSeries PrimaryClient::getRobotSeries()
{
  auto robot_type = getRobotType();
  if (robot_type == RobotType::UNDEFINED)
  {
    return RobotSeries::UNDEFINED;
  }

  auto version_info = getRobotVersion();
  if (version_info == nullptr)
  {
    return RobotSeries::UNDEFINED;
  }

  return robotSeriesFromTypeAndVersion(robot_type, *version_info);
}

}  // namespace primary_interface
}  // namespace urcl
