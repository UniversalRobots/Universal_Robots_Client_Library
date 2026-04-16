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
  std::cout << "Key message callback: " << msg.toString() << std::endl;
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
      return true;

    case SafetyMode::REDUCED:
      return true;

    case SafetyMode::RECOVERY:
      return true;

    // Safety mode might be unknown, as it is only updated on changes.
    case SafetyMode::UNDEFINED_SAFETY_MODE:
      return true;

    default:
      return false;
  }
}

bool PrimaryClient::sendScript(const std::string& program, std::string script_name, ScriptTypes script_type,
                               std::chrono::milliseconds timeout)
{
  ScriptInfo script_with_name = prepare_script(program, script_name, script_type);

  std::cout << script_with_name.script_code << std::endl;

  RobotMode robot_mode = getRobotMode();
  while (robot_mode == RobotMode::UNKNOWN)
  {
    URCL_LOG_INFO("Robot mode not received yet, waiting for it to be received.");
    std::chrono::milliseconds update_period(100);
    std::this_thread::sleep_for(update_period);
    robot_mode = getRobotMode();
  }
  if (robot_mode != RobotMode::RUNNING)
  {
    URCL_LOG_ERROR("Robot is not running, cannot execute script.");
    std::stringstream ss;
    ss << "Robot is in mode: " << urcl::robotModeString(robot_mode) << " (" << int(robot_mode) << ")";
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  if (!safetyModeAllowsExecution())
  {
    URCL_LOG_ERROR("Robot safety mode is not normal, cannot execute script.");
    std::stringstream ss;
    ss << "Robot safety mode is: " << safetyModeString(safety_mode) << " (" << unsigned(safety_mode) << ")";
    URCL_LOG_ERROR(ss.str().c_str());
  }
  uint64_t exception_timestamp = 0;
  {
    std::scoped_lock lock(runtime_exception_mutex_);
    if (latest_runtime_exception_ != nullptr)
    {
      exception_timestamp = latest_runtime_exception_->timestamp_;
    }
  }

  bool script_sent = sendScriptNoWrapping(script_with_name.script_code);
  if (!script_sent)
  {
    URCL_LOG_ERROR("Script could not be sent.");
    return false;
  }
  // No feedback from secondary programs, so we assume success
  if (script_type == ScriptTypes::SEC)
  {
    return true;
  }
  const auto script_start_time = std::chrono::system_clock::now();
  // Ignore start delay if it is 0
  bool script_started = timeout == std::chrono::milliseconds(0) ? true : false;
  while (true)
  {
    {
      std::scoped_lock lock(runtime_exception_mutex_);
      if (latest_runtime_exception_ != nullptr && latest_runtime_exception_->timestamp_ > exception_timestamp)
      {
        URCL_LOG_ERROR("Runtime exception occured during script execution");
        std::stringstream ss;
        ss << "Exception occured at line " << latest_runtime_exception_->line_number_ << ", column "
           << latest_runtime_exception_->column_number_ << "\n";
        // Debug print for the user
        auto script_lines = splitString(script_with_name.script_code, "\n");
        for (int i = 0; i < static_cast<int>(script_lines.size()); i++)
        {
          if (!script_lines[i].empty())
          {
            ss << script_lines[i] << "\n";
          }
          if (i == latest_runtime_exception_->line_number_ - 1)
          {
            for (int j = 0; j < latest_runtime_exception_->column_number_ - 1; j++)
            {
              ss << " ";
            }
            ss << "^\n";
          }
        }
        URCL_LOG_ERROR(ss.str().c_str());
        URCL_LOG_ERROR("Runtime exception text: %s", latest_runtime_exception_->text_.c_str());
        return false;
      }
    }

    auto errors = getErrorCodes();
    if (errors.size() > 0)
    {
      URCL_LOG_ERROR("Robot encountered error(s) during script execution, stopping program");
      for (auto error : errors)
      {
        URCL_LOG_ERROR("Robot error code: %s", error.to_string.c_str());
      }
      commandStop();
      return false;
    }

    {
      std::scoped_lock lock(key_message_queue_mutex_);
      if (key_message_queue_.size() > 0)
      {
        auto key_messages = key_message_queue_;
        key_message_queue_.clear();
        for (auto message : key_messages)
        {
          if (message.title_ == "PROGRAM_XXX_STOPPED" && message.text_ == script_with_name.script_name)
          {
            URCL_LOG_INFO("Script with name %s executed successfully", script_with_name.script_name.c_str());
            return true;
          }
          else if (!script_started && message.title_ == "PROGRAM_XXX_STARTED" &&
                   message.text_ == script_with_name.script_name)
          {
            URCL_LOG_INFO("Script with name %s started", script_with_name.script_name.c_str());
            script_started = true;
          }
          else  // Put irrelevant messages back in the queue
          {
            key_message_queue_.push_back(message);
          }
        }
      }
    }
    auto current_time = std::chrono::system_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - script_start_time);

    if (!script_started && elapsed_time > timeout)
    {
      URCL_LOG_ERROR("Script not started within timeout");
      return false;
    }
    std::chrono::milliseconds wait_period(10);
    std::this_thread::sleep_for(wait_period);
  }
}

std::vector<std::string> PrimaryClient::strip_comments_and_whitespace(std::vector<std::string> split_script)
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

ScriptInfo PrimaryClient::prepare_script(std::string script, std::string script_name, ScriptTypes script_type)
{
  // Validate script_name
  static const std::regex valid_name(R"(^[A-Za-z_][A-Za-z0-9_]*$)");
  if (!script_name.empty() && !std::regex_match(script_name, valid_name))
  {
    throw urcl::ScriptCodeSyntaxException("Invalid script name: '" + script_name +
                                          "'. Can only contain letters, numbers and underscores. First character must "
                                          "be a letter or "
                                          "underscore.");
  }
  // Split the given script in to separate lines
  std::vector<std::string> split_script = splitString(script, "\n");

  // Remove all comments and white-space-only lines
  std::vector<std::string> stripped_script = strip_comments_and_whitespace(split_script);

  // Use given scipt name or create one
  unsigned int current_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  std::string actual_script_name = script_name.size() != 0 ? script_name : "script_" + std::to_string(current_time);
  // Limit script name length to 31, to ensure backwards compatibility
  if (actual_script_name.size() > 31)
  {
    actual_script_name = actual_script_name.substr(0, 31);
  }

  // Is the script wrapped in a function definition? If not add one
  if (stripped_script[0].substr(0, 4).find("def ") == script.npos &&
      stripped_script[0].substr(0, 4).find("sec ") == script.npos)
  {
    // Assign appropriate type
    std::string type;
    switch (script_type)
    {
      case ScriptTypes::DEF:
        type = "def";
        break;
      case ScriptTypes::SEC:
        type = "sec";
        break;
    }

    std::string start = type + " " + actual_script_name + "():";
    std::string end = "end";
    // Add indentation to the existing script code
    for (std::size_t i = 0; i < stripped_script.size(); i++)
    {
      stripped_script[i] = "  " + stripped_script[i];
    }
    // Add function definition and end statement to the stripped script lines vector
    stripped_script.insert(stripped_script.begin(), start);
    stripped_script.push_back(end);
  }

  if (stripped_script.back().find("end") == script.npos)
  {
    throw urcl::ScriptCodeSyntaxException("Script contains either function definition or secondary process definition, "
                                          "but no 'end' "
                                          "term. Script is invalid.");
  }

  // Concatenate all the script lines in to the final script
  std::string prepared_script = "";
  for (auto line : stripped_script)
  {
    prepared_script.append(line + "\n");
  }

  // Return final script code as well as the name of the script as it will be exectuted
  return ScriptInfo(actual_script_name, prepared_script);
}

bool PrimaryClient::sendScriptNoWrapping(const std::string& program)
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
  if (!sendScriptNoWrapping("power on"))
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
  if (!sendScriptNoWrapping("power off"))
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
  if (!sendScriptNoWrapping("set robotmode run"))
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
  if (!sendScriptNoWrapping("set unlock protective stop"))
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

  if (!sendScriptNoWrapping("stop program"))
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

}  // namespace primary_interface
}  // namespace urcl
