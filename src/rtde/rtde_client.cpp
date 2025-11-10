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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/rtde/rtde_client.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/log.h"
#include <algorithm>
#include <chrono>

namespace urcl
{
namespace rtde_interface
{
RTDEClient::RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
                       const std::string& input_recipe_file, double target_frequency, bool ignore_unavailable_outputs)
  : stream_(robot_ip, UR_RTDE_PORT)
  , output_recipe_(ensureTimestampIsPresent(readRecipe(output_recipe_file)))
  , ignore_unavailable_outputs_(ignore_unavailable_outputs)
  , parser_(output_recipe_)
  , prod_(std::make_unique<comm::URProducer<RTDEPackage>>(stream_, parser_))
  , notifier_(notifier)
  , pipeline_(std::make_unique<comm::Pipeline<RTDEPackage>>(*prod_, PIPELINE_NAME, notifier, true))
  , writer_(&stream_, input_recipe_)
  , reconnecting_(false)
  , stop_reconnection_(false)
  , max_frequency_(URE_MAX_FREQUENCY)
  , target_frequency_(target_frequency)
  , client_state_(ClientState::UNINITIALIZED)
{
  if (!input_recipe_file.empty())
  {
    input_recipe_ = readRecipe(input_recipe_file);
    writer_.setInputRecipe(input_recipe_);
  }
}

RTDEClient::RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::vector<std::string>& output_recipe,
                       const std::vector<std::string>& input_recipe, double target_frequency,
                       bool ignore_unavailable_outputs)
  : stream_(robot_ip, UR_RTDE_PORT)
  , output_recipe_(ensureTimestampIsPresent(output_recipe))
  , ignore_unavailable_outputs_(ignore_unavailable_outputs)
  , input_recipe_(input_recipe)
  , parser_(output_recipe_)
  , prod_(std::make_unique<comm::URProducer<RTDEPackage>>(stream_, parser_))
  , notifier_(notifier)
  , pipeline_(std::make_unique<comm::Pipeline<RTDEPackage>>(*prod_, PIPELINE_NAME, notifier, true))
  , writer_(&stream_, input_recipe_)
  , reconnecting_(false)
  , stop_reconnection_(false)
  , max_frequency_(URE_MAX_FREQUENCY)
  , target_frequency_(target_frequency)
  , client_state_(ClientState::UNINITIALIZED)
{
}

RTDEClient::~RTDEClient()
{
  prod_->setReconnectionCallback(nullptr);
  stop_reconnection_ = true;
  if (reconnecting_thread_.joinable())
  {
    reconnecting_thread_.join();
  }
  disconnect();
}

bool RTDEClient::init(const size_t max_connection_attempts, const std::chrono::milliseconds reconnection_timeout,
                      const size_t max_initialization_attempts, const std::chrono::milliseconds initialization_timeout)
{
  if (max_initialization_attempts <= 0)
  {
    throw UrException("The number of initialization attempts has to be greater than 0.");
  }

  if (client_state_ > ClientState::UNINITIALIZED)
  {
    return true;
  }

  prod_->setReconnectionCallback(nullptr);

  unsigned int attempts = 0;
  std::stringstream ss;

  while (!setupCommunication(max_connection_attempts, reconnection_timeout))
  {
    if (++attempts >= max_initialization_attempts)
    {
      disconnect();
      ss << "Failed to initialize RTDE client after " << max_initialization_attempts << " attempts";
      throw UrException(ss.str());
    }
    // disconnect to start on a clean slate when trying to set up communication again
    disconnect();
    URCL_LOG_ERROR("Failed to initialize RTDE client, retrying in %d seconds", initialization_timeout.count() / 1000);
    std::this_thread::sleep_for(initialization_timeout);
  }
  // Stop pipeline again
  pipeline_->stop();
  client_state_ = ClientState::INITIALIZED;
  // Set reconnection callback after we are initialized to ensure that a disconnect during initialization doesn't
  // trigger a reconnect
  prod_->setReconnectionCallback(std::bind(&RTDEClient::reconnectCallback, this));
  return true;
}

bool RTDEClient::setupCommunication(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  // The state initializing is used inside disconnect to stop the pipeline again.
  client_state_ = ClientState::INITIALIZING;
  // A running pipeline is needed inside setup.
  try
  {
    pipeline_->init(max_num_tries, reconnection_time);
  }
  catch (const UrException& exc)
  {
    URCL_LOG_ERROR("Caught exception %s, while trying to initialize pipeline", exc.what());
    return false;
  }
  pipeline_->run();

  uint16_t protocol_version = negotiateProtocolVersion();
  // Protocol version must be above zero
  if (protocol_version == 0)
  {
    return false;
  }

  bool is_rtde_comm_setup = true;
  is_rtde_comm_setup = queryURControlVersion();

  if (is_rtde_comm_setup)
  {
    setTargetFrequency();
  }

  is_rtde_comm_setup = is_rtde_comm_setup && setupOutputs(protocol_version);

  is_rtde_comm_setup = is_rtde_comm_setup && isRobotBooted();

  if (input_recipe_.size() > 0)
  {
    is_rtde_comm_setup = is_rtde_comm_setup && setupInputs();
  }
  return is_rtde_comm_setup;
}

uint16_t RTDEClient::negotiateProtocolVersion()
{
  uint16_t protocol_version = MAX_RTDE_PROTOCOL_VERSION;
  while (protocol_version > 0)
  {
    // Protocol version should always be 1 before starting negotiation
    parser_.setProtocolVersion(1);
    unsigned int num_retries = 0;
    uint8_t buffer[4096];
    size_t size;
    size_t written;
    size = RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);
    if (!stream_.write(buffer, size, written))
    {
      URCL_LOG_ERROR("Sending protocol version query to robot failed");
      return 0;
    }

    while (num_retries < MAX_REQUEST_RETRIES)
    {
      std::unique_ptr<RTDEPackage> package;
      if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
      {
        URCL_LOG_ERROR("failed to get package from RTDE interface");
        return 0;
      }
      if (rtde_interface::RequestProtocolVersion* tmp_version =
              dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get()))
      {
        if (tmp_version->accepted_)
        {
          URCL_LOG_INFO("Negotiated RTDE protocol version to %hu.", protocol_version);
          parser_.setProtocolVersion(protocol_version);
          return protocol_version;
        }
        break;
      }
      else
      {
        std::stringstream ss;
        ss << "Did not receive protocol negotiation answer from robot. Message received instead: " << std::endl
           << package->toString() << ". Retrying...";
        num_retries++;
        URCL_LOG_WARN("%s", ss.str().c_str());
      }
    }

    URCL_LOG_INFO("Robot did not accept RTDE protocol version '%hu'. Trying lower protocol version", protocol_version);
    protocol_version--;
  }
  URCL_LOG_ERROR("Protocol version for RTDE communication could not be established. Robot didn't accept any of "
                 "the suggested versions.");
  return 0;
}

bool RTDEClient::queryURControlVersion()
{
  unsigned int num_retries = 0;
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = GetUrcontrolVersionRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Sending urcontrol version query request to robot failed");
    return false;
  }

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("No answer to urcontrol version query was received from robot");
      return false;
    }

    if (rtde_interface::GetUrcontrolVersion* tmp_urcontrol_version =
            dynamic_cast<rtde_interface::GetUrcontrolVersion*>(package.get()))
    {
      urcontrol_version_ = tmp_urcontrol_version->version_information_;
      return true;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive URControl version from robot. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      URCL_LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not query urcontrol version after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  return false;
}

void RTDEClient::setTargetFrequency()
{
  if (urcontrol_version_.major < 5)
  {
    max_frequency_ = CB3_MAX_FREQUENCY;
  }

  if (target_frequency_ == 0)
  {
    // Default to maximum frequency
    target_frequency_ = max_frequency_;
  }
  else if (target_frequency_ <= 0.0 || target_frequency_ > max_frequency_)
  {
    // Target frequency outside valid range
    throw UrException("Invalid target frequency of RTDE connection");
  }
}

void RTDEClient::resetOutputRecipe(const std::vector<std::string> new_recipe)
{
  disconnect();

  output_recipe_.assign(new_recipe.begin(), new_recipe.end());

  // Reset pipeline first otherwise we will segfault, if the producer object no longer exists, when destroying the
  // pipeline
  pipeline_.reset();

  parser_ = RTDEParser(output_recipe_);
  prod_ = std::make_unique<comm::URProducer<RTDEPackage>>(stream_, parser_);
  pipeline_ = std::make_unique<comm::Pipeline<RTDEPackage>>(*prod_, PIPELINE_NAME, notifier_, true);
}

bool RTDEClient::setupOutputs(const uint16_t protocol_version)
{
  unsigned int num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[65536];
  URCL_LOG_INFO("Setting up RTDE communication with frequency %f", target_frequency_);

  while (num_retries < MAX_REQUEST_RETRIES)
  {
    URCL_LOG_DEBUG("Sending output recipe");
    if (protocol_version == 2)
    {
      size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, target_frequency_, output_recipe_);
    }
    else
    {
      if (target_frequency_ != max_frequency_)
      {
        URCL_LOG_WARN("It is not possible to set a target frequency when using protocol version 1. A frequency "
                      "equivalent to the maximum frequency will be used instead.");
      }
      size = ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, output_recipe_);
    }

    // Send output recipe to robot
    if (!stream_.write(buffer, size, written))
    {
      URCL_LOG_ERROR("Could not send RTDE output recipe to robot");
      return false;
    }

    std::unique_ptr<RTDEPackage> package;
    if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Did not receive confirmation on RTDE output recipe");
      return false;
    }

    if (rtde_interface::ControlPackageSetupOutputs* tmp_output =
            dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(package.get()))

    {
      std::vector<std::string> variable_types = splitVariableTypes(tmp_output->variable_types_);
      std::vector<std::string> available_variables;
      std::vector<std::string> unavailable_variables;
      assert(output_recipe_.size() == variable_types.size());
      for (std::size_t i = 0; i < variable_types.size(); ++i)
      {
        const std::string variable_name = output_recipe_[i];
        URCL_LOG_DEBUG("%s confirmed as datatype: %s", variable_name.c_str(), variable_types[i].c_str());

        if (variable_types[i] == "NOT_FOUND")
        {
          unavailable_variables.push_back(variable_name);
        }
        else
        {
          available_variables.push_back(variable_name);
        }
      }

      if (!unavailable_variables.empty())
      {
        std::stringstream error_message;
        error_message << "The following variables are not recognized by the robot:";
        std::for_each(
            unavailable_variables.begin(), unavailable_variables.end(),
            [&error_message](const std::string& variable_name) { error_message << "\n  - '" << variable_name << "'"; });
        error_message << "\nEither your output recipe contains errors "
                         "or the urcontrol version does not support "
                         "them.";

        if (ignore_unavailable_outputs_)
        {
          error_message << " They will be removed from the output recipe.";
          URCL_LOG_WARN("%s", error_message.str().c_str());

          // Some variables are not available so retry setting up the communication with a stripped-down output recipe
          resetOutputRecipe(available_variables);
          return false;
        }
        else
        {
          URCL_LOG_ERROR("%s", error_message.str().c_str());
          throw UrException(error_message.str());
        }
      }
      else
      {
        // All variables are accounted for in the RTDE package
        return true;
      }
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE output setup. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      URCL_LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not setup RTDE outputs after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  URCL_LOG_ERROR(ss.str().c_str());
  return false;
}

bool RTDEClient::setupInputs()
{
  unsigned int num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  size = ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe_);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Could not send RTDE input recipe to robot");
    return false;
  }

  while (num_retries < MAX_REQUEST_RETRIES)
  {
    std::unique_ptr<RTDEPackage> package;
    if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Did not receive confirmation on RTDE input recipe");
      return false;
    }
    if (rtde_interface::ControlPackageSetupInputs* tmp_input =
            dynamic_cast<rtde_interface::ControlPackageSetupInputs*>(package.get()))

    {
      std::vector<std::string> variable_types = splitVariableTypes(tmp_input->variable_types_);
      assert(input_recipe_.size() == variable_types.size());
      for (std::size_t i = 0; i < variable_types.size(); ++i)
      {
        URCL_LOG_DEBUG("%s confirmed as datatype: %s", input_recipe_[i].c_str(), variable_types[i].c_str());
        if (variable_types[i] == "NOT_FOUND")
        {
          std::string message = "Variable '" + input_recipe_[i] +
                                "' not recognized by the robot. Probably your input recipe contains errors";
          throw UrException(message);
        }
        else if (variable_types[i] == "IN_USE")
        {
          std::string message = "Variable '" + input_recipe_[i] +
                                "' is currently controlled by another RTDE client. The input recipe can't be used as "
                                "configured";
          throw UrException(message);
        }
      }
      writer_.init(tmp_input->input_recipe_id_);

      return true;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE input setup. Message received instead: " << std::endl
         << package->toString() << ". Retrying...";
      num_retries++;
      URCL_LOG_WARN("%s", ss.str().c_str());
    }
  }
  std::stringstream ss;
  ss << "Could not setup RTDE inputs after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  URCL_LOG_ERROR(ss.str().c_str());
  return false;
}

void RTDEClient::disconnect()
{
  // If communication is started it should be paused before disconnecting
  if (client_state_ == ClientState::RUNNING)
  {
    pause();
  }
  if (client_state_ >= ClientState::INITIALIZING)
  {
    pipeline_->stop();
  }
  if (client_state_ > ClientState::UNINITIALIZED)
  {
    stream_.disconnect();
    writer_.stop();
  }
  client_state_ = ClientState::UNINITIALIZED;
}

bool RTDEClient::isRobotBooted()
{
  // We need  to trigger the robot to start sending RTDE data packages in the negotiated format, in order to read
  // the time since the controller was started.
  if (!sendStart())
    return false;

  std::unique_ptr<RTDEPackage> package;
  double timestamp = 0;
  int reading_count = 0;
  // During bootup the RTDE interface gets restarted once. If we connect to the RTDE interface before that happens, we
  // might end up in a situation where the RTDE connection is in an invalid state.
  // It should be fine if we manage to read from the RTDE interface for at least one second or if the robot has been up
  // for more then 40 seconds (During the reset the timestamp will also be reset to 0).
  // TODO (anyone): Find a better solution to check for a proper connection.

  while (timestamp < 40 && reading_count < target_frequency_ * 2)
  {
    // Set timeout based on target frequency, to make sure that reading doesn't timeout
    int timeout = static_cast<int>((1 / target_frequency_) * 1000) * 10;
    if (pipeline_->getLatestProduct(package, std::chrono::milliseconds(timeout)))
    {
      rtde_interface::DataPackage* tmp_input = dynamic_cast<rtde_interface::DataPackage*>(package.get());
      tmp_input->getData("timestamp", timestamp);
      reading_count++;
    }
    else
    {
      return false;
    }
  }

  // Pause connection again
  if (!sendPause())
    return false;

  return true;
}

bool RTDEClient::start()
{
  if (client_state_ == ClientState::RUNNING)
    return true;

  if (client_state_ == ClientState::UNINITIALIZED)
  {
    URCL_LOG_ERROR("Cannot start an unitialized client, please initialize it first");
    return false;
  }

  pipeline_->run();

  if (sendStart())
  {
    client_state_ = ClientState::RUNNING;
    return true;
  }
  else
  {
    return false;
  }
}

bool RTDEClient::pause()
{
  if (client_state_ == ClientState::PAUSED)
    return true;
  if (client_state_ != ClientState::RUNNING)
  {
    URCL_LOG_ERROR("Can't pause the client, as it hasn't been started");
    return false;
  }

  if (sendPause())
  {
    client_state_ = ClientState::PAUSED;
    return true;
  }
  else
  {
    return false;
  }
}

bool RTDEClient::sendStart()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = ControlPackageStartRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Sending RTDE start command failed!");
    return false;
  }

  std::unique_ptr<RTDEPackage> package;
  unsigned int num_retries = 0;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Could not get response to RTDE communication start request from robot");
      return false;
    }

    if (rtde_interface::ControlPackageStart* tmp = dynamic_cast<rtde_interface::ControlPackageStart*>(package.get()))
    {
      return tmp->accepted_;
    }
    else if (rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(package.get()))
    {
      // There is a race condition whether the last received packet was the start confirmation or
      // is already a data package. In that case consider the start as successful.
      double timestamp;
      return tmp->getData("timestamp", timestamp);
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE start request. Message received instead: " << std::endl
         << package->toString();
      URCL_LOG_WARN("%s", ss.str().c_str());
      num_retries++;
    }
  }
  std::stringstream ss;
  ss << "Could not start RTDE communication after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

bool RTDEClient::sendPause()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = ControlPackagePauseRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Sending RTDE pause command failed!");
    return false;
  }
  std::unique_ptr<RTDEPackage> package;
  std::chrono::time_point start = std::chrono::steady_clock::now();
  int seconds = 5;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds))
  {
    if (!pipeline_->getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Could not get response to RTDE communication pause request from robot");
      return false;
    }
    if (rtde_interface::ControlPackagePause* tmp = dynamic_cast<rtde_interface::ControlPackagePause*>(package.get()))
    {
      return tmp->accepted_;
    }
  }
  std::stringstream ss;
  ss << "Could not receive answer to pause RTDE communication after " << seconds << " seconds.";
  throw UrException(ss.str());
}

std::vector<std::string> RTDEClient::readRecipe(const std::string& recipe_file) const
{
  std::vector<std::string> recipe;
  std::ifstream file(recipe_file);
  if (file.fail())
  {
    std::stringstream msg;
    msg << "Opening file '" << recipe_file << "' failed with error: " << strerror(errno);
    URCL_LOG_ERROR("%s", msg.str().c_str());
    throw UrException(msg.str());
  }

  if (file.peek() == std::ifstream::traits_type::eof())
  {
    std::stringstream msg;
    msg << "The recipe '" << recipe_file << "' file is empty exiting ";
    URCL_LOG_ERROR("%s", msg.str().c_str());
    throw UrException(msg.str());
  }

  std::string line;
  while (std::getline(file, line))
  {
    recipe.push_back(line);
  }

  return recipe;
}

std::vector<std::string> RTDEClient::ensureTimestampIsPresent(const std::vector<std::string>& output_recipe) const
{
  // Add timestamp to rtde output recipe, if not already existing.
  // The timestamp is used to check if robot is booted or not.
  std::vector<std::string> recipe = output_recipe;
  const std::string timestamp = "timestamp";
  auto it = std::find(recipe.begin(), recipe.end(), timestamp);
  if (it == recipe.end())
  {
    recipe.push_back(timestamp);
  }
  return recipe;
}

std::unique_ptr<rtde_interface::DataPackage> RTDEClient::getDataPackage(std::chrono::milliseconds timeout)
{
  // Cannot get data packages while reconnecting as we could end up getting some of the configuration packages
  if (reconnect_mutex_.try_lock())
  {
    std::unique_ptr<RTDEPackage> urpackage;
    if (pipeline_->getLatestProduct(urpackage, timeout))
    {
      rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
      if (tmp != nullptr)
      {
        urpackage.release();
        reconnect_mutex_.unlock();
        return std::unique_ptr<rtde_interface::DataPackage>(tmp);
      }
    }
    reconnect_mutex_.unlock();
  }
  else
  {
    URCL_LOG_WARN("Unable to get data package while reconnecting to the RTDE interface");
    auto period = std::chrono::duration<double>(1.0 / target_frequency_);
    std::this_thread::sleep_for(period);
  }

  return std::unique_ptr<rtde_interface::DataPackage>(nullptr);
}

std::string RTDEClient::getIP() const
{
  return stream_.getIP();
}

RTDEWriter& RTDEClient::getWriter()
{
  return writer_;
}

std::vector<std::string> RTDEClient::splitVariableTypes(const std::string& variable_types) const
{
  std::vector<std::string> result;
  std::stringstream ss(variable_types);
  std::string substr = "";
  while (getline(ss, substr, ','))
  {
    result.push_back(substr);
  }
  return result;
}

void RTDEClient::reconnect()
{
  URCL_LOG_INFO("Reconnecting to the RTDE interface");
  // Locking mutex to ensure that calling getDataPackage doesn't influence the communication needed for reconfiguring
  // the RTDE connection
  std::lock_guard<std::mutex> lock(reconnect_mutex_);
  ClientState cur_client_state = client_state_;
  disconnect();

  const size_t max_initialization_attempts = 3;
  size_t cur_initialization_attempt = 0;
  bool client_reconnected = false;
  while (cur_initialization_attempt < max_initialization_attempts)
  {
    bool is_communication_setup = false;
    try
    {
      is_communication_setup = setupCommunication(1, std::chrono::milliseconds{ 10000 });
    }
    catch (const UrException& exc)
    {
      URCL_LOG_ERROR("Caught exception while reconnecting to the RTDE interface %s. Unable to reconnect", exc.what());
      disconnect();
      reconnecting_ = false;
      return;
    }

    const std::string reconnecting_stopped_msg = "Reconnecting has been stopped, because the object is being destroyed";
    if (stop_reconnection_)
    {
      URCL_LOG_WARN(reconnecting_stopped_msg.c_str());
      return;
    }

    if (is_communication_setup)
    {
      client_reconnected = true;
      break;
    }

    auto duration = std::chrono::seconds(1);
    if (stream_.getState() != comm::SocketState::Connected)
    {
      // We don't wanna count it as an initialization attempt if we cannot connect to the socket and we want to wait
      // longer before reconnecting.
      duration = std::chrono::seconds(10);
      URCL_LOG_ERROR("Failed to connect to the RTDE server, retrying in %i seconds", duration.count());
    }
    else
    {
      URCL_LOG_ERROR("Failed to initialize RTDE client, retrying in %i second", duration.count());
      cur_initialization_attempt += 1;
    }

    disconnect();

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < duration)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      if (stop_reconnection_)
      {
        URCL_LOG_WARN(reconnecting_stopped_msg.c_str());
        return;
      }
    }
  }

  if (client_reconnected == false)
  {
    URCL_LOG_ERROR("Failed to initialize RTDE client after %i attempts, unable to reconnect",
                   max_initialization_attempts);
    disconnect();
    reconnecting_ = false;
    return;
  }

  start();
  if (cur_client_state == ClientState::PAUSED)
  {
    pause();
  }
  URCL_LOG_INFO("Done reconnecting to the RTDE interface");
  reconnecting_ = false;
}

void RTDEClient::reconnectCallback()
{
  if (reconnecting_ || stop_reconnection_)
  {
    return;
  }
  if (reconnecting_thread_.joinable())
  {
    reconnecting_thread_.join();
  }
  reconnecting_ = true;
  reconnecting_thread_ = std::thread(&RTDEClient::reconnect, this);
}

}  // namespace rtde_interface
}  // namespace urcl
