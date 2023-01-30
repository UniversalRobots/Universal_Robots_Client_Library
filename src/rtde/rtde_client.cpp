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
#include <algorithm>

namespace urcl
{
namespace rtde_interface
{
RTDEClient::RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
                       const std::string& input_recipe_file, double target_frequency)
  : stream_(robot_ip, UR_RTDE_PORT)
  , output_recipe_(readRecipe(output_recipe_file))
  , input_recipe_(readRecipe(input_recipe_file))
  , parser_(output_recipe_)
  , prod_(stream_, parser_)
  , pipeline_(prod_, PIPELINE_NAME, notifier, true)
  , writer_(&stream_, input_recipe_)
  , max_frequency_(URE_MAX_FREQUENCY)
  , target_frequency_(target_frequency)
  , client_state_(ClientState::UNINITIALIZED)
{
}

RTDEClient::~RTDEClient()
{
  disconnect();
}

bool RTDEClient::init()
{
  if (client_state_ > ClientState::UNINITIALIZED)
  {
    return true;
  }

  unsigned int attempts = 0;
  while (attempts < MAX_INITIALIZE_ATTEMPTS)
  {
    setupCommunication();
    if (client_state_ == ClientState::INITIALIZED)
      return true;

    URCL_LOG_ERROR("Failed to initialize RTDE client, retrying in 10 seconds");
    std::this_thread::sleep_for(std::chrono::seconds(10));
    attempts++;
  }
  std::stringstream ss;
  ss << "Failed to initialize RTDE client after " << MAX_INITIALIZE_ATTEMPTS << " attempts";
  throw UrException(ss.str());
}

void RTDEClient::setupCommunication()
{
  client_state_ = ClientState::INITIALIZING;
  // A running pipeline is needed inside setup
  pipeline_.init();
  pipeline_.run();

  uint16_t protocol_version = MAX_RTDE_PROTOCOL_VERSION;
  while (!negotiateProtocolVersion(protocol_version) && client_state_ == ClientState::INITIALIZING)
  {
    URCL_LOG_INFO("Robot did not accept RTDE protocol version '%hu'. Trying lower protocol version", protocol_version);
    protocol_version--;
    if (protocol_version == 0)
    {
      throw UrException("Protocol version for RTDE communication could not be established. Robot didn't accept any of "
                        "the suggested versions.");
    }
  }
  if (client_state_ == ClientState::UNINITIALIZED)
    return;

  URCL_LOG_INFO("Negotiated RTDE protocol version to %hu.", protocol_version);
  parser_.setProtocolVersion(protocol_version);

  queryURControlVersion();
  if (client_state_ == ClientState::UNINITIALIZED)
    return;

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

  setupOutputs(protocol_version);
  if (client_state_ == ClientState::UNINITIALIZED)
    return;

  if (!isRobotBooted())
  {
    disconnect();
    return;
  }

  setupInputs();
  if (client_state_ == ClientState::UNINITIALIZED)
    return;

  // We finished communication for now
  pipeline_.stop();
  client_state_ = ClientState::INITIALIZED;
}

bool RTDEClient::negotiateProtocolVersion(const uint16_t protocol_version)
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
    URCL_LOG_ERROR("Sending protocol version query to robot failed, disconnecting");
    disconnect();
    return false;
  }

  while (num_retries < MAX_REQUEST_RETRIES)
  {
    std::unique_ptr<RTDEPackage> package;
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("failed to get package from rtde interface, disconnecting");
      disconnect();
      return false;
    }
    if (rtde_interface::RequestProtocolVersion* tmp_version =
            dynamic_cast<rtde_interface::RequestProtocolVersion*>(package.get()))
    {
      // Reset the num_tries variable in case we have to try with another protocol version.
      num_retries = 0;
      return tmp_version->accepted_;
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
  std::stringstream ss;
  ss << "Could not negotiate RTDE protocol version after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

void RTDEClient::queryURControlVersion()
{
  unsigned int num_retries = 0;
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  size = GetUrcontrolVersionRequest::generateSerializedRequest(buffer);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Sending urcontrol version query request to robot failed, disconnecting");
    disconnect();
    return;
  }

  std::unique_ptr<RTDEPackage> package;
  while (num_retries < MAX_REQUEST_RETRIES)
  {
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("No answer to urcontrol version query was received from robot, disconnecting");
      disconnect();
      return;
    }

    if (rtde_interface::GetUrcontrolVersion* tmp_urcontrol_version =
            dynamic_cast<rtde_interface::GetUrcontrolVersion*>(package.get()))
    {
      urcontrol_version_ = tmp_urcontrol_version->version_information_;
      return;
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
  std::stringstream ss;
  ss << "Could not query urcontrol version after " << MAX_REQUEST_RETRIES
     << " tries. Please check the output of the "
        "negotiation attempts above to get a hint what could be wrong.";
  throw UrException(ss.str());
}

void RTDEClient::setupOutputs(const uint16_t protocol_version)
{
  unsigned int num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  URCL_LOG_INFO("Setting up RTDE communication with frequency %f", target_frequency_);
  // Add timestamp to rtde output recipe, used to check if robot is booted
  const std::string timestamp = "timestamp";
  auto it = std::find(output_recipe_.begin(), output_recipe_.end(), timestamp);
  if (it == output_recipe_.end())
  {
    output_recipe_.push_back(timestamp);
  }
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
    URCL_LOG_ERROR("Could not send RTDE output recipe to robot, disconnecting");
    disconnect();
    return;
  }

  while (num_retries < MAX_REQUEST_RETRIES)
  {
    std::unique_ptr<RTDEPackage> package;
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Did not receive confirmation on RTDE output recipe, disconnecting");
      disconnect();
      return;
    }

    if (rtde_interface::ControlPackageSetupOutputs* tmp_output =
            dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(package.get()))

    {
      std::vector<std::string> variable_types = splitVariableTypes(tmp_output->variable_types_);
      assert(output_recipe_.size() == variable_types.size());
      for (std::size_t i = 0; i < variable_types.size(); ++i)
      {
        URCL_LOG_DEBUG("%s confirmed as datatype: %s", output_recipe_[i].c_str(), variable_types[i].c_str());
        if (variable_types[i] == "NOT_FOUND")
        {
          std::string message = "Variable '" + output_recipe_[i] +
                                "' not recognized by the robot. Probably your output recipe contains errors";
          throw UrException(message);
        }
      }
      return;
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
  throw UrException(ss.str());
}

void RTDEClient::setupInputs()
{
  unsigned int num_retries = 0;
  size_t size;
  size_t written;
  uint8_t buffer[4096];
  size = ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe_);
  if (!stream_.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Could not send RTDE input recipe to robot, disconnecting");
    disconnect();
    return;
  }

  while (num_retries < MAX_REQUEST_RETRIES)
  {
    std::unique_ptr<RTDEPackage> package;
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Did not receive confirmation on RTDE input recipe, disconnecting");
      disconnect();
      return;
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

      return;
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
  throw UrException(ss.str());
}

void RTDEClient::disconnect()
{
  // If communication is started it should be paused before disconnecting
  sendPause();
  pipeline_.stop();
  stream_.disconnect();
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
    if (pipeline_.getLatestProduct(package, std::chrono::milliseconds(timeout)))
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

  pipeline_.run();

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
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Could not get response to RTDE communication start request from robot");
      return false;
    }

    if (rtde_interface::ControlPackageStart* tmp = dynamic_cast<rtde_interface::ControlPackageStart*>(package.get()))
    {
      return tmp->accepted_;
    }
    else
    {
      std::stringstream ss;
      ss << "Did not receive answer to RTDE start request. Message received instead: " << std::endl
         << package->toString();
      URCL_LOG_WARN("%s", ss.str().c_str());
      return false;
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
    if (!pipeline_.getLatestProduct(package, std::chrono::milliseconds(1000)))
    {
      URCL_LOG_ERROR("Could not get response to RTDE communication pause request from robot");
      return false;
    }
    if (rtde_interface::ControlPackagePause* tmp = dynamic_cast<rtde_interface::ControlPackagePause*>(package.get()))
    {
      client_state_ = ClientState::PAUSED;
      return tmp->accepted_;
    }
  }
  std::stringstream ss;
  ss << "Could not receive answer to pause RTDE communication after " << seconds << " seconds.";
  throw UrException(ss.str());
}

std::vector<std::string> RTDEClient::readRecipe(const std::string& recipe_file)
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
  std::string line;
  while (std::getline(file, line))
  {
    recipe.push_back(line);
  }
  return recipe;
}

std::unique_ptr<rtde_interface::DataPackage> RTDEClient::getDataPackage(std::chrono::milliseconds timeout)
{
  std::unique_ptr<RTDEPackage> urpackage;
  if (pipeline_.getLatestProduct(urpackage, timeout))
  {
    rtde_interface::DataPackage* tmp = dynamic_cast<rtde_interface::DataPackage*>(urpackage.get());
    if (tmp != nullptr)
    {
      urpackage.release();
      return std::unique_ptr<rtde_interface::DataPackage>(tmp);
    }
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
}  // namespace rtde_interface
}  // namespace urcl
