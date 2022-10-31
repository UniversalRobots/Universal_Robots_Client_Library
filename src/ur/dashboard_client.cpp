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

#include <regex>
#include <unistd.h>
#include <ur_client_library/log.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/exceptions.h>

namespace urcl
{
DashboardClient::DashboardClient(const std::string& host) : host_(host), port_(DASHBOARD_SERVER_PORT)
{
}

void DashboardClient::rtrim(std::string& str, const std::string& chars)
{
  str.erase(str.find_last_not_of(chars) + 1);
}

bool DashboardClient::connect()
{
  if (getState() == comm::SocketState::Connected)
  {
    URCL_LOG_ERROR("%s", "Socket is already connected. Refusing to reconnect.");
    return false;
  }
  bool ret_val = false;
  if (TCPSocket::setup(host_, port_))
  {
    URCL_LOG_INFO("%s", read().c_str());
    ret_val = true;
  }

  timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  TCPSocket::setReceiveTimeout(tv);

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
  std::string response = "ERROR";
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (send(text))
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
  std::string response = sendAndReceive(command + "\n");
  bool ret = std::regex_match(response, std::regex(expected));
  if (!ret)
  {
    URCL_LOG_WARN("Expected: \"%s\", but received: \"%s\"", expected.c_str(), response.c_str());
  }
  return ret;
}

bool DashboardClient::waitForReply(const std::string& command, const std::string& expected, double timeout)
{
  const unsigned int TIME_STEP_SIZE_US(100000);  // 100ms

  double count = 0;
  std::string response;

  while (count < timeout)
  {
    // Send the request
    response = sendAndReceive(command + "\n");

    // Check it the response was as expected
    if (std::regex_match(response, std::regex(expected)))
    {
      return true;
    }

    // wait 100ms before trying again
    usleep(TIME_STEP_SIZE_US);
    count = count + (0.000001 * TIME_STEP_SIZE_US);
  }

  URCL_LOG_WARN("Did not got the expected \"%s\" respone within the timeout. Last respone was: \"%s\"",
                expected.c_str(), response.c_str());
  return false;
}

bool DashboardClient::retryCommand(const std::string& requestCommand, const std::string& requestExpectedResponse,
                                   const std::string& waitRequest, const std::string& waitExpectedResponse,
                                   unsigned int timeout)
{
  const double RETRY_EVERY_SECOND(1.0);
  unsigned int count(0);
  do
  {
    sendRequest(requestCommand, requestExpectedResponse);
    count++;

    if (waitForReply(waitRequest, waitExpectedResponse, RETRY_EVERY_SECOND))
    {
      return true;
    }
  } while (count < timeout);
  return false;
}

bool DashboardClient::commandPowerOff()
{
  return sendRequest("power off", "Powering off") && waitForReply("robotmode", "Robotmode: POWER_OFF");
}

bool DashboardClient::commandPowerOn(unsigned int timeout)
{
  return retryCommand("power on", "Powering on", "robotmode", "Robotmode: IDLE", timeout);
}

bool DashboardClient::commandBrakeRelease()
{
  return sendRequest("brake release", "Brake releasing") && waitForReply("robotmode", "Robotmode: RUNNING");
}

bool DashboardClient::commandLoadProgram(const std::string& program_file_name)
{
  return sendRequest("load " + program_file_name + "", "(?:Loading program: ).*(?:" + program_file_name + ").*") &&
         waitForReply("programState", "STOPPED " + program_file_name);
}

bool DashboardClient::commandPlay()
{
  return sendRequest("play", "Starting program") && waitForReply("programState", "(?:PLAYING ).*");
}

bool DashboardClient::commandPause()
{
  return sendRequest("pause", "Pausing program") && waitForReply("programState", "(?:PAUSED ).*");
}

bool DashboardClient::commandStop()
{
  return sendRequest("stop", "Stopped") && waitForReply("programState", "(?:STOPPED ).*");
}

bool DashboardClient::commandClosePopup()
{
  return sendRequest("close popup", "closing popup");
}

bool DashboardClient::commandCloseSafetyPopup()
{
  return sendRequest("close safety popup", "closing safety popup");
}

bool DashboardClient::commandRestartSafety()
{
  return sendRequest("restart safety", "Restarting safety") && waitForReply("robotmode", "Robotmode: POWER_OFF");
}

bool DashboardClient::commandUnlockProtectiveStop()
{
  return sendRequest("unlock protective stop", "Protective stop releasing");
}

bool DashboardClient::commandShutdown()
{
  return sendRequest("shutdown", "Shutting down");
}

}  // namespace urcl
