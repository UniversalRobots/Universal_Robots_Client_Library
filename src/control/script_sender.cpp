// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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
 * \date    2021-06-01
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/control/script_sender.h>

namespace urcl
{
namespace control
{
ScriptSender::ScriptSender(uint32_t port, const std::string& program)
  : server_(port), script_thread_(), program_(program)
{
  server_.setMessageCallback(
      std::bind(&ScriptSender::messageCallback, this, std::placeholders::_1, std::placeholders::_2));
  server_.setConnectCallback(std::bind(&ScriptSender::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&ScriptSender::disconnectionCallback, this, std::placeholders::_1));
  server_.start();
}

void ScriptSender::connectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_DEBUG("New client connected at FD %d.", filedescriptor);
}

void ScriptSender::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_DEBUG("Client at FD %d disconnected.", filedescriptor);
}

void ScriptSender::messageCallback(const socket_t filedescriptor, char* buffer)
{
  if (std::string(buffer) == PROGRAM_REQUEST_)
  {
    URCL_LOG_INFO("Robot requested program");
    sendProgram(filedescriptor);
  }
}

void ScriptSender::sendProgram(const socket_t filedescriptor)
{
  // urscripts (snippets) must end with a newline, or otherwise the controller's runtime will
  // not execute them. To avoid problems, we always just append a newline here, even if
  // there may already be one.
  const std::string send_string = program_ + "\n";
  size_t len = send_string.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(send_string.c_str());
  size_t written;

  if (server_.write(filedescriptor, data, len, written))
  {
    URCL_LOG_INFO("Sent program to robot");
  }
  else
  {
    URCL_LOG_ERROR("Could not send program to robot");
  }
}

}  // namespace control
}  // namespace urcl