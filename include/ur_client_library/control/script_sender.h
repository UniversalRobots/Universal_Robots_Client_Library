
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
 * \date    2019-05-22
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_SCRIPT_SENDER_H_INCLUDED
#define UR_CLIENT_LIBRARY_SCRIPT_SENDER_H_INCLUDED

#include <thread>
#include <string>

#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/log.h"

namespace urcl
{
namespace control
{
/*!
 * \brief The ScriptSender class starts a TCPServer for a robot to connect to and waits for a
 * request to receive a program. This program is then delivered to the requesting robot.
 */
class ScriptSender
{
public:
  ScriptSender() = delete;
  /*!
   * \brief Creates a ScriptSender object, including a new TCPServer
   *
   * \param port Port to start the server on
   * \param program Program to send to the robot upon request
   */
  ScriptSender(uint32_t port, const std::string& program);

private:
  comm::TCPServer server_;
  std::thread script_thread_;
  std::string program_;

  const std::string PROGRAM_REQUEST_ = std::string("request_program\n");

  void connectionCallback(const int filedescriptor);

  void disconnectionCallback(const int filedescriptor);

  void messageCallback(const int filedescriptor, char* buffer);

  void sendProgram(const int filedescriptor);
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_SCRIPT_SENDER_H_INCLUDED
