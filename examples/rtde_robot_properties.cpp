// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <ur_client_library/rtde/rtde_client.h>

#include <iostream>

using namespace urcl;

const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
  const std::string robot_ip = argc > 1 ? argv[1] : DEFAULT_ROBOT_IP;

  comm::INotifier notifier;
  rtde_interface::RTDEClient client(robot_ip, notifier, std::vector<std::string>{ "timestamp" },
                                    std::vector<std::string>{});
  // The client reads the robot properties as part of init().
  if (!client.init())
  {
    std::cerr << "Could not connect to RTDE on " << robot_ip << std::endl;
    return 1;
  }

  // The robot properties are first available from software 10.15 (PolyScope X) and 5.26.3 (PolyScope 5).
  // Older controllers do not provide any. When getRobotProperties() succeeds, the software version
  // is always there; the other getters depend on the catalog for that version.
  rtde_interface::ReadProperties properties;
  if (!client.getRobotProperties(properties))
  {
    std::cout << "The controller did not provide any robot properties. They require software 10.15 or 5.26.3 "
                 "and newer."
              << std::endl;
    return 0;
  }

  if (const auto version = properties.getSoftwareVersion())
  {
    std::cout << "Software version: " << version->toString() << std::endl;
  }
  if (const auto control_box = properties.getControlBoxType())
  {
    std::cout << "Control box: " << control_box->toString() << std::endl;
  }
  if (const auto tool_flange = properties.getToolFlangeType())
  {
    std::cout << "Tool flange: " << tool_flange->toString() << std::endl;
  }
  return 0;
}
