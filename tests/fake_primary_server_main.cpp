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

#include <chrono>
#include <cstdlib>
#include <string>
#include <thread>

#include <ur_client_library/log.h>

#include "fake_primary_server.h"

int main(int argc, char* argv[])
{
  int port = urcl::primary_interface::UR_PRIMARY_PORT;
  if (argc > 1)
  {
    port = std::atoi(argv[1]);
  }

  urcl::FakePrimaryServer server(port);
  URCL_LOG_INFO("Fake primary server listening on port %d. Press Ctrl-C to quit.", server.getPort());

  // Send a version message every time a client connects, mirroring the real robot's behaviour.
  std::thread version_thread([&server]() {
    size_t last_count = 0;
    while (true)
    {
      const size_t count = server.getClientCount();
      if (count > last_count)
      {
        URCL_LOG_INFO("Client connected, sending VersionMessage.");
        server.sendVersionMessage();
      }
      last_count = count;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  version_thread.detach();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
