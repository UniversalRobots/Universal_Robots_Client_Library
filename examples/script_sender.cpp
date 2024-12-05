// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2024 Universal Robots A/S
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
 * \author  Felix Exner feex@universal-robots.com
 * \date    2024-12-04
 *
 */
//----------------------------------------------------------------------

#include <iostream>
#include <thread>

#include <ur_client_library/control/script_sender.h>

constexpr uint32_t PORT = 12345;

int main(int argc, char* argv[])
{
  // Parse how may seconds to run
  int second_to_run = -1;
  if (argc > 2)
  {
    second_to_run = std::stoi(argv[2]);
  }
  urcl::control::ScriptSender sender(PORT, "textmsg(\"Hello, World!\")");

  std::cout << "Waiting for incoming requests on port " << PORT << "..." << std::endl;

  unsigned long startTime = clock();
  while (second_to_run < 0 || ((clock() - startTime) / CLOCKS_PER_SEC) < static_cast<unsigned int>(second_to_run))
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}