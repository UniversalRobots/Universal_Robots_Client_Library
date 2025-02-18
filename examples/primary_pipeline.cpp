// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-09-11
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/shell_consumer.h>
#include <ur_client_library/primary/primary_parser.h>

using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a better configuration
// system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.56.101";

int main(int argc, char* argv[])
{
  // Set the loglevel to info get print out the DH parameters
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Parse how many seconds to run
  int second_to_run = -1;
  if (argc > 2)
  {
    second_to_run = std::stoi(argv[2]);
  }

  // First of all, we need a stream that connects to the robot's primary interface
  comm::URStream<primary_interface::PrimaryPackage> primary_stream(robot_ip, urcl::primary_interface::UR_PRIMARY_PORT);

  // This will parse the primary packages
  primary_interface::PrimaryParser parser;

  // The producer needs both, the stream and the parser to fully work
  comm::URProducer<primary_interface::PrimaryPackage> prod(primary_stream, parser);

  // Connect to the stream
  prod.setupProducer();

  // The shell consumer will print the package contents to the shell
  auto consumer = std::make_unique<comm::ShellConsumer<primary_interface::PrimaryPackage>>();

  // The notifer will be called at some points during connection setup / loss. This isn't fully
  // implemented atm.
  comm::INotifier notifier;

  // Now that we have all components, we can create and start the pipeline to run it all.
  comm::Pipeline<primary_interface::PrimaryPackage> pipeline(prod, consumer.get(), "Pipeline", notifier);
  pipeline.run();

  // Package contents will be printed while not being interrupted
  // Note: Packages for which the parsing isn't implemented, will only get their raw bytes printed.
  do
  {
    std::this_thread::sleep_for(std::chrono::seconds(second_to_run));
  } while (second_to_run < 0);
  return 0;
}
