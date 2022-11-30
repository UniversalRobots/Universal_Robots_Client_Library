// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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

#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/comm/shell_consumer.h>
#include <ur_client_library/primary/primary_parser.h>

using namespace urcl;

class CalibrationConsumer : public urcl::comm::IConsumer<urcl::primary_interface::PrimaryPackage>
{
public:
  CalibrationConsumer() : calibrated_(0), have_received_data(false)
  {
  }
  virtual ~CalibrationConsumer() = default;

  virtual bool consume(std::shared_ptr<urcl::primary_interface::PrimaryPackage> product)
  {
    auto kin_info = std::dynamic_pointer_cast<urcl::primary_interface::KinematicsInfo>(product);
    if (kin_info != nullptr)
    {
      URCL_LOG_INFO("%s", product->toString().c_str());
      calibrated_ = kin_info->calibration_status_;
      have_received_data = true;
    }
    return true;
  }

  bool isCalibrated() const
  {
    const uint32_t LINEARIZED = 2;
    return calibrated_ == LINEARIZED;
  }

  bool calibrationStatusReceived()
  {
    return have_received_data;
  }

private:
  uint32_t calibrated_;
  bool have_received_data;
};

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

  // First of all, we need a stream that connects to the robot
  comm::URStream<primary_interface::PrimaryPackage> primary_stream(robot_ip, urcl::primary_interface::UR_PRIMARY_PORT);

  // This will parse the primary packages
  primary_interface::PrimaryParser parser;

  // The producer needs both, the stream and the parser to fully work
  comm::URProducer<primary_interface::PrimaryPackage> prod(primary_stream, parser);
  prod.setupProducer();

  // The calibration consumer will print the package contents to the shell
  CalibrationConsumer calib_consumer;

  // The notifer will be called at some points during connection setup / loss. This isn't fully
  // implemented atm.
  comm::INotifier notifier;

  // Now that we have all components, we can create and start the pipeline to run it all.
  comm::Pipeline<primary_interface::PrimaryPackage> calib_pipeline(prod, &calib_consumer, "Pipeline", notifier);
  calib_pipeline.run();

  // Package contents will be printed while not being interrupted
  // Note: Packages for which the parsing isn't implemented, will only get their raw bytes printed.
  while (!calib_consumer.calibrationStatusReceived())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (calib_consumer.isCalibrated())
  {
    printf("The robot on IP: %s is calibrated\n", robot_ip.c_str());
  }
  else
  {
    printf("The robot controller on IP: %s do not have a valid calibration\n", robot_ip.c_str());
    printf("Remeber to turn on the robot to get calibration stored on the robot!\n");
  }

  return 0;
}
