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

#include <ur_client_library/primary/primary_client.h>

using namespace urcl;

// Create a primary consumer for logging calibration data
class CalibrationConsumer : public primary_interface::AbstractPrimaryConsumer
{
public:
  CalibrationConsumer() : calibrated_(0), have_received_data(false)
  {
  }
  virtual ~CalibrationConsumer() = default;

  // We should consume all primary packages supported, in this example we just ignore the messages
  virtual bool consume(primary_interface::RobotMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::RobotState& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::ProgramStateMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::VersionMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::ErrorCodeMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::RuntimeExceptionMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::KeyMessage& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::RobotModeData& pkg) override
  {
    return true;
  }
  virtual bool consume(primary_interface::TextMessage& pkg) override
  {
    return true;
  }

  // The kinematics info stores the calibration data
  virtual bool consume(primary_interface::KinematicsInfo& pkg) override
  {
    calibrated_ = pkg.calibration_status_;
    have_received_data = true;
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

// The purpose of this example is to show how to add a primary consumer to the primary client. This consumer is used to
// check that the robot is calibrated.
int main(int argc, char* argv[])
{
  // Set the loglevel to info to print out the DH parameters
  urcl::setLogLevel(urcl::LogLevel::INFO);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  // Create a primary client
  primary_interface::PrimaryClient primary_client(robot_ip);

  std::shared_ptr<CalibrationConsumer> calibration_consumer;
  calibration_consumer.reset(new CalibrationConsumer());

  // Add the calibration consumer to the primary consumers
  primary_client.addPrimaryConsumer(calibration_consumer);

  // Kinematics info is only send when you connect to the primary interface, so triggering a reconnect
  primary_client.reconnect();

  while (!calibration_consumer->calibrationStatusReceived())
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  if (calibration_consumer->isCalibrated())
  {
    printf("The robot on IP: %s is calibrated\n", robot_ip.c_str());
  }
  else
  {
    printf("The robot controller on IP: %s do not have a valid calibration\n", robot_ip.c_str());
    printf("Remeber to turn on the robot to get calibration stored on the robot!\n");
  }

  // We can remove the consumer again once we are done using it
  primary_client.removePrimaryConsumer(calibration_consumer);

  return 0;
}
