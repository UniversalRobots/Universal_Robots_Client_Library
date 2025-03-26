// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <ur_client_library/example_robot_wrapper.h>
#include <iostream>
#include "ur_client_library/exceptions.h"
#include "ur_client_library/log.h"
#include "ur_client_library/ur/version_information.h"

namespace urcl
{
ExampleRobotWrapper::ExampleRobotWrapper(const std::string& robot_ip, const std::string& output_recipe_file,
                                         const std::string& input_recipe_file, const bool headless_mode,
                                         const std::string& autostart_program, const std::string& script_file)
  : headless_mode_(headless_mode), autostart_program_(autostart_program)
{
  primary_client_ = std::make_shared<urcl::primary_interface::PrimaryClient>(robot_ip, notifier_);

  primary_client_->start();

  auto robot_version = primary_client_->getRobotVersion();
  if (*robot_version < VersionInformation::fromString("10.0.0"))
  {
    dashboard_client_ = std::make_shared<DashboardClient>(robot_ip);
    // Connect the robot Dashboard
    if (!dashboard_client_->connect())
    {
      URCL_LOG_ERROR("Could not connect to dashboard");
    }

    // In CI we the dashboard client times out for no obvious reason. Hence we increase the timeout
    // here.
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    dashboard_client_->setReceiveTimeout(tv);
  }

  if (!initializeRobotWithPrimaryClient())
  {
    throw UrException("Could not initialize robot with primary client");
  }

  UrDriverConfiguration driver_config;
  driver_config.robot_ip = robot_ip;
  driver_config.script_file = script_file;
  driver_config.output_recipe_file = output_recipe_file;
  driver_config.input_recipe_file = input_recipe_file;
  driver_config.handle_program_state =
      std::bind(&ExampleRobotWrapper::handleRobotProgramState, this, std::placeholders::_1);
  driver_config.headless_mode = headless_mode;
  ur_driver_ = std::make_shared<UrDriver>(driver_config);

  if (!headless_mode && !std::empty(autostart_program))
  {
    startRobotProgram(autostart_program);
  }

  if (headless_mode || !std::empty(autostart_program))
  {
    if (!waitForProgramRunning(500))
    {
      throw UrException("Program did not start running. Is the robot in remote control?");
    }
  }
}

ExampleRobotWrapper::~ExampleRobotWrapper()
{
  if (rtde_communication_started_)
  {
    stopConsumingRTDEData();
  }
}

bool ExampleRobotWrapper::clearProtectiveStop()
{
  if (primary_client_->isRobotProtectiveStopped())
  {
    URCL_LOG_INFO("Robot is in protective stop, trying to release it");
    if (dashboard_client_ != nullptr)
    {
      dashboard_client_->commandClosePopup();
      dashboard_client_->commandCloseSafetyPopup();
    }
    try
    {
      primary_client_->commandUnlockProtectiveStop();
    }
    catch (const TimeoutException&)
    {
      std::this_thread::sleep_for(std::chrono::seconds(5));
      try
      {
        primary_client_->commandUnlockProtectiveStop();
      }
      catch (const TimeoutException&)
      {
        URCL_LOG_ERROR("Robot could not unlock the protective stop");
        return false;
      }
    }
  }
  return true;
}

bool ExampleRobotWrapper::initializeRobotWithDashboard()
{
  if (!clearProtectiveStop())
  {
    URCL_LOG_ERROR("Could not clear protective stop");
    return false;
  }

  // Stop program, if there is one running
  if (!dashboard_client_->commandStop())
  {
    URCL_LOG_ERROR("Could not send stop program command");
    return false;
  }

  // Power it off
  if (!dashboard_client_->commandPowerOff())
  {
    URCL_LOG_ERROR("Could not send Power off command");
    return false;
  }

  // Power it on
  if (!dashboard_client_->commandPowerOn())
  {
    URCL_LOG_ERROR("Could not send Power on command");
    return false;
  }

  // Release the brakes
  if (!dashboard_client_->commandBrakeRelease())
  {
    URCL_LOG_ERROR("Could not send BrakeRelease command");
    return false;
  }

  // Now the robot is ready to receive a program
  URCL_LOG_INFO("Robot ready to start a program");
  robot_initialized_ = true;
  return true;
}

bool ExampleRobotWrapper::initializeRobotWithPrimaryClient()
{
  try
  {
    waitFor([&]() { return primary_client_->getRobotModeData() != nullptr; }, std::chrono::seconds(5));
    clearProtectiveStop();
  }
  catch (const std::exception& exc)
  {
    URCL_LOG_ERROR("Could not clear protective stop (%s)", exc.what());
    return false;
  }

  try
  {
    primary_client_->commandStop();
    primary_client_->commandBrakeRelease();
  }
  catch (const TimeoutException& exc)
  {
    URCL_LOG_ERROR(exc.what());
    return false;
  }

  // Now the robot is ready to receive a program
  URCL_LOG_INFO("Robot ready to start a program");
  robot_initialized_ = true;
  return true;
}

void ExampleRobotWrapper::handleRobotProgramState(bool program_running)
{
  // Print the text in green so we see it better
  std::cout << "\033[1;32mProgram running: " << std::boolalpha << program_running << "\033[0m\n" << std::endl;
  if (program_running)
  {
    std::lock_guard<std::mutex> lk(program_running_mutex_);
    program_running_ = program_running;
    program_running_cv_.notify_one();
  }
  else
  {
    std::lock_guard<std::mutex> lk(program_not_running_mutex_);
    program_running_ = program_running;
    program_not_running_cv_.notify_one();
  }
}

void ExampleRobotWrapper::startRTDECommununication(const bool consume_data)
{
  if (!rtde_communication_started_)
  {
    ur_driver_->startRTDECommunication();
    rtde_communication_started_ = true;
  }
  if (consume_data)
  {
    startConsumingRTDEData();
  }
}

void ExampleRobotWrapper::startConsumingRTDEData()
{
  consume_rtde_packages_ = true;
  rtde_consumer_thread_ = std::thread([this]() {
    while (consume_rtde_packages_)
    {
      // Consume package to prevent pipeline overflow
      std::lock_guard<std::mutex> lk(read_package_mutex_);
      data_pkg_ = ur_driver_->getDataPackage();
    }
  });
}

void ExampleRobotWrapper::stopConsumingRTDEData()
{
  if (consume_rtde_packages_)
  {
    consume_rtde_packages_ = false;
    if (rtde_consumer_thread_.joinable())
    {
      rtde_consumer_thread_.join();
    }
  }
}

bool ExampleRobotWrapper::readDataPackage(std::unique_ptr<rtde_interface::DataPackage>& data_pkg)
{
  if (consume_rtde_packages_ == true)
  {
    URCL_LOG_ERROR("Unable to read packages while consuming, this should not happen!");
    return false;
  }
  std::lock_guard<std::mutex> lk(read_package_mutex_);
  data_pkg = ur_driver_->getDataPackage();
  if (data_pkg == nullptr)
  {
    URCL_LOG_ERROR("Timed out waiting for a new package from the robot");
    return false;
  }
  return true;
}

bool ExampleRobotWrapper::waitForProgramRunning(int milliseconds)
{
  std::unique_lock<std::mutex> lk(program_running_mutex_);
  if (program_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
      program_running_ == true)
  {
    return true;
  }
  return false;
}

bool ExampleRobotWrapper::waitForProgramNotRunning(int milliseconds)
{
  std::unique_lock<std::mutex> lk(program_not_running_mutex_);
  if (program_not_running_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
      program_running_ == false)
  {
    return true;
  }
  return false;
}

bool ExampleRobotWrapper::startRobotProgram(const std::string& program_file_name)
{
  if (dashboard_client_ != nullptr)
  {
    if (!dashboard_client_->commandLoadProgram(program_file_name))
    {
      URCL_LOG_ERROR("Could not load program '%s'", program_file_name.c_str());
      return false;
    }

    return dashboard_client_->commandPlay();
  }
  URCL_LOG_ERROR("Dashboard client is not initialized. If you are running a PolyScope X robot, the dashboard server is "
                 "not available. Loading and running polyscope programs isn't possible. Please use the headless mode "
                 "or the teach pendant instead.");
  return false;
}
bool ExampleRobotWrapper::resendRobotProgram()
{
  if (headless_mode_)
  {
    return ur_driver_->sendRobotProgram();
  }
  return startRobotProgram(autostart_program_);
}

bool ExampleRobotWrapper::isHealthy() const
{
  if (!robot_initialized_)
  {
    URCL_LOG_ERROR("Robot is not initialized");
    return false;
  }
  if (!program_running_)
  {
    URCL_LOG_ERROR("Robot program is not running");
    return false;
  }
  return true;
}

}  // namespace urcl
