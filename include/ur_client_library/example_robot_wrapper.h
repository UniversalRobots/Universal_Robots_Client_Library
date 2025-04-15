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

#ifndef UR_CLIENT_LIBRARY_EXAMPLE_ROBOT_WRAPPER_H_INCLUDED
#define UR_CLIENT_LIBRARY_EXAMPLE_ROBOT_WRAPPER_H_INCLUDED

#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/log.h>

namespace urcl
{
/*!
 * \class ExampleRobotWrapper
 * \brief This class is a high-level abstraction around UrDriver and DashboardClient. It's main
 * purpose is to help us avoiding repetitive robot initialization code in our examples and tests.
 *
 * It is therefore not intended to be used in production code, but rather as a helper class for
 * developers. If you want to use this wrapper in your own code, please make sure to understand the
 * logic behind it and adjust it to your needs.
 *
 * Since this is mainly intended for internal use, don't count on the API being stable for this
 * class!
 */
class ExampleRobotWrapper
{
public:
  inline static const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
  inline static const std::string SCRIPT_FILE = "resources/external_control.urscript";

  ExampleRobotWrapper() = delete;
  /*!
   * \brief Construct a new Example Robot Wrapper object
   *
   * This will connect to a robot and initialize it. In headless mode the program will be running
   * instantly, in teach pendant mode the from \p autostart_program will be started.
   *
   * Note: RTDE communication has to be started separately.
   *
   * \param robot_ip IP address of the robot to connect to
   * \param output_recipe_file Output recipe file for RTDE communication
   * \param input_recipe_file Input recipe file for RTDE communication
   * \param headless_mode Should the driver be started in headless mode or not?
   * \param autostart_program Program to start automatically after initialization when not in
   * headless mode. This flag is ignored in headless mode.
   * \param script_file URScript file to send to the robot. That should be script code
   * communicating to the driver's reverse interface and trajectory interface.
   */
  ExampleRobotWrapper(const std::string& robot_ip, const std::string& output_recipe_file,
                      const std::string& input_recipe_file, const bool headless_mode = true,
                      const std::string& autostart_program = "", const std::string& script_file = SCRIPT_FILE);
  ~ExampleRobotWrapper();

  /**
   * @brief Initializes the robot in order to be able to start a program.
   *
   * The robot will be power-cycled once and end up switched on, breaks released.
   */
  bool initializeRobotWithDashboard();

  bool initializeRobotWithPrimaryClient();

  /**
   * @brief Starts RTDE communication with the robot.
   *
   * @param consume_data Once the RTDE client is started, it's data has to be consumed. If you
   * don't actually care about that data, this class can silently consume RTDE data when `true` is
   * passed. This can be stopped and started at any time using the startConsumingRTDEData() and
   * stopConsumingRTDEData() methods.
   */
  void startRTDECommununication(const bool consume_data = false);

  /**
   * @brief Start consuming RTDE data in the background.
   */
  void startConsumingRTDEData();

  /**
   * @brief Stop consuming RTDE data in the background. Note that data has to be consumed manually
   * using readDataPackage().
   */
  void stopConsumingRTDEData();

  /**
   * @brief Get the latest RTDE package.
   *
   * Do not call this, while RTDE data is being consumed in the background. In doubt, call
   * stopConsumingRTDEData() before calling this function.
   *
   * @param[out] data_pkg The data package will be stored in that object
   * @return true on a successful read, false if no package can be read or when RTDE data is
   * already being consumed in the background.
   */
  bool readDataPackage(std::unique_ptr<rtde_interface::DataPackage>& data_pkg);

  /**
   * @brief Blocks until there is a robot program connected to the driver's reverse interface or
   * until the timeout is hit.
   *
   * @param milliseconds How long to wait for a successful connection.
   * @return True on a successful connection, false if not connection could be detected before the
   * timeout.
   */
  bool waitForProgramRunning(int milliseconds = 100);

  /**
   * @brief Blocks until there is a disconnection event from the driver's reverse interface
   * detected or until the timeout is hit.
   *
   * @param milliseconds How long to wait for a disconnection.
   * @return True on a disconnection event has been detected, false if no event could be detected before the
   * timeout.
   */
  bool waitForProgramNotRunning(int milliseconds = 100);

  /**
   * @brief  Depending on whether it is headless or not start autostart_program or call driver's resendRobotProgram
   * function.
   *
   * @return True on successful program start, false otherwise.
   */
  bool resendRobotProgram();

  /**
   * @brief Start the program \p program_file_name on the robot.
   *
   * The program has be be present on the robot, otherwise this call does not succeed. The robot
   * needs to be in remote_control mode for this to work properly.
   *
   * @param program_file_name Filename on the robot including the ".urp" extension.
   * @return True on successful program start, false otherwise.
   */
  bool startRobotProgram(const std::string& program_file_name);

  /**
   * @brief Clear protective stop on the robot.
   *
   * This will try to clear a protective stop on the robot. If the robot is not in protective stop
   * this call will do nothing.
   */
  bool clearProtectiveStop();

  bool isHealthy() const;

  std::shared_ptr<DashboardClient> getDashboardClient() const
  {
    return dashboard_client_;
  };
  std::shared_ptr<primary_interface::PrimaryClient> getPrimaryClient() const
  {
    return primary_client_;
  };
  std::shared_ptr<UrDriver> getUrDriver() const
  {
    return ur_driver_;
  };

private:
  void handleRobotProgramState(bool program_running);

  //! Dashboard client to interact with the robot
  std::shared_ptr<urcl::DashboardClient> dashboard_client_;

  //! primary client to interact with the robot
  std::shared_ptr<urcl::primary_interface::PrimaryClient> primary_client_;

  //! UR driver to interact with the robot
  std::shared_ptr<urcl::UrDriver> ur_driver_;

  comm::INotifier notifier_;

  std::atomic<bool> rtde_communication_started_ = false;
  std::atomic<bool> consume_rtde_packages_ = false;
  std::mutex read_package_mutex_;
  std::unique_ptr<rtde_interface::DataPackage> data_pkg_;

  bool robot_initialized_ = false;

  bool program_running_;
  std::condition_variable program_running_cv_;
  std::condition_variable program_not_running_cv_;
  std::mutex program_running_mutex_;
  std::mutex program_not_running_mutex_;

  std::thread rtde_consumer_thread_;

  bool headless_mode_;
  std::string autostart_program_;
};
}  // namespace urcl

#endif
