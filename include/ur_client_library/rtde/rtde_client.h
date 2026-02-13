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
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_RTDE_CLIENT_H_INCLUDED
#define UR_CLIENT_LIBRARY_RTDE_CLIENT_H_INCLUDED

#include <memory>

#include "ur_client_library/comm/producer.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/rtde/data_package.h"
#include "ur_client_library/rtde/rtde_package.h"
#include "ur_client_library/rtde/rtde_parser.h"
#include "ur_client_library/rtde/rtde_writer.h"

static const int UR_RTDE_PORT = 30004;

namespace urcl
{
namespace rtde_interface
{
static const uint16_t MAX_RTDE_PROTOCOL_VERSION = 2;
static const unsigned MAX_REQUEST_RETRIES = 5;

enum class UrRtdeRobotStatusBits
{
  IS_POWER_ON = 0,
  IS_PROGRAM_RUNNING = 1,
  IS_TEACH_BUTTON_PRESSED = 2,
  IS_POWER_BUTTON_PRESSED = 3
};

enum class UrRtdeSafetyStatusBits
{
  IS_NORMAL_MODE = 0,
  IS_REDUCED_MODE = 1,
  IS_PROTECTIVE_STOPPED = 2,
  IS_RECOVERY_MODE = 3,
  IS_SAFEGUARD_STOPPED = 4,
  IS_SYSTEM_EMERGENCY_STOPPED = 5,
  IS_ROBOT_EMERGENCY_STOPPED = 6,
  IS_EMERGENCY_STOPPED = 7,
  IS_VIOLATION = 8,
  IS_FAULT = 9,
  IS_STOPPED_DUE_TO_SAFETY = 10,
  IS_3PE_INPUT_ACTIVE = 11
};

enum class ClientState
{
  UNINITIALIZED = 0,
  INITIALIZING = 1,
  INITIALIZED = 2,
  RUNNING = 3,
  PAUSED = 4,
  CONNECTION_LOST = 5
};

/*!
 * \brief The RTDEClient class manages communication over the RTDE interface. It contains the RTDE
 * handshake and read and write functionality to and from the robot.
 */
class RTDEClient
{
public:
  RTDEClient() = delete;
  /*!
   * \brief Creates a new RTDEClient object, including a used URStream and Producer to handle the
   * communication with the robot.
   *
   * \param robot_ip The IP of the robot
   * \param notifier The notifier to notify of start and stop events
   * \param output_recipe_file Path to the file containing the output recipe
   * \param input_recipe_file Path to the file containing the input recipe
   * \param target_frequency Frequency to run at. Defaults to 0.0 which means maximum frequency.
   * \param ignore_unavailable_outputs Configure the behaviour when a variable of the output recipe is not available
   * \param port Optionally specify a different port
   * from the robot: output is silently ignored if true, a UrException is raised otherwise.
   */
  RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
             const std::string& input_recipe_file, double target_frequency = 0.0,
             bool ignore_unavailable_outputs = false, const uint32_t port = UR_RTDE_PORT);

  /*!
   * \brief Creates a new RTDEClient object, including a used URStream and Producer to handle the
   * communication with the robot.
   *
   * \param robot_ip The IP of the robot
   * \param notifier The notifier to notify of start and stop events
   * \param output_recipe Vector containing the output recipe
   * \param input_recipe Vector containing the input recipe
   * \param target_frequency Frequency to run at. Defaults to 0.0 which means maximum frequency.
   * \param ignore_unavailable_outputs Configure the behaviour when a variable of the output recipe is not available
   * \param port Optionally specify a different port
   * from the robot: output is silently ignored if true, a UrException is raised otherwise.
   */
  RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::vector<std::string>& output_recipe,
             const std::vector<std::string>& input_recipe, double target_frequency = 0.0,
             bool ignore_unavailable_outputs = false, const uint32_t port = UR_RTDE_PORT);
  ~RTDEClient();

  /*!
   * \brief Sets up RTDE communication with the robot. The handshake includes negotiation of the
   * used protocol version and setting of input and output recipes.
   *
   * \param max_connection_attempts Maximum number of (socket) connection attempts before counting the connection as
   * failed. Unlimited number of attempts when set to 0.
   * \param reconnection_timeout Time in between connection attempts to the socket
   * \param max_initialization_attempts Maximum number of initialization attempts before counting
   * the initialization as failed. Initialization can
   * fail given an established socket connection e.g. when the connected socket does not implement
   * an RTDE interface.
   * \param initialization_timeout Time in between initialization attempts of the RTDE interface
   *
   * \returns Success of the handshake
   */
  bool init(const size_t max_connection_attempts = 0,
            const std::chrono::milliseconds reconnection_timeout = comm::TCPSocket::DEFAULT_RECONNECTION_TIME,
            const size_t max_initialization_attempts = 3,
            const std::chrono::milliseconds initialization_timeout = std::chrono::seconds(1));
  /*!
   * \brief Triggers the robot to start sending RTDE data packages in the negotiated format.
   *
   * \param read_packages_in_background Whether to start a background thread to read packages from.
   * If packages are read in the background, getDataPackage() will return the latest one received.
   * If packages aren't read in the background, the application is required to call
   * getDataPackageBlocking() frequently.
   *
   * \returns Success of the requested start
   */
  bool start(const bool read_packages_in_background = true);

  /*!
   * \brief Pauses RTDE data package communication
   *
   * \returns Whether the RTDE data package communication was paused successfully
   */
  bool pause();

  /*!
   * \brief Return the latest data package received
   *
   * \deprecated This method allocates memory on each call. Please use the overload which takes a
   * reference to an existing DataPackage instead. This function will be removed in May 2027.
   *
   * When packages are read from the background thread, this will return the latest data package
   * received from the robot. When no new data has been received since the last call to this
   * function, it will wait for the time specified in the \p timeout parameter.
   *
   * When packages are not read from the background thread, this function will return nullptr and
   * print an error message.
   *
   * \param timeout Time to wait if no data package is currently in the queue
   *
   * \returns Unique ptr to the package, if a package was fetched successfully, nullptr otherwise
   */
  [[deprecated("This method allocates memory on each call. Please use the overload which takes a reference to an "
               "existing DataPackage instead. This function will be removed in May 2027.")]]
  std::unique_ptr<rtde_interface::DataPackage> getDataPackage(std::chrono::milliseconds timeout);

  /*!
   * \brief Return the latest data package received
   *
   * When packages are read from the background thread, the latest data package
   * received from the robot can be fetched with this. When no new data has been received since the last call to this
   * function, it will wait for the time specified in the \p timeout parameter.
   *
   * When packages are not read from the background thread, this function will return false and
   * print an error message.
   *
   * \param data_package Reference to a DataPackage where the received data package will be stored
   * if a package was fetched successfully.
   * \param timeout Time to wait if no data package is currently in the queue
   *
   * \returns Whether a data package was received successfully
   */
  bool getDataPackage(DataPackage& data_package, std::chrono::milliseconds timeout);
  bool getDataPackage(std::unique_ptr<DataPackage>& data_package, std::chrono::milliseconds timeout);

  /*!
   * \brief Blocking call to get the next data package received from the robot.
   *
   * This function will block until a new data package is received from the robot and return it.
   *
   * \param data_package Reference to a unique ptr where the received data package will be stored.
   * For optimal performance, the data package pointer should contain a pre-allocated data package
   * that was initialized with the same output recipe as used in this RTDEClient. If it is not an
   * initialized data package, a new one will be allocated internally which will have a negative
   * performance impact and print a warning.
   *
   * \returns Whether a data package was received successfully
   */
  bool getDataPackageBlocking(std::unique_ptr<rtde_interface::DataPackage>& data_package);

  /*!
   * \brief Getter for the maximum frequency the robot can publish RTDE data packages with.
   *
   * \returns The maximum frequency
   */
  double getMaxFrequency() const
  {
    return max_frequency_;
  }

  /*!
   * \brief Getter for the target frequency that the robot will publish RTDE data packages with.
   *
   * \returns The target frequency
   */
  double getTargetFrequency() const
  {
    return target_frequency_;
  }

  /*!
   * \brief Getter for the UR control version received from the robot.
   *
   * \returns The VersionInformation received from the robot
   */
  VersionInformation getVersion()
  {
    return urcontrol_version_;
  }

  /*!
   * \brief Returns the IP address (of the machine running this driver) used for the socket connection.
   *
   * \returns The IP address as a string (e.g. "192.168.0.1")
   */
  std::string getIP() const;

  /*!
   * \brief Getter for the RTDE writer, which is used to send data via the RTDE interface to the
   * robot.
   *
   * \returns A reference to the used RTDEWriter
   */
  RTDEWriter& getWriter();

  /*!
   * \brief Getter for the RTDE output recipe.
   *
   * \returns The output recipe
   */
  std::vector<std::string> getOutputRecipe()
  {
    return output_recipe_;
  }

  /*!
   * \brief Getter for the RTDE input recipe.
   *
   * \returns The input recipe
   */
  std::vector<std::string> getInputRecipe()
  {
    return input_recipe_;
  }

  /// Reads output or input recipe from a file and parses it into a vector of strings where each
  /// string is a line from the file.
  static std::vector<std::string> readRecipe(const std::string& recipe_file);

  ClientState getClientState() const
  {
    return client_state_;
  }

  /*! \brief Starts a background thread to read data packages from the robot.
   *
   * After calling this function, getDataPackage() can be used to get the latest data package
   * received from the robot.
   */
  void startBackgroundRead();

  /*! \brief Stops the background thread reading data packages from the robot.
   *
   * After calling this function, getDataPackageBlocking() must be used to get data packages
   * from the robot.
   *
   * \note When getDataPackageBlocking() is not called frequently enough, the internal buffer
   * of received data packages might fill up, causing the robot to shutdown the RTDE connection.
   */
  void stopBackgroundRead();

protected:
  comm::URStream<RTDEPackage> stream_;
  std::vector<std::string> output_recipe_;
  bool ignore_unavailable_outputs_;
  std::vector<std::string> input_recipe_;
  RTDEParser parser_;
  std::unique_ptr<comm::URProducer<RTDEPackage>> prod_;
  comm::INotifier notifier_;
  RTDEWriter writer_;
  std::atomic<bool> reconnecting_;
  std::atomic<bool> stop_reconnection_;
  std::mutex reconnect_mutex_;
  std::thread reconnecting_thread_;

  VersionInformation urcontrol_version_;

  double max_frequency_;
  double target_frequency_;

  std::unique_ptr<RTDEPackage> data_buffer0_;
  std::unique_ptr<RTDEPackage> data_buffer1_;
  std::mutex read_mutex_;
  std::mutex write_mutex_;
  std::atomic<bool> new_data_ = false;
  std::atomic<bool> background_read_running_ = false;
  std::thread background_read_thread_;
  std::condition_variable background_read_cv_;

  DataPackage preallocated_data_pkg_;

  ClientState client_state_;

  uint16_t protocol_version_;

  constexpr static const double CB3_MAX_FREQUENCY = 125.0;
  constexpr static const double URE_MAX_FREQUENCY = 500.0;

  // Helper function to ensure that timestamp is present in the output recipe. The timestamp is needed to ensure that
  // the robot is booted.
  std::vector<std::string> ensureTimestampIsPresent(const std::vector<std::string>& output_recipe) const;

  bool setupCommunication(const size_t max_num_tries = 0,
                          const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10));
  uint16_t negotiateProtocolVersion();
  bool queryURControlVersion();
  void setTargetFrequency();
  bool setupOutputs();
  bool setupInputs();
  void disconnect();

  /*!
   * \brief Updates the output recipe to the given one and recreates all the objects which depend on it.
   * It should only be called while setting up the communication.
   *
   * \param new_recipe the new output recipe to use
   */
  void resetOutputRecipe(const std::vector<std::string> new_recipe);

  /*!
   * \brief Checks whether the robot is booted, this is done by looking at the timestamp from the robot controller, this
   * will show the time in seconds since the controller was started. If the timestamp is below 40, we will read from
   * the stream for approximately 1 second to ensure that the RTDE interface is up and running. This will ensure that we
   * don't finalize setting up communication, before the controller is up and running. Else we could end up connecting
   * to the RTDE interface, before a restart occurs during robot boot which would then destroy the connection
   * established.
   *
   * \returns true if the robot is booted, false otherwise which will essentially trigger a reconnection.
   */
  bool isRobotBooted();
  bool sendStart();
  bool sendPause();

  /*!
   * \brief Reconnects to the RTDE interface and set the input and output recipes again.
   */
  void reconnect();
  void reconnectCallback();

  size_t max_connection_attempts_;
  std::chrono::milliseconds reconnection_timeout_;
  size_t max_initialization_attempts_;
  std::chrono::milliseconds initialization_timeout_;

  void backgroundReadThreadFunc();
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_RTDE_CLIENT_H_INCLUDED
