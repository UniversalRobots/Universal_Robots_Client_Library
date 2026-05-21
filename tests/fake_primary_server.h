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

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/primary/package_header.h"
#include "ur_client_library/primary/robot_message.h"
#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/primary/robot_state.h"
#include "ur_client_library/ur/datatypes.h"

namespace urcl
{
/*!
 * \brief A fake primary interface server that can be used in tests.
 *
 * The server is built on top of comm::TCPServer and exposes convenience functions for sending the
 * most common primary interface packages (version, key, text, runtime exception, safety mode,
 * error code, robot state sub-messages, ...). All messages are serialized using the same wire
 * format that the real UR controller uses, so existing primary interface clients can be pointed at
 * this server for unit testing.
 *
 * URScript code (or any other data) that the connected client sends to the server is captured and
 * can be inspected through the script callback or via \ref getLastReceivedScript.
 *
 * Multiple clients can be connected simultaneously. Sending always broadcasts to every connected
 * client.
 */
class FakePrimaryServer
{
public:
  /*!
   * \brief A single ROBOT_STATE sub-package, consisting of its sub-package type and the already
   * serialized payload (without the 4-byte sub-package size and the 1-byte type).
   */
  using RobotStateSubPackage = std::pair<primary_interface::RobotStateType, std::vector<uint8_t>>;

  FakePrimaryServer() = delete;

  /*!
   * \brief Construct a new fake primary server bound to \p port.
   *
   * If 0 is passed, the OS picks a free port that can be queried via \ref getPort.
   *
   * \param port Port to bind the server to. Defaults to the standard UR primary interface port.
   */
  explicit FakePrimaryServer(const int port = primary_interface::UR_PRIMARY_PORT);

  ~FakePrimaryServer();

  /*!
   * \brief Get the port this server is bound to.
   */
  int getPort() const
  {
    return server_.getPort();
  }

  /*!
   * \brief Get the number of currently connected clients.
   */
  size_t getClientCount() const;

  /*!
   * \brief Block until at least one client is connected, or the timeout expires.
   *
   * \returns true if a client is connected before the timeout, false otherwise.
   */
  bool waitForClient(const std::chrono::milliseconds timeout = std::chrono::seconds(1));

  // ===================== Generic send =====================

  /*!
   * \brief Send a fully formed primary package to all connected clients.
   *
   * The bytes are forwarded to the connected client(s) without modification. This is useful when
   * replaying real recordings of primary interface traffic.
   *
   * \param data Buffer holding the complete primary package (including its 4-byte length header).
   * \param size Number of bytes in \p data.
   *
   * \returns true if every connected client accepted the data, false if any write failed.
   */
  bool sendRaw(const uint8_t* data, size_t size);

  /*!
   * \brief Wrap the given payload as a ROBOT_MESSAGE and send it to all connected clients.
   *
   * The resulting on-wire package looks like:
   * \verbatim
   *   <4-byte size> <1-byte ROBOT_MESSAGE> <8-byte timestamp> <1-byte source>
   *   <1-byte message_type> <payload...>
   * \endverbatim
   *
   * \param message_type The robot message type.
   * \param payload Raw payload that follows the robot message header.
   * \param timestamp Timestamp to embed.
   * \param source Source byte to embed.
   */
  bool sendRobotMessage(primary_interface::RobotMessagePackageType message_type, const std::vector<uint8_t>& payload,
                        uint64_t timestamp = 0, uint8_t source = 0);

  /*!
   * \brief Wrap the given sub-packages as a ROBOT_STATE package and send it to all clients.
   *
   * The resulting on-wire package looks like:
   * \verbatim
   *   <4-byte size> <1-byte ROBOT_STATE>
   *     <4-byte sub-size> <1-byte sub-type> <sub-payload>
   *     ...
   * \endverbatim
   *
   * \param sub_packages List of (type, payload) tuples to include in the ROBOT_STATE.
   */
  bool sendRobotState(const std::vector<RobotStateSubPackage>& sub_packages);

  // ===================== Concrete RobotMessage helpers =====================

  /*!
   * \brief Send a VersionMessage.
   */
  bool sendVersionMessage(const std::string& project_name = "URControl", uint8_t major_version = 5,
                          uint8_t minor_version = 24, int32_t svn_version = 0, int32_t build_number = 0,
                          const std::string& build_date = "01-01-2026, 00:00:00");

  /*!
   * \brief Send a TextMessage with the given text.
   */
  bool sendTextMessage(const std::string& text);

  /*!
   * \brief Send a KeyMessage (used by the controller to e.g. signal that a program has
   * started/stopped).
   */
  bool sendKeyMessage(const std::string& title, const std::string& text, int32_t message_code = 0,
                      int32_t message_argument = 0);

  /*!
   * \brief Send a RuntimeExceptionMessage.
   */
  bool sendRuntimeExceptionMessage(uint32_t line_number, uint32_t column_number, const std::string& text);

  /*!
   * \brief Send a SafetyModeMessage.
   */
  bool sendSafetyModeMessage(SafetyMode safety_mode = SafetyMode::NORMAL, int32_t message_code = 0,
                             int32_t message_argument = 0, uint32_t report_data_type = 0, uint32_t report_data = 0);

  /*!
   * \brief Send an ErrorCodeMessage.
   */
  bool sendErrorCodeMessage(int32_t message_code, int32_t message_argument, primary_interface::ReportLevel report_level,
                            const std::string& text, uint32_t data_type = 0, uint32_t data = 0);

  // ===================== Concrete RobotState helpers =====================

  /*!
   * \brief Send a ROBOT_STATE package containing a single RobotModeData sub-message.
   *
   * Defaults correspond to a robot in RUNNING mode without any running program.
   */
  bool sendRobotModeData(RobotMode robot_mode = RobotMode::RUNNING, bool is_real_robot_connected = true,
                         bool is_real_robot_enabled = true, bool is_robot_power_on = true,
                         bool is_emergency_stopped = false, bool is_protective_stopped = false,
                         bool is_program_running = false, bool is_program_paused = false, uint8_t control_mode = 0,
                         double target_speed_fraction = 1.0, double speed_scaling = 1.0,
                         double target_speed_fraction_limit = 1.0);

  // ===================== Receive hooks =====================

  /*!
   * \brief Set a callback that will be called for every chunk of data received from a client.
   *
   * The primary interface accepts URScript code from clients. This callback exposes that data as a
   * string. Note that, because TCP is a stream, the chunks are not guaranteed to align with full
   * scripts; if you need full scripts use \ref getLastReceivedScript after sending one.
   */
  void setScriptCallback(std::function<void(const std::string&)> callback);

  /*!
   * \brief Return the most recent data received from a client as a string.
   */
  std::string getLastReceivedScript();

  /*!
   * \brief Clear the stored "last received script".
   */
  void clearLastReceivedScript();

private:
  void connectionCallback(const socket_t filedescriptor);
  void disconnectionCallback(const socket_t filedescriptor);
  void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv);

  // Build a complete primary package: 4-byte size + 1-byte type + body.
  static std::vector<uint8_t> buildPrimaryPackage(primary_interface::RobotPackageType type,
                                                  const std::vector<uint8_t>& body);

  // Build the body of a ROBOT_MESSAGE package: 8-byte timestamp + 1-byte source + 1-byte
  // message_type + payload.
  static std::vector<uint8_t> buildRobotMessageBody(uint64_t timestamp, uint8_t source,
                                                    primary_interface::RobotMessagePackageType message_type,
                                                    const std::vector<uint8_t>& payload);

  // Build the body of a ROBOT_STATE package by concatenating all sub-packages.
  static std::vector<uint8_t> buildRobotStateBody(const std::vector<RobotStateSubPackage>& sub_packages);

  comm::TCPServer server_;

  mutable std::mutex clients_mutex_;
  std::condition_variable clients_cv_;
  std::vector<socket_t> clients_;

  std::mutex script_mutex_;
  std::string last_received_script_;
  std::function<void(const std::string&)> script_callback_;
};

}  // namespace urcl
