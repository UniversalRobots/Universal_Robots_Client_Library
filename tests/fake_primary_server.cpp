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

#include "fake_primary_server.h"

#include <algorithm>

#include "ur_client_library/comm/package_serializer.h"
#include "ur_client_library/log.h"

namespace urcl
{

FakePrimaryServer::FakePrimaryServer(const int port) : server_(port)
{
  server_.setMessageCallback(std::bind(&FakePrimaryServer::messageCallback, this, std::placeholders::_1,
                                       std::placeholders::_2, std::placeholders::_3));
  server_.setConnectCallback(std::bind(&FakePrimaryServer::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&FakePrimaryServer::disconnectionCallback, this, std::placeholders::_1));
  server_.start();
}

FakePrimaryServer::~FakePrimaryServer()
{
  server_.shutdown();
}

size_t FakePrimaryServer::getClientCount() const
{
  std::lock_guard<std::mutex> lock(clients_mutex_);
  return clients_.size();
}

bool FakePrimaryServer::waitForClient(const std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(clients_mutex_);
  return clients_cv_.wait_for(lock, timeout, [this]() { return !clients_.empty(); });
}

void FakePrimaryServer::connectionCallback(const socket_t filedescriptor)
{
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.push_back(filedescriptor);
  }
  clients_cv_.notify_all();
  URCL_LOG_INFO("Client connected to fake primary server on FD %d", filedescriptor);
}

void FakePrimaryServer::disconnectionCallback(const socket_t filedescriptor)
{
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(std::remove(clients_.begin(), clients_.end(), filedescriptor), clients_.end());
  }
  URCL_LOG_INFO("Client disconnected from fake primary server on FD %d", filedescriptor);
}

void FakePrimaryServer::messageCallback([[maybe_unused]] const socket_t filedescriptor, char* buffer, int nbytesrecv)
{
  std::string received(buffer, static_cast<size_t>(nbytesrecv));
  std::function<void(const std::string&)> cb;
  {
    std::lock_guard<std::mutex> lock(script_mutex_);
    last_received_script_ = received;
    cb = script_callback_;
  }
  if (cb)
  {
    cb(received);
  }
}

bool FakePrimaryServer::sendRaw(const uint8_t* data, size_t size)
{
  std::vector<socket_t> clients_snapshot;
  {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_snapshot = clients_;
  }
  if (clients_snapshot.empty())
  {
    URCL_LOG_WARN("Fake primary server has no connected clients to send to.");
    return false;
  }
  bool all_ok = true;
  for (const socket_t fd : clients_snapshot)
  {
    size_t written = 0;
    if (!server_.write(fd, data, size, written) || written != size)
    {
      URCL_LOG_ERROR("Failed to write %zu bytes to client FD %d (wrote %zu)", size, fd, written);
      all_ok = false;
    }
  }
  return all_ok;
}

std::vector<uint8_t> FakePrimaryServer::buildPrimaryPackage(primary_interface::RobotPackageType type,
                                                            const std::vector<uint8_t>& body)
{
  const int32_t total_size = static_cast<int32_t>(sizeof(int32_t) + sizeof(uint8_t) + body.size());
  std::vector<uint8_t> packet(total_size);
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(packet.data() + offset, total_size);
  offset += comm::PackageSerializer::serialize(packet.data() + offset, static_cast<int8_t>(type));
  std::copy(body.begin(), body.end(), packet.begin() + offset);
  return packet;
}

std::vector<uint8_t> FakePrimaryServer::buildRobotMessageBody(uint64_t timestamp, uint8_t source,
                                                              primary_interface::RobotMessagePackageType message_type,
                                                              const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> body(sizeof(uint64_t) + sizeof(uint8_t) + sizeof(uint8_t) + payload.size());
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(body.data() + offset, timestamp);
  offset += comm::PackageSerializer::serialize(body.data() + offset, source);
  offset += comm::PackageSerializer::serialize(body.data() + offset, static_cast<uint8_t>(message_type));
  std::copy(payload.begin(), payload.end(), body.begin() + offset);
  return body;
}

std::vector<uint8_t> FakePrimaryServer::buildRobotStateBody(const std::vector<RobotStateSubPackage>& sub_packages)
{
  std::vector<uint8_t> body;
  for (const auto& sub : sub_packages)
  {
    const uint32_t sub_size = static_cast<uint32_t>(sizeof(uint32_t) + sizeof(uint8_t) + sub.second.size());
    const size_t old_size = body.size();
    body.resize(old_size + sub_size);
    size_t offset = old_size;
    offset += comm::PackageSerializer::serialize(body.data() + offset, sub_size);
    offset += comm::PackageSerializer::serialize(body.data() + offset, static_cast<uint8_t>(sub.first));
    std::copy(sub.second.begin(), sub.second.end(), body.begin() + offset);
  }
  return body;
}

bool FakePrimaryServer::sendRobotMessage(primary_interface::RobotMessagePackageType message_type,
                                         const std::vector<uint8_t>& payload, uint64_t timestamp, uint8_t source)
{
  std::vector<uint8_t> body = buildRobotMessageBody(timestamp, source, message_type, payload);
  std::vector<uint8_t> packet = buildPrimaryPackage(primary_interface::RobotPackageType::ROBOT_MESSAGE, body);
  return sendRaw(packet.data(), packet.size());
}

bool FakePrimaryServer::sendRobotState(const std::vector<RobotStateSubPackage>& sub_packages)
{
  std::vector<uint8_t> body = buildRobotStateBody(sub_packages);
  std::vector<uint8_t> packet = buildPrimaryPackage(primary_interface::RobotPackageType::ROBOT_STATE, body);
  return sendRaw(packet.data(), packet.size());
}

bool FakePrimaryServer::sendVersionMessage(const std::string& project_name, uint8_t major_version,
                                           uint8_t minor_version, int32_t svn_version, int32_t build_number,
                                           const std::string& build_date)
{
  std::vector<uint8_t> payload(sizeof(int8_t) + project_name.size() + sizeof(uint8_t) + sizeof(uint8_t) +
                               sizeof(int32_t) + sizeof(int32_t) + build_date.size());
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<int8_t>(project_name.size()));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, project_name);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, major_version);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, minor_version);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, svn_version);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, build_number);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, build_date);
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_VERSION, payload);
}

bool FakePrimaryServer::sendTextMessage(const std::string& text)
{
  std::vector<uint8_t> payload(text.size());
  if (!text.empty())
  {
    comm::PackageSerializer::serialize(payload.data(), text);
  }
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_TEXT, payload);
}

bool FakePrimaryServer::sendKeyMessage(const std::string& title, const std::string& text, int32_t message_code,
                                       int32_t message_argument)
{
  std::vector<uint8_t> payload(sizeof(int32_t) + sizeof(int32_t) + sizeof(uint8_t) + title.size() + text.size());
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_code);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_argument);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(title.size()));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, title);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, text);
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_KEY, payload);
}

bool FakePrimaryServer::sendRuntimeExceptionMessage(uint32_t line_number, uint32_t column_number,
                                                    const std::string& text)
{
  std::vector<uint8_t> payload(sizeof(uint32_t) + sizeof(uint32_t) + text.size());
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(payload.data() + offset, line_number);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, column_number);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, text);
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_RUNTIME_EXCEPTION, payload);
}

bool FakePrimaryServer::sendSafetyModeMessage(SafetyMode safety_mode, int32_t message_code, int32_t message_argument,
                                              uint32_t report_data_type, uint32_t report_data)
{
  std::vector<uint8_t> payload(sizeof(int32_t) + sizeof(int32_t) + sizeof(uint8_t) + sizeof(uint32_t) +
                               sizeof(uint32_t));
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_code);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_argument);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(safety_mode));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, report_data_type);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, report_data);
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_SAFETY_MODE, payload);
}

bool FakePrimaryServer::sendErrorCodeMessage(int32_t message_code, int32_t message_argument,
                                             primary_interface::ReportLevel report_level, const std::string& text,
                                             uint32_t data_type, uint32_t data)
{
  std::vector<uint8_t> payload(sizeof(int32_t) + sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t) +
                               sizeof(uint32_t) + text.size());
  size_t offset = 0;
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_code);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, message_argument);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<int32_t>(report_level));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, data_type);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, data);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, text);
  return sendRobotMessage(primary_interface::RobotMessagePackageType::ROBOT_MESSAGE_ERROR_CODE, payload);
}

bool FakePrimaryServer::sendRobotModeData(RobotMode robot_mode, bool is_real_robot_connected,
                                          bool is_real_robot_enabled, bool is_robot_power_on, bool is_emergency_stopped,
                                          bool is_protective_stopped, bool is_program_running, bool is_program_paused,
                                          uint8_t control_mode, double target_speed_fraction, double speed_scaling,
                                          double target_speed_fraction_limit)
{
  std::vector<uint8_t> payload(sizeof(uint64_t) + 7 * sizeof(uint8_t) + sizeof(int8_t) + sizeof(uint8_t) +
                               3 * sizeof(double));
  size_t offset = 0;
  uint64_t timestamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  offset += comm::PackageSerializer::serialize(payload.data() + offset, timestamp);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_real_robot_connected));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_real_robot_enabled));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_robot_power_on));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_emergency_stopped));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_protective_stopped));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_program_running));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<uint8_t>(is_program_paused));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, static_cast<int8_t>(robot_mode));
  offset += comm::PackageSerializer::serialize(payload.data() + offset, control_mode);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, target_speed_fraction);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, speed_scaling);
  offset += comm::PackageSerializer::serialize(payload.data() + offset, target_speed_fraction_limit);
  return sendRobotState({ { primary_interface::RobotStateType::ROBOT_MODE_DATA, payload } });
}

void FakePrimaryServer::setScriptCallback(std::function<void(const std::string&)> callback)
{
  std::lock_guard<std::mutex> lock(script_mutex_);
  script_callback_ = std::move(callback);
}

std::string FakePrimaryServer::getLastReceivedScript()
{
  std::lock_guard<std::mutex> lock(script_mutex_);
  return last_received_script_;
}

void FakePrimaryServer::clearLastReceivedScript()
{
  std::lock_guard<std::mutex> lock(script_mutex_);
  last_received_script_.clear();
}

}  // namespace urcl
