#include "fake_rtde_server.h"
#include <stdexcept>
#include "ur_client_library/comm/package_serializer.h"
#include "ur_client_library/log.h"

namespace urcl
{

RTDEServer::RTDEServer(const int port) : server_(port)
{
  start_time_ = std::chrono::steady_clock::now();
  server_.setMessageCallback(std::bind(&RTDEServer::messageCallback, this, std::placeholders::_1, std::placeholders::_2,
                                       std::placeholders::_3));
  server_.setConnectCallback(std::bind(&RTDEServer::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&RTDEServer::disconnectionCallback, this, std::placeholders::_1));
  server_.setMaxClientsAllowed(1);
  server_.start();
}

RTDEServer::~RTDEServer()
{
  stopSendingDataPackages();
}

void RTDEServer::connectionCallback(const socket_t filedescriptor)
{
  client_socket_ = filedescriptor;
  URCL_LOG_INFO("Client connected to RTDE server on FD %d", filedescriptor);
}
void RTDEServer::disconnectionCallback(const socket_t filedescriptor)
{
  URCL_LOG_INFO("Client disconnected from RTDE server on FD %d", filedescriptor);
  stopSendingDataPackages();
}
void RTDEServer::messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv)
{
  comm::BinParser bp(reinterpret_cast<uint8_t*>(buffer), nbytesrecv);
  rtde_interface::PackageHeader::_package_size_type size;
  rtde_interface::PackageType type;
  bp.parse(size);
  bp.parse(type);

  switch (type)
  {
    case rtde_interface::PackageType::RTDE_REQUEST_PROTOCOL_VERSION:
    {
      bool accepted = true;
      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_REQUEST_PROTOCOL_VERSION, sizeof(uint8_t));
      size += serializer.serialize(buffer + size, accepted);

      size_t written;
      server_.write(filedescriptor, buffer, size, written);
      break;
    }
    case rtde_interface::PackageType::RTDE_GET_URCONTROL_VERSION:
    {
      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_GET_URCONTROL_VERSION, 4 * sizeof(uint32_t));
      uint32_t version = 10;
      size += serializer.serialize(buffer + size, version);  // major
      size += serializer.serialize(buffer + size, version);  // minor
      size += serializer.serialize(buffer + size, version);  // bugfix
      size += serializer.serialize(buffer + size, version);  // build

      size_t written;
      server_.write(filedescriptor, buffer, size, written);
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      bp.parse(output_frequency_);
      URCL_LOG_DEBUG("Frequency is set to %f", output_frequency_);
      std::string variable_names_str;
      bp.parseRemainder(variable_names_str);
      output_recipe_ = splitString(variable_names_str);

      output_data_package_ = std::make_unique<rtde_interface::DataPackage>(output_recipe_);
      output_data_package_->initEmpty();

      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS,
          variable_names_str.length() + sizeof(uint8_t));
      uint8_t recipe_id = 1;
      size += serializer.serialize(buffer + size, recipe_id);
      size += serializer.serialize(buffer + size,
                                   variable_names_str);  // We return the variable
                                                         // names list directly. For the initialization process, it is
                                                         // only important, that no field is "NOT_FOUND".

      size_t written;
      server_.write(filedescriptor, buffer, size, written);
      URCL_LOG_INFO("Output recipe set");
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
    {
      std::string variable_names_str;
      bp.parseRemainder(variable_names_str);
      input_recipe_ = splitString(variable_names_str);

      input_data_package_ = std::make_unique<rtde_interface::DataPackage>(input_recipe_);

      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS,
          variable_names_str.length() + sizeof(uint8_t));
      uint8_t recipe_id = 1;
      size += serializer.serialize(buffer + size, recipe_id);
      size += serializer.serialize(buffer + size,
                                   variable_names_str);  // We return the variable
                                                         // names list directly. For the initialization process, it is
                                                         // only important, that no field is "NOT_FOUND".

      size_t written;
      server_.write(filedescriptor, buffer, size, written);

      URCL_LOG_INFO("Input recipe set with %zu variables.", input_recipe_.size());
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_START:
    {
      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_START, sizeof(uint8_t));
      bool accepted = true;
      size += serializer.serialize(buffer + size, accepted);

      size_t written;
      server_.write(filedescriptor, buffer, size, written);
      startSendingDataPackages();
      break;
    }
    case rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_PAUSE:
    {
      comm::PackageSerializer serializer;
      uint8_t buffer[4096];
      size_t size = 0;
      size += rtde_interface::PackageHeader::serializeHeader(
          buffer, rtde_interface::PackageType::RTDE_CONTROL_PACKAGE_PAUSE, sizeof(uint8_t));
      bool accepted = true;
      size += serializer.serialize(buffer + size, accepted);

      size_t written;
      server_.write(filedescriptor, buffer, size, written);
      stopSendingDataPackages();
      break;
    }
    case rtde_interface::PackageType::RTDE_DATA_PACKAGE:
    {
      if (input_data_package_ == nullptr)
      {
        throw std::runtime_error("Fake RTDE Server received a data package before input recipe was setup. This should "
                                 "not happen.");
      }
      input_data_package_->parseWith(bp);
      actOnInput();
      break;
    }
    case rtde_interface::PackageType::RTDE_TEXT_MESSAGE:
    {
      URCL_LOG_WARN("Received Text message which usually shouldn't be sent to the RTDE server.");
      break;
    }
    default:
    {
      URCL_LOG_WARN("Received unknown package type %d", static_cast<int>(type));
      break;
    }
  }
}

void RTDEServer::startSendingDataPackages()
{
  URCL_LOG_INFO("Start sending data.");
  send_loop_running_ = true;
  send_thread_ = std::thread(&RTDEServer::sendDataLoop, this);
}

void RTDEServer::stopSendingDataPackages()
{
  URCL_LOG_INFO("Stop sending data.");
  send_loop_running_ = false;
  if (send_thread_.joinable())
  {
    send_thread_.join();
  }
}

void RTDEServer::sendDataLoop()
{
  while (send_loop_running_)
  {
    if (output_data_package_ != nullptr)
    {
      std::lock_guard<std::mutex> data_lock(output_data_mutex_);
      double timestamp = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
      output_data_package_->setData("timestamp", timestamp);
      uint8_t buffer[65536];
      size_t size = output_data_package_->serializePackage(buffer);
      size_t written;
      server_.write(client_socket_, buffer, size, written);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / output_frequency_));
  }
}

void RTDEServer::actOnInput()
{
  // This is not complete!
  if (std::find(input_recipe_.begin(), input_recipe_.end(), "speed_slider_mask") != input_recipe_.end())
  {
    double speed_slider_fraction = 0.0;
    input_data_package_->getData("speed_slider_fraction", speed_slider_fraction);
    std::lock_guard<std::mutex> data_lock(output_data_mutex_);
    output_data_package_->setData("target_speed_fraction", speed_slider_fraction);
  }
}

void RTDEServer::setStartTime(const std::chrono::steady_clock::time_point& start_time)
{
  start_time_ = start_time;
}

}  // namespace urcl
