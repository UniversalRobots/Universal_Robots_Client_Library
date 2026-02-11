#pragma once

#include <string>
#include <vector>

#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/rtde/rtde_package.h"
#include "ur_client_library/rtde/rtde_parser.h"

namespace urcl
{

class RTDEServer
{
public:
  RTDEServer() = delete;
  explicit RTDEServer(const int port);

  ~RTDEServer();

  void startSendingDataPackages();
  void stopSendingDataPackages();

  void setStartTime(const std::chrono::steady_clock::time_point& start_time);

private:
  std::vector<std::string> input_recipe_;
  std::vector<std::string> output_recipe_;

  std::unique_ptr<rtde_interface::DataPackage> output_data_package_;
  std::unique_ptr<rtde_interface::DataPackage> input_data_package_;
  comm::TCPServer server_;

  std::thread send_thread_;

  virtual void connectionCallback(const socket_t filedescriptor);

  virtual void disconnectionCallback(const socket_t filedescriptor);

  virtual void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv);

  void sendDataLoop();
  std::atomic<bool> send_loop_running_;

  double output_frequency_;

  std::chrono::steady_clock::time_point start_time_;

  socket_t client_socket_;

  void actOnInput();

  std::mutex output_data_mutex_;
};

}  // namespace urcl
