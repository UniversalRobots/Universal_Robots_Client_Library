#pragma once

#include <deque>
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

  /*!
   * \brief Makes the server send \p message ahead of its answer to the URControl version query.
   *
   * Real controllers do this: a PolyScope X simulator reports "SafetySetup has not been confirmed
   * yet" on every connect until it has been switched on. Queue more messages than the client is
   * willing to retry and it will give up on the query.
   */
  void queueTextMessageBeforeVersionReply(const std::string& message);

  /*!
   * \brief Makes the server refuse every protocol version above \p highest_accepted.
   *
   * Lets a test drive the client's fallback to an older protocol version, the way a controller too
   * old for the newest version would.
   */
  void setHighestAcceptedProtocolVersion(const uint16_t highest_accepted);

  /*!
   * \brief The protocol versions the client asked for, in the order it asked, so a test can see it
   * work its way down.
   */
  std::vector<uint16_t> requestedProtocolVersions();

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

  void sendTextMessage(const socket_t filedescriptor, const std::string& message);

  std::mutex output_data_mutex_;
  std::mutex thread_control_mutex_;

  std::mutex negotiation_mutex_;
  std::deque<std::string> pending_text_messages_;
  uint16_t highest_accepted_protocol_version_ = 2;
  std::vector<uint16_t> requested_protocol_versions_;
};

}  // namespace urcl
