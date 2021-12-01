#pragma once

#include <atomic>
#include <functional>
#include <thread>

#include "SocketServer.h"

#include <asio.hpp>

namespace urcl::comm
{
class ServerImpl;

class Server
{
public:
  Server() = delete;
  Server(const int port);
  ~Server();

public:
  void setConnectCallback(std::function<void(const int)> func);
  void setDisconnectCallback(std::function<void(const int)> func);
  void setMessageCallback(std::function<void(const int, char*, int)> func);
  void setMaxClientsAllowed(const uint32_t& max_clients_allowed);

  void start();

  bool write(const int fd, const uint8_t* buf, const size_t buf_len, size_t& written);

private:
  std::shared_ptr<ServerImpl> m_pImpl;
};

}  // namespace urcl::comm