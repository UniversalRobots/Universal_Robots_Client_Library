#pragma once

#include <memory>
#include <functional>
#include <cassert>

namespace urcl::comm {

  using Bytes = std::vector<int8_t>;

  class SocketListener;
  class SocketServerImpl;
  class SocketConnection;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Message
  {
  public:
    Message(const Bytes& rBytes)
      : m_Bytes(rBytes)
    {
    }
    virtual ~Message() = default;

    Message(const Message&) = delete;
    Message(Message&&) = delete;
    Message& operator=(Message const&) = delete;
    Message& operator=(Message&&) = delete;

  public:
    const Bytes& GetBytes() const
    {
      return m_Bytes;
    }

  private:
    Bytes m_Bytes;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Transport
  {
  public:
    virtual ~Transport() = default;
    virtual bool send_message(Message&& message) = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class Session
  {
    Session()
    {

    }

    Session(const int32_t& id, std::unique_ptr<Transport> transport)
      : m_transport(std::move(transport))
    {
    }


  public:
    const std::unique_ptr<Transport>& get_transport() const
    {
      return m_transport;
    }

  private:
    std::unique_ptr<Transport> m_transport;
  };

  class Router
  {
  public:
    void ProcessMessage()
    {
      assert(false);
    }

  private:
    std::unordered_map<int32_t, std::shared_ptr<Session>> m_sessions;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class MessageProcessor
  {
  public:
    MessageProcessor() = default;
    virtual ~MessageProcessor() = default;

  public:
    virtual void process_message(const std::unique_ptr<Message>& message, std::unique_ptr<Transport>&& transport,
                         SocketConnection* connection_base) = 0;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SocketServer
  {
  public:
    SocketServer(std::shared_ptr<MessageProcessor> pMessageProcessor);
    ~SocketServer();

  public:
    void attach_listener(const std::shared_ptr<SocketListener>& listener);
    void start();
    void shutdown();
  
    void SetConnectCallback(const std::function<void(const std::shared_ptr<SocketConnection>)>& rOnConnection);
    void SetDisconnectCallback(const std::function<void(const std::shared_ptr<SocketConnection>)>& rOnConnection);

  private:
    std::shared_ptr<SocketServerImpl> m_impl;
  };

} // namespace holodeck {
