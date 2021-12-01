#include <set>

#include "ur_client_library/comm/SocketServer.h"
#include "ur_client_library/comm/SocketConnection.h"
#include "ur_client_library/comm/SocketListener.h"

namespace urcl::comm {

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SocketTransport : public Transport
  {
  public:
    SocketTransport(const std::shared_ptr<SocketConnection>& connection);

  public:
    bool send_message(Message&& message) override;

  private:
    std::shared_ptr<SocketConnection> m_connection;
  };

  SocketTransport::SocketTransport(const std::shared_ptr<SocketConnection>& connection)
    : m_connection(connection)
  {
  }

  bool SocketTransport::send_message(Message&& message)
  {
    //BONEFISH_TRACE("sending message: %1%", message_type_to_string(message.get_type()));
    //expandable_buffer buffer = m_serializer->serialize(message);
    assert(false);

    auto bytes = message.GetBytes();
    return m_connection->send_message((const char *)bytes.data(), bytes.size());
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SocketServerImpl : public std::enable_shared_from_this<SocketServerImpl>
  {
  public:
    SocketServerImpl(std::shared_ptr<MessageProcessor> pMessageProcessor);

  public:
    void attach_listener(const std::shared_ptr< SocketListener >& listener);

    void start();
    void shutdown();

    void SetConnectCallback(const std::function<void(const std::shared_ptr<SocketConnection>)>& rOnConnection);
    void SetDisconnectCallback(const std::function<void(const std::shared_ptr<SocketConnection>)>& rOnConnection);

  private:
    void on_connect(const std::shared_ptr<SocketConnection>& connection);
    void on_message(const std::shared_ptr<SocketConnection>& connection, const char* buffer, size_t length);
    void on_close(const std::shared_ptr<SocketConnection>& connection);
    void on_fail(const std::shared_ptr<SocketConnection>& connection, const char* reason);

    void teardown_connection(const std::shared_ptr<SocketConnection>& connection);

  private:
    std::shared_ptr<MessageProcessor> m_message_processor;

    std::set<std::shared_ptr<SocketListener>, std::owner_less<std::shared_ptr<SocketListener>>> m_listeners;

    std::set<std::shared_ptr<SocketConnection>, std::owner_less<std::shared_ptr<SocketConnection>>> m_connections;

    std::function<void(const std::shared_ptr<SocketConnection>)> m_ConnectCallback;
    std::function<void(const std::shared_ptr<SocketConnection>)> m_DisconnectCallback;

    //Logger m_Logger{ Log::GetLogger("SocketServer") };
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SocketServerImpl::SocketServerImpl(std::shared_ptr<MessageProcessor> pMessageProcessor)
    : m_message_processor(pMessageProcessor)
  {
  }

  void SocketServerImpl::attach_listener(const std::shared_ptr<SocketListener>& listener)
  {
    std::weak_ptr<SocketServerImpl> weak_self = shared_from_this();

    listener->set_error_handler([weak_self](SocketListener& listener, const asio::error_code& error_code)
      {
        auto shared_self = weak_self.lock();
        if (shared_self) 
        {
          // We better have an error code to process otherwise the
          // caller is almost certainly calling us by mistake..
          assert(error_code);

          // If the error code is reporting an aborted operation then
          // it is an indication that the listener is shutting down so
          // there is no action to take here.
          if (error_code == asio::error::operation_aborted) 
          {
            return;
          }

          // Otherwise, the listener has encountered some kind of network
          // error when trying to accept connections. As a result, we should
          // try to re-establish a listening socket otherwise this listener
          // could be rendered unusable.
          assert(listener.is_listening());
          assert(false);
          // shared_self->m_Logger.Error("Listener error: {}", error_code.message());
          listener.stop_listening();
          listener.start_listening();
        }
      }
    );

    listener->set_accept_handler([weak_self](const std::shared_ptr<SocketConnection>& connection) 
      {
        auto shared_self = weak_self.lock();
        if (shared_self) 
        {
          shared_self->on_connect(connection);
        }
      }
    );

    m_listeners.insert(listener);
  }

  void SocketServerImpl::start()
  {
    //m_Logger.Debug("Starting");

    assert(!m_listeners.empty());
    for (auto& listener : m_listeners) 
    {
      listener->start_listening();
    }
  }

  void SocketServerImpl::shutdown()
  {
    //m_Logger.Debug("Stopping");

    for (auto& listener : m_listeners) 
    {
      listener->stop_listening();
    }
  }

  void SocketServerImpl::on_connect(const std::shared_ptr<SocketConnection>& connection)
  {
    std::weak_ptr<SocketServerImpl> weak_self = shared_from_this();

    auto message_handler = [weak_self](const std::shared_ptr<SocketConnection>& connection, const char* buffer, size_t length)
    {
      auto shared_self = weak_self.lock();
      if (shared_self) 
      {
        shared_self->on_message(connection, buffer, length);
      }
    };
    connection->set_message_handler(message_handler);

    auto close_handler = [weak_self](const std::shared_ptr<SocketConnection>& connection)
    {
      auto shared_self = weak_self.lock();
      if (shared_self) 
      {
        shared_self->on_close(connection);
      }
    };
    connection->set_close_handler(close_handler);

    auto fail_handler = [weak_self](const std::shared_ptr<SocketConnection>& connection, const char* reason)
    {
      auto shared_self = weak_self.lock();
      if (shared_self)
      {
        shared_self->on_fail(connection, reason);
      }
    };
    connection->set_fail_handler(fail_handler);

    if (m_ConnectCallback)
    {
      m_ConnectCallback(connection);
    }

    connection->async_receive(true);

    // XXXMAE BUGBUG
    //  connection->AsyncReadSome();

    m_connections.insert(connection);
  }

  void SocketServerImpl::on_close(const std::shared_ptr<SocketConnection>& connection)
  {
    teardown_connection(connection);
  }

  void SocketServerImpl::on_fail(const std::shared_ptr<SocketConnection>& connection, const char* reason)
  {
    teardown_connection(connection);
  }

  void SocketServerImpl::on_message(const std::shared_ptr<SocketConnection>& connection, const char* buffer, size_t length)
  {
    try 
    {
      Bytes bytes;
      bytes.resize(length);
      memcpy(&bytes[0], buffer, length);
      auto message = std::make_unique<Message>(bytes);
      auto transport = std::make_unique<SocketTransport>(connection);

      if (message) 
      {
        m_message_processor->process_message(message, std::move(transport), connection.get());
      }
    }
    catch (const std::exception& e) 
    {
      //m_Logger.Error("unhandled exception: {}", e.what());
    }
  }

  void SocketServerImpl::teardown_connection(const std::shared_ptr<SocketConnection>& connection)
  {
    //if (connection->has_session_id()) 
    //{
    //  std::shared_ptr<wamp_router> router = m_routers->get_router(connection->get_realm());
    //  if (router) 
    //  {
    //    router->detach_session(connection->get_session_id());
    //  }
    //}

    if (m_DisconnectCallback)
    {
      m_DisconnectCallback(connection);
    }

    m_connections.erase(connection);
  }

  void SocketServerImpl::SetConnectCallback(
      const std::function<void(const std::shared_ptr<SocketConnection>)>& rConnectCallback)
  {
    m_ConnectCallback = rConnectCallback;
  }

  void SocketServerImpl::SetDisconnectCallback(
      const std::function<void(const std::shared_ptr<SocketConnection>)>& rDisconnectCallback)
  {
    m_DisconnectCallback = rDisconnectCallback;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SocketServer::SocketServer(std::shared_ptr<MessageProcessor> pMessageProcessor)
    : m_impl(std::make_shared<SocketServerImpl>(pMessageProcessor))
  {
  }

  SocketServer::~SocketServer()
  {
  }

  void SocketServer::attach_listener(const std::shared_ptr<SocketListener>& listener)
  {
    m_impl->attach_listener(listener);
  }

  void SocketServer::start()
  {
    m_impl->start();
  }

  void SocketServer::shutdown()
  {
    m_impl->shutdown();
  }

  void
  SocketServer::SetConnectCallback(const std::function<void(const std::shared_ptr<SocketConnection>)>& rConnectCallback)
  {
    m_impl->SetConnectCallback(rConnectCallback);
  }

  void SocketServer::SetDisconnectCallback(
      const std::function<void(const std::shared_ptr<SocketConnection>)>& rDisconnectCallback)
  {
    m_impl->SetDisconnectCallback(rDisconnectCallback);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

} // namespace holodeck {
