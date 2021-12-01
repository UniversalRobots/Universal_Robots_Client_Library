#pragma once

#include <cassert>

#include <asio/io_service.hpp>
#include <asio/ip/tcp.hpp>

#include <asio/local/stream_protocol.hpp>

namespace urcl::comm {

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SocketConnection;

  class SocketListener
  {
  public:
    using error_handler = std::function<void(SocketListener& listener, const asio::error_code& error_code)>;
    using accept_handler = std::function<void(const std::shared_ptr<SocketConnection>&)>;

  public:
    SocketListener() = default;
    virtual ~SocketListener() = default;

  public:
    virtual void start_listening() = 0;
    virtual void stop_listening() = 0;

    virtual std::shared_ptr<SocketConnection> create_connection() = 0;

    bool is_listening() const;

    const error_handler& get_error_handler() const;
    void set_error_handler(const error_handler& handler);

    const accept_handler& get_accept_handler() const;
    void set_accept_handler(const accept_handler& handler);

    void handle_accept(const asio::error_code& error_code);

  protected:
    virtual void async_accept() = 0;

    void set_listening(bool is_listening);

  private:
    std::atomic_bool m_listening{ false };
    error_handler m_error_handler{};
    accept_handler m_accept_handler{};
  };

  inline bool SocketListener::is_listening() const
  {
    return m_listening;
  }

  inline const typename SocketListener::error_handler& SocketListener::get_error_handler() const
  {
    return m_error_handler;
  }

  inline void SocketListener::set_error_handler(const SocketListener::error_handler& handler)
  {
    m_error_handler = handler;
  }

  inline const typename SocketListener::accept_handler& SocketListener::get_accept_handler() const
  {
    return m_accept_handler;
  }

  inline void SocketListener::set_accept_handler(const SocketListener::accept_handler& handler)
  {
    m_accept_handler = handler;
  }

  inline void SocketListener::handle_accept(const asio::error_code& error_code)
  {
    // If an error has occured, notify the registered error
    // handler so appropriate action can be taken.
    if (error_code) 
    {
      assert(get_error_handler());
      const auto& error_handler = get_error_handler();
      error_handler(*this, error_code);
      return;
    }

    // Otherwise, this is a new incoming connection so go ahead
    // and create a new connection and notify the registered
    // accept handler.
    assert(get_accept_handler());
    const auto& accept_handler = get_accept_handler();
    accept_handler(create_connection());
    async_accept();
  }

  inline void SocketListener::set_listening(bool is_listening)
  {
    m_listening = is_listening;
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class TcpListener : public SocketListener, public std::enable_shared_from_this<TcpListener>
  {
  public:
    TcpListener(asio::io_service& io_service, const asio::ip::address& ip_address, uint16_t port);
    ~TcpListener() override;

  public:
    void start_listening() override;
    void stop_listening() override;

    std::shared_ptr<SocketConnection> create_connection() override;

  protected:
    void async_accept() override;
  
  private:
    asio::ip::tcp::socket m_socket;
    asio::ip::tcp::acceptor m_acceptor;
    asio::ip::tcp::endpoint m_endpoint;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class UdsListener : public SocketListener, public std::enable_shared_from_this<UdsListener>
  {
  public:
    UdsListener(asio::io_service& io_service, const std::string& path, bool deleteFile);
    ~UdsListener() override;

  public:
    void start_listening() override;
    void stop_listening() override;

    std::shared_ptr<SocketConnection> create_connection() override;

  protected:
    void async_accept() override;

  private:
    asio::local::stream_protocol::socket m_socket;
    asio::local::stream_protocol::acceptor m_acceptor;
    asio::local::stream_protocol::endpoint m_endpoint;

    bool m_DeleteFile{ false };
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

} // namespace holodeck {
