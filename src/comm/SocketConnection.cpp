#include "ur_client_library/comm/SocketConnection.h" 

namespace urcl::comm {

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  SocketConnection::SocketConnection(size_t message_bufer_size)
    : m_close_handler()
    , m_fail_handler()
    , m_message_handler()

    , m_message_length(message_bufer_size)
{
    m_message_buffer.reserve(m_message_length);
  }

  void SocketConnection::async_receive(bool read_some)
  {
    std::weak_ptr<SocketConnection> weak_self = std::static_pointer_cast<SocketConnection>(shared_from_this());

    auto handler = [weak_self](const asio::error_code& error_code, size_t bytes_transferred)
    {
      auto shared_self = weak_self.lock();
      if (shared_self)
      {
        shared_self->receive_message_handler(error_code, bytes_transferred);
      }
    };

    async_read(m_message_buffer.data(), m_message_length, handler, read_some);
  }

  bool SocketConnection::send_message(const char* message, size_t length)
  {
    asio::error_code error_code;

    // First write the message length prefix.
    uint32_t length_prefix = htonl(length);
    write(&length_prefix, sizeof(length_prefix), error_code);
    if (error_code)
    {
      handle_system_error(error_code);
      return false;
    }

    // Then write the actual message.
    write(message, length, error_code);
    if (error_code)
    {
      handle_system_error(error_code);
      return false;
    }

    return true;
  }

  const SocketConnection::close_handler& SocketConnection::get_close_handler() const
  {
    return m_close_handler;
  }

  const SocketConnection::fail_handler& SocketConnection::get_fail_handler() const
  {
    return m_fail_handler;
  }

  const SocketConnection::message_handler& SocketConnection::get_message_handler() const
  {
    return m_message_handler;
  }

  void SocketConnection::set_close_handler(const SocketConnection::close_handler& handler)
  {
    m_close_handler = handler;
  }

  void SocketConnection::set_fail_handler(const SocketConnection::fail_handler& handler)
  {
    m_fail_handler = handler;
  }

  void SocketConnection::set_message_handler(const SocketConnection::message_handler& handler)
  {
    m_message_handler = handler;
  }

  void SocketConnection::receive_message_handler(const asio::error_code& error_code, size_t bytes_transferred)
  {
    if (error_code)
    {
      handle_system_error(error_code);
      return;
    }

    const auto& message_handler = get_message_handler();
    assert(message_handler);
    message_handler(shared_from_this(), m_message_buffer.data(), bytes_transferred);

    async_receive(true);
  }

  void SocketConnection::handle_system_error(const asio::error_code& error_code)
  {
    // NOTE: The boost documentation does not indicate what all of the possible error
    //       codes are that can occur for the async receive handlers. So it will be an
    //       ongoing exercise in trying to figure this out.
    if (error_code == asio::error::eof)
    {
      std::cout << /* m_Logger.Warning( */"connection closed: " << error_code.message() << std::endl;
      const auto& close_handler = get_close_handler();
      close_handler(shared_from_this());
    }
    else if (error_code != asio::error::operation_aborted)
    {
      std::cout << /*m_Logger.Error(*/ "connection failed: " << error_code.message() << std::endl;
      const auto& fail_handler = get_fail_handler();
      fail_handler(shared_from_this(), error_code.message().c_str());
    }
    else
    {
      assert(false);
      //m_Logger.Error("unhandled system error: {}", error_code.message());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  TcpConnection::TcpConnection(asio::ip::tcp::socket&& socket)
    : SocketConnection(100)
    , m_socket(std::move(socket))
  {
    // Disable Nagle algorithm to get lower latency (and lower throughput).
    m_socket.set_option(asio::ip::tcp::no_delay(true));
  }

  TcpConnection::~TcpConnection()
  {
    if (m_socket.is_open())
    {
      m_socket.close();
    }
  }

  void TcpConnection::async_read(void* data, size_t length, const read_handler& handler, bool readSome)
  {
    if (readSome)
    {
      m_socket.async_read_some(asio::buffer(data, length), handler);
    }
    else
    {
      asio::async_read(m_socket, asio::buffer(data, length), handler);
    }
  }

  void TcpConnection::write(const void* data, size_t length, asio::error_code& error_code)
  {
    asio::write(m_socket, asio::buffer(data, length), error_code);
    //m_socket.async_write_some(asio::buffer(data, length), [this](asio::error_code _err, size_t)
    //  {
    //    if (!_err) 
    //    {
    //      std::cout << "!" << std::endl;
    //    }
    //    else 
    //    {
    //      std::cerr << "[ERROR]: " << _err.message() << std::endl;
    //      m_socket.close();
    //    }
    //  }
    //);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  UdsConnection::UdsConnection(asio::local::stream_protocol::socket&& socket)
    : SocketConnection(100)
    , m_socket(std::move(socket))
  {
    //std::cerr << "creating uds connection" << std::endl;
  }

  UdsConnection::~UdsConnection()
  {
    //std::cerr << "destroying uds connection" << std::endl;
    if (m_socket.is_open())
    {
      //std::cerr << "Closed the uds socket" << std::endl;
      m_socket.close();
    }
  }

  void UdsConnection::async_read(void* data, size_t length, const read_handler& handler, bool readSome)
  {
    asio::async_read(m_socket, asio::buffer(data, length), handler);
  }

  void UdsConnection::write(const void* data, size_t length, asio::error_code& error_code)
  {
    asio::write(m_socket, asio::buffer(data, length), error_code);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

} // namespace holodeck {
