#include "ur_client_library/comm/SocketListener.h" 
#include "ur_client_library/comm/SocketConnection.h" 

namespace urcl::comm {

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  TcpListener::TcpListener(asio::io_service& io_service, const asio::ip::address& ip_address, uint16_t port)
    : SocketListener()
    , m_socket(io_service)
    , m_acceptor(io_service)
    , m_endpoint(ip_address, port)
  {
  }

  TcpListener::~TcpListener()
  {
    stop_listening();
  }

  void TcpListener::start_listening()
  {
    assert(get_error_handler());
    assert(get_accept_handler());

    if (is_listening()) 
    {
      return;
    }

    m_acceptor.open(m_endpoint.protocol());
    m_acceptor.set_option(asio::ip::tcp::acceptor::reuse_address(true));
    m_acceptor.bind(m_endpoint);
    m_acceptor.listen();

    set_listening(true);
    async_accept();
  }

  void TcpListener::stop_listening()
  {
    if (!is_listening()) 
    {
      return;
    }

    m_acceptor.close();
    set_listening(false);
  }

  void TcpListener::async_accept()
  {
    m_acceptor.async_accept(m_socket, std::bind(&SocketListener::handle_accept, shared_from_this(), std::placeholders::_1));
  }

  std::shared_ptr<SocketConnection> TcpListener::create_connection()
  {
    return std::make_shared<TcpConnection>(std::move(m_socket));
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  UdsListener::UdsListener(asio::io_service& io_service, const std::string& path, bool deleteFile)
    : SocketListener()
    , m_socket(io_service)
    , m_acceptor(io_service)
    , m_endpoint(path)
    , m_DeleteFile(deleteFile)
  {
  }

  UdsListener::~UdsListener()
  {
    stop_listening();
  }

  void UdsListener::start_listening()
  {
    assert(get_error_handler());
    assert(get_accept_handler());

    if (is_listening()) 
    {
      return;
    }

    if (m_DeleteFile)
    {
      //unlink(m_endpoint.path().c_str());
      std::remove(m_endpoint.path().c_str());
    }

    m_acceptor.open(m_endpoint.protocol());
#ifdef WIN32
    m_acceptor.set_option(asio::local::stream_protocol::acceptor::reuse_address(false));
#else
    m_acceptor.set_option(asio::local::stream_protocol::acceptor::reuse_address(true));
#endif
    m_acceptor.bind(m_endpoint);
    m_acceptor.listen();

    set_listening(true);
    async_accept();
  }

  void UdsListener::stop_listening()
  {
    if (!is_listening()) 
    {
      return;
    }

    m_acceptor.close();
    set_listening(false);
  }

  void UdsListener::async_accept()
  {
    m_acceptor.async_accept(m_socket, std::bind(&SocketListener::handle_accept, shared_from_this(), std::placeholders::_1));
  }

  std::shared_ptr<SocketConnection> UdsListener::create_connection()
  {
    return std::make_shared<UdsConnection>(std::move(m_socket));
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

} // namespace holodeck {
