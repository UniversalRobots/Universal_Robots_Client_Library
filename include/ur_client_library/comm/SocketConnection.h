#pragma once

#include <iostream>
#include <mutex>

#include <asio/buffer.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/read.hpp>
#include <asio/write.hpp>

#include <asio/local/stream_protocol.hpp>

namespace urcl::comm {

  class DataBuffer;
  using ParseFunctionCallback = std::function<void(DataBuffer&)>;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SocketConnection : public std::enable_shared_from_this<SocketConnection>
  {
    friend class TcpConnection;

  public:
    using close_handler = std::function<void(const std::shared_ptr<SocketConnection>&)>;
    using fail_handler = std::function<void(const std::shared_ptr<SocketConnection>&, const char*)>;
    using message_handler = std::function<void(const std::shared_ptr<SocketConnection>&&, const char*, size_t)>;
    using read_handler = std::function<void(const asio::error_code&, size_t)>;

  public:
    SocketConnection(size_t message_bufer_size);
    virtual ~SocketConnection() = default;

  public:
    virtual void async_read(void* data, size_t length, const read_handler& handler, bool readSome) = 0;
    virtual void write(const void* data, size_t length, asio::error_code& error_code) = 0;

    void async_receive(bool read_some);

    bool send_message(const char* message, size_t length);

    const close_handler& get_close_handler() const;
    const fail_handler& get_fail_handler() const;
    const message_handler& get_message_handler() const;

    void set_close_handler(const close_handler& handler);
    void set_fail_handler(const fail_handler& handler);
    void set_message_handler(const message_handler& handler);


    /// new stuff
    //void SetParseFunction(const ParseFunctionCallback& rParseFunction);

    //virtual void AsyncReadSome() = 0;

    //void RegisterParser(const ParseFunctionCallback& parseFunction)
    //{
    //  std::lock_guard<std::recursive_mutex> lock(m_ParseFunctionMutex);

    //  m_ParseDataFunction = parseFunction;
    //}

    //void UnregisterParser()
    //{
    //  std::lock_guard<std::recursive_mutex> lock(m_ParseFunctionMutex);

    //  m_ParseDataFunction = nullptr;
    //}

  private:
    void receive_message_handler(const asio::error_code& error_code, size_t bytes_transferred);

    void handle_system_error(const asio::error_code& error_code);

  protected:
    // new stuff
    //void ReadLoopHandler(const asio::error_code& rError, std::size_t bytesTransferred);
    //void ClearBuffer();

  protected:
    /// new stuff
    //DataBuffer m_ReadBuffer;
    //BufferWriter m_BufferWriter;

    //ParseFunctionCallback m_ParseDataFunction;

    //std::recursive_mutex m_ParseFunctionMutex;

  private:
    close_handler m_close_handler;
    fail_handler m_fail_handler;
    message_handler m_message_handler;

    uint32_t m_message_length;
    std::vector<char> m_message_buffer;

    //Logger m_Logger{ Log::GetLogger("SocketConnection") };
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class TcpConnection : public SocketConnection
  {
  public:
    TcpConnection(asio::ip::tcp::socket&& socket);
    ~TcpConnection() override;

  public:
    void async_read(void* data, size_t length, const read_handler& handler, bool readSome) override;
    void write(const void* data, size_t length, asio::error_code& error_code) override;

    //void AsyncReadSome() override;

  private:
    asio::ip::tcp::socket m_socket;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class UdsConnection : public SocketConnection
  {
  public:
    UdsConnection(asio::local::stream_protocol::socket&& socket);
    virtual ~UdsConnection() override;

  public:
    virtual void async_read(void* data, size_t length, const read_handler& handler, bool readSome) override;
    virtual void write(const void* data, size_t length, asio::error_code& error_code) override;

    //void AsyncReadSome() override
    //{
    //  assert(false);
    //}

  private:
    asio::local::stream_protocol::socket m_socket;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

} // namespace holodeck {
