/*
 * Copyright 2022, FZI Forschungszentrum Informatik (templating)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <chrono>
#include <atomic>
#include <thread>
#include "ur_client_library/comm/stream.h"

namespace urcl
{
namespace comm
{
/*! \brief A special URStream that can perform reconnects as needed
 *
 */
template <typename T>
class URPersistentStream : public URStream<T>
{
public:
  /*!
   * \brief Creates a new persistent stream.
   *
   * As with a normal URStream object, this does not immediately open the socket. This has to be done manually using the
   * connect() function.
   *
   * \param host IP address of the remote host
   * \param port Port on which the socket shall be connected
   */
  URPersistentStream(const std::string& host, int port)
    : URStream<T>(host, port), timeout_(std::chrono::seconds(1)), stop_(false)
  {
  }

  /*!
   * \brief Stop all currently running reconnect attempts
   */
  ~URPersistentStream();

  URPersistentStream(const URPersistentStream&) = delete;
  URPersistentStream& operator=(const URPersistentStream&) = delete;

  /*! \brief Reads a full UR package from the socket, trying to reconnect if required.
   *
   * This is done by repeatedly reading bytes from the socket until the number of bytes occupied by a single package
   * are received. In case of a socket disconnect during package transimission, the already received segment is
   * discarded.
   *
   * \param[out] buf The byte buffer where the content shall be stored
   * \param[in] buf_len Maximum number of bytes that can be stored in buf
   * \param[out] read Number of bytes actually read from the socket
   *
   * \returns Success of the operation
   */
  bool read(uint8_t* buf, const size_t buf_len, size_t& read);

  /*! \brief Write directly to the underlying socket, trying to reconnect if required
   *
   * If a socket disconnect occurs during transmission, the already sent data is discarded and the complete write
   * operation is repeated after the reconnect.
   *
   * \param[in] buf Byte stream that should be sent
   * \param[in] buf_len Number of bytes in buf to be sent
   * \param[out] written Number of bytes actually written to the socket
   *
   * \returns Success of the operation
   */
  bool write(const uint8_t* buf, const size_t buf_len, size_t& written);

private:
  bool tryReconnect();

  std::atomic<std::chrono::seconds> timeout_;
  std::atomic<bool> stop_;
};

template <typename T>
URPersistentStream<T>::~URPersistentStream()
{
  stop_.store(true);
}

template <typename T>
bool URPersistentStream<T>::read(uint8_t* buf, const size_t buf_len, size_t& read)
{
  while (!stop_.load())
  {
    if (URStream<T>::read(buf, buf_len, read))
    {
      timeout_.store(std::chrono::seconds(1));
      return true;
    }

    if (!tryReconnect())
    {
      return false;
    }
  }

  return false;
}

template <typename T>
bool URPersistentStream<T>::write(const uint8_t* buf, const size_t buf_len, size_t& written)
{
  while (!stop_.load())
  {
    if (URStream<T>::write(buf, buf_len, written))
    {
      timeout_.store(std::chrono::seconds(1));
      return true;
    }

    if (!tryReconnect())
    {
      return false;
    }
  }

  return false;
}

template <typename T>
bool URPersistentStream<T>::tryReconnect()
{
  if (URStream<T>::closed())
  {
    return false;
  }

  auto cur_timeout = timeout_.load();
  URCL_LOG_WARN("Reconnecting in %ld seconds...", cur_timeout.count());
  std::this_thread::sleep_for(cur_timeout);

  if (URStream<T>::connect())
  {
    timeout_.store(std::chrono::seconds(1));
  }
  else if (cur_timeout <= std::chrono::seconds(60))
  {
    timeout_.compare_exchange_strong(cur_timeout, 2 * cur_timeout);
  }

  return true;
}
}  // namespace comm
}  // namespace urcl
