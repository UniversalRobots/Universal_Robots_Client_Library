/*
 * Copyright 2019, FZI Forschungszentrum Informatik (templating)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
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
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/parser.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/comm/package.h"
#include "ur_client_library/exceptions.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief A general producer for URPackages. Implements funcionality to produce packages by
 * reading and parsing from a byte stream.
 *
 * @tparam HeaderT Header type of packages to produce.
 */
template <typename T>
class URProducer : public IProducer<T>
{
private:
  URStream<T>& stream_;
  Parser<T>& parser_;
  std::chrono::seconds timeout_;
  std::function<void()> on_reconnect_cb_;

  bool running_;

public:
  /*!
   * \brief Creates a URProducer object, registering a stream and a parser.
   *
   * \param stream The stream to read from
   * \param parser The parser to use to interpret received byte information
   */
  URProducer(URStream<T>& stream, Parser<T>& parser) : stream_(stream), parser_(parser), timeout_(1), running_(false)
  {
  }

  /*!
   * \brief Triggers the stream to connect to the robot.
   *
   * \param max_num_tries Maximum number of connection attempts before counting the connection as
   * failed. Unlimited number of attempts when set to 0.
   * \param reconnection_time time in between connection attempts to the server
   */
  void setupProducer(const size_t max_num_tries = 0,
                     const std::chrono::milliseconds reconnection_time = std::chrono::seconds(10)) override
  {
    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    stream_.setReceiveTimeout(tv);
    if (!stream_.connect(max_num_tries, reconnection_time))
    {
      throw UrException("Failed to connect to robot. Please check if the robot is booted and connected.");
    }
  }
  /*!
   * \brief Tears down the producer. Currently no special handling needed.
   */
  void teardownProducer() override
  {
    stopProducer();
  }
  /*!
   * \brief Stops the producer. Currently no functionality needed.
   */
  void stopProducer() override
  {
    running_ = false;
  }

  void startProducer() override
  {
    running_ = true;
  }

  /*!
   * \brief Attempts to read byte stream from the robot and parse it as a URPackage.
   *
   * \param products Unique pointer to hold the produced package
   *
   * \returns Success of reading and parsing the package
   */
  bool tryGet(std::vector<std::unique_ptr<T>>& products) override
  {
    // TODO This function has become really ugly! That should be refactored!

    // 4KB should be enough to hold any packet received from UR
    uint8_t buf[4096];
    size_t read = 0;
    // expoential backoff reconnects
    while (true)
    {
      if (stream_.read(buf, sizeof(buf), read))
      {
        // reset sleep amount
        timeout_ = std::chrono::seconds(1);
        BinParser bp(buf, read);
        return parser_.parse(bp, products);
      }

      if (!running_)
        return true;

      if (stream_.getState() == SocketState::Connected)
      {
        continue;
      }

      if (stream_.closed())
        return false;

      if (on_reconnect_cb_)
      {
        URCL_LOG_WARN("Failed to read from stream, invoking on reconnect callback and stopping the producer");
        on_reconnect_cb_();
        return false;
      }

      URCL_LOG_WARN("Failed to read from stream, reconnecting in %ld seconds...", timeout_.count());
      std::this_thread::sleep_for(timeout_);

      if (stream_.connect())
        continue;

      auto next = timeout_ * 2;
      if (next <= std::chrono::seconds(120))
        timeout_ = next;
    }

    return false;
  }

  /*!
   * \brief Sets the reconnection callback. Use this to configure a reconnection callback instead of connecting directly
   * to the stream again. This is needed for RTDE as it requires setting up the communication again upon reconnection it
   * is not enough to just reconnect to the stream.
   *
   * \param on_reconnect_cb Callback to be invoked when connection is lost to the stream.
   */
  void setReconnectionCallback(std::function<void()> on_reconnect_cb)
  {
    on_reconnect_cb_ = on_reconnect_cb;
  }
};
}  // namespace comm
}  // namespace urcl
