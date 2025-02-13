// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright © 2024-2025 Ocado Group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED

#include <memory>
#include <deque>

#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/primary/abstract_primary_consumer.h>
#include <ur_client_library/primary/primary_consumer.h>
#include <ur_client_library/primary/primary_package.h>
#include <ur_client_library/primary/primary_parser.h>

namespace urcl
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(const std::string& robot_ip, comm::INotifier& notifier);
  ~PrimaryClient();

  /*!
   * \brief Adds a primary consumer to the list of consumers
   *
   * \param primary_consumer Primary consumer that should be added to the list
   */
  void addPrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer);

  /*!
   * \brief Remove a primary consumer from the list of consumers
   *
   * \param primary_consumer Primary consumer that should be removed from the list
   */
  void removePrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer);
  void start(const size_t max_connection_attempts = 0,
             const std::chrono::milliseconds reconnection_timeout = urcl::comm::TCPSocket::DEFAULT_RECONNECTION_TIME);

  /*!
   * \brief Retrieves previously raised error codes from PrimaryClient. After calling this, recorded errors will be
   * deleted.
   */
  std::deque<ErrorCode> getErrorCodes();

private:
  // The function is called whenever an error code message is received
  void errorMessageCallback(ErrorCode& code);

  PrimaryParser parser_;
  std::shared_ptr<PrimaryConsumer> consumer_;
  std::unique_ptr<comm::MultiConsumer<PrimaryPackage>> multi_consumer_;

  comm::INotifier notifier_;

  comm::URStream<PrimaryPackage> stream_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> prod_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;

  std::mutex error_code_queue_mutex_;
  std::deque<ErrorCode> error_code_queue_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CLIENT_H_INCLUDED
