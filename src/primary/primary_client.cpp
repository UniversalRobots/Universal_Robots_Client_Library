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

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/primary/robot_message.h>
#include <ur_client_library/primary/robot_state.h>

namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(const std::string& robot_ip, comm::INotifier& notifier)
  : stream_(robot_ip, UR_PRIMARY_PORT)
{
  prod_.reset(new comm::URProducer<PrimaryPackage>(stream_, parser_));

  consumer_.reset(new PrimaryConsumer());
  consumer_->setErrorCodeMessageCallback(std::bind(&PrimaryClient::errorMessageCallback, this, std::placeholders::_1));

  // Configure multi consumer even though we only have one consumer as default, as this enables the user to add more
  // consumers after the object has been created
  std::vector<std::shared_ptr<comm::IConsumer<PrimaryPackage>>> consumers;
  consumers.push_back(consumer_);
  multi_consumer_.reset(new comm::MultiConsumer<PrimaryPackage>(consumers));

  pipeline_.reset(
      new comm::Pipeline<PrimaryPackage>(*prod_, multi_consumer_.get(), "PrimaryClient Pipeline", notifier_));
}

PrimaryClient::~PrimaryClient()
{
  URCL_LOG_INFO("Stopping primary client pipeline");
  pipeline_->stop();
}

void PrimaryClient::start(const size_t max_num_tries, const std::chrono::milliseconds reconnection_time)
{
  URCL_LOG_INFO("Starting primary client pipeline");
  pipeline_->init(max_num_tries, reconnection_time);
  pipeline_->run();
}

void PrimaryClient::addPrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->addConsumer(primary_consumer);
}

void PrimaryClient::removePrimaryConsumer(std::shared_ptr<comm::IConsumer<PrimaryPackage>> primary_consumer)
{
  multi_consumer_->removeConsumer(primary_consumer);
}

void PrimaryClient::errorMessageCallback(ErrorCode& code)
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  error_code_queue_.push_back(code);
}

std::deque<ErrorCode> PrimaryClient::getErrorCodes()
{
  std::lock_guard<std::mutex> lock_guard(error_code_queue_mutex_);
  std::deque<ErrorCode> error_codes;
  error_codes = error_code_queue_;
  error_code_queue_.clear();
  return error_codes;
}
}  // namespace primary_interface
}  // namespace urcl
