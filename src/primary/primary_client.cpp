#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/primary/robot_message.h>
#include <ur_client_library/primary/robot_state.h>

namespace urcl
{
namespace primary_interface
{
PrimaryClient::PrimaryClient(comm::URStream<PrimaryPackage>& stream)
  : stream_(stream)
{
  prod_.reset(new comm::URProducer<PrimaryPackage>(stream_, parser_));

  consumer_.reset(new PrimaryConsumer());
  consumer_->setErrorCodeMessageCallback(std::bind(&PrimaryClient::errorMessageCallback, this, std::placeholders::_1));
  
  // Configure multi consumer even though we only have one consumer as default, as this enables the user to add more
  // consumers after the object has been created
  std::vector<std::shared_ptr<comm::IConsumer<PrimaryPackage>>> consumers;
  consumers.push_back(consumer_);
  multi_consumer_.reset(new comm::MultiConsumer<PrimaryPackage>(consumers));
  
  pipeline_.reset(new comm::Pipeline<PrimaryPackage>(*prod_, multi_consumer_.get(), "PrimaryClient Pipeline", notifier_));
}

PrimaryClient::~PrimaryClient()
{
  URCL_LOG_INFO("Stopping primary client pipeline");
  pipeline_->stop();
}

void PrimaryClient::start()
{
  URCL_LOG_INFO("Starting primary client pipeline");
  pipeline_->run();
}

void PrimaryClient::addPrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer)
{
  multi_consumer_->addConsumer(primary_consumer);
}

void PrimaryClient::removePrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer)
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
