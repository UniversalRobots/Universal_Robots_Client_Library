#include <ur_client_library/ur/error_code_reader.h>

namespace urcl
{
bool ErrorCodeReader::consume(std::shared_ptr<primary_interface::PrimaryPackage> product)
{
  auto error_code_message = std::dynamic_pointer_cast<primary_interface::ErrorCodeMessage>(product);
  if (error_code_message != nullptr)
  {
    urcl::primary_interface::ErrorCode code;
    code.message_code = error_code_message->message_code_;
    code.message_argument = error_code_message->message_argument_;
    code.report_level = error_code_message->report_level_;
    code.data_type = error_code_message->data_type_;
    code.data = error_code_message->data_;
    code.text = error_code_message->text_;
    code.timestamp = error_code_message->timestamp_;
    code.to_string = error_code_message->toString();

    const auto logContents = "Logging an ErrorCodeMessage from the UR Controller Box: " + error_code_message->toString();

    switch(code.report_level) {
      case urcl::primary_interface::ReportLevel::DEBUG:
      case urcl::primary_interface::ReportLevel::DEVL_DEBUG:
        URCL_LOG_DEBUG(logContents.c_str());
        break;
      case urcl::primary_interface::ReportLevel::INFO:
      case urcl::primary_interface::ReportLevel::DEVL_INFO:
        URCL_LOG_INFO(logContents.c_str());
        break;
      case urcl::primary_interface::ReportLevel::WARNING:
      case urcl::primary_interface::ReportLevel::DEVL_WARNING:
        URCL_LOG_WARN(logContents.c_str());
        break;
      default:
        //urcl::primary_interface::ReportLevel::VIOLATION:
        //urcl::primary_interface::ReportLevel::DEVL_VIOLATION:
        //urcl::primary_interface::ReportLevel::FAULT:
        //urcl::primary_interface::ReportLevel::DEVL_FAULT:
        URCL_LOG_ERROR(logContents.c_str());
        break;
    }

    std::lock_guard<std::mutex> lock_guard(queue_mutex_);
    queue_.push_back(code);
  }
  return true;
}

std::deque<primary_interface::ErrorCode> ErrorCodeReader::getErrorCodesFromQueue() 
{
  std::lock_guard<std::mutex> lock_guard(queue_mutex_);
  std::deque<primary_interface::ErrorCode> error_codes;
  error_codes = queue_;
  queue_.clear();
  return error_codes;
}

}  // namespace urcl
