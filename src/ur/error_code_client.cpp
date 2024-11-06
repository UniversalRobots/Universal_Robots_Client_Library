#include "ur_client_library/ur/error_code_client.h"
#include "ur_client_library/primary/robot_message.h"
#include "ur_client_library/primary/robot_state.h"

namespace urcl
{
ErrorCodeClient::ErrorCodeClient(comm::URStream<primary_interface::PrimaryPackage>& stream, comm::INotifier& notifier, 
                primary_interface::PrimaryParser& parser)
: stream_(stream)
, parser_(parser)
, prod_(stream_, parser_)
, pipeline_(prod_, &consumer_, "ErrorCodeClient Pipeline", notifier)
{
}

ErrorCodeClient::~ErrorCodeClient()
{
    URCL_LOG_INFO("Stopping error code client pipeline");
    pipeline_.stop();
}

void ErrorCodeClient::start()
{
    URCL_LOG_INFO("Starting error code client pipeline");
    pipeline_.run();
}

std::deque<urcl::primary_interface::ErrorCode> ErrorCodeClient::getErrorCodes()
{
    return consumer_.getErrorCodesFromQueue();
}
}
