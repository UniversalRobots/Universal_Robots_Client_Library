#include <memory>

#include "ur_client_library/comm/stream.h"
#include <ur_client_library/comm/pipeline.h>
#include "ur_client_library/comm/producer.h"
#include "ur_client_library/primary/primary_package.h"
#include <ur_client_library/primary/primary_parser.h>
#include <ur_client_library/ur/error_code_reader.h>
namespace urcl
{
class ErrorCodeClient
{
public: 
  ErrorCodeClient() = delete;
  ErrorCodeClient(comm::URStream<primary_interface::PrimaryPackage>& stream, comm::INotifier& notifier, 
                primary_interface::PrimaryParser& parser);
  ~ErrorCodeClient();

void start();

std::deque<urcl::primary_interface::ErrorCode> getErrorCodes();

private:
  comm::URStream<primary_interface::PrimaryPackage>& stream_;
  urcl::ErrorCodeReader consumer_;
  primary_interface::PrimaryParser parser_;
  comm::URProducer<primary_interface::PrimaryPackage> prod_;
  comm::Pipeline<primary_interface::PrimaryPackage> pipeline_;
};
}
