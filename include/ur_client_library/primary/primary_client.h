#include <memory>

#include <ur_client_library/comm/stream.h>
#include <ur_client_library/comm/pipeline.h>
#include <ur_client_library/comm/producer.h>
#include <ur_client_library/primary/abstract_primary_consumer.h>
#include <ur_client_library/primary/primary_consumer.h>
#include <ur_client_library/primary/primary_package.h>
#include <ur_client_library/primary/primary_parser.h>
#include <ur_client_library/ur/error_code_reader.h>
namespace urcl
{
namespace primary_interface
{
class PrimaryClient
{
public:
  PrimaryClient() = delete;
  PrimaryClient(comm::URStream<PrimaryPackage>& stream);
  ~PrimaryClient();

  /*!
   * \brief Adds a primary consumer to the list of consumers
   *
   * \param primary_consumer Primary consumer that should be added to the list
   */
  void addPrimaryConsumer (std::shared_ptr<AbstractPrimaryConsumer> primary_consumer);

  /*!
   * \brief Remove a primary consumer from the list of consumers
   *
   * \param primary_consumer Primary consumer that should be removed from the list
   */
  void removePrimaryConsumer(std::shared_ptr<AbstractPrimaryConsumer> primary_consumer);
  void start();

  std::deque<ErrorCode> getErrorCodes();

private:

  // The function is called whenever an error code message is received
  void errorMessageCallback(ErrorCode& code);

  PrimaryParser parser_;
  std::shared_ptr<PrimaryConsumer> consumer_;
  std::unique_ptr<comm::MultiConsumer<PrimaryPackage>> multi_consumer_;

  comm::INotifier notifier_;

  comm::URStream<PrimaryPackage>& stream_;
  std::unique_ptr<comm::URProducer<PrimaryPackage>> prod_;
  std::unique_ptr<comm::Pipeline<PrimaryPackage>> pipeline_;

  std::mutex error_code_queue_mutex_;
  std::deque<ErrorCode> error_code_queue_;
};

}  // namespace primary_interface
}  // namespace urcl
