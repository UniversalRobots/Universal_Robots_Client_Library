#ifndef UR_CLIENT_LIBRARY_UR_ERROR_CODE_READER_H_INCLUDED
#define UR_CLIENT_LIBRARY_UR_ERROR_CODE_READER_H_INCLUDED

#include <queue>
#include <mutex>
#include <ur_client_library/comm/pipeline.h>

#include <ur_client_library/primary/robot_message/error_code_message.h>

namespace urcl
{
/*!
 * \brief The ErrorCodeReader class consumes primary packages ignoring all but RobotCommMessage
 * packages to retrieve the latest code reported by the robot.
 */
class ErrorCodeReader : public comm::IConsumer<primary_interface::PrimaryPackage>
{
public:
  virtual ~ErrorCodeReader() = default;

  /*!
   * \brief Empty setup function, as no setup is needed.
   */
  virtual void setupConsumer()
  {
  }
  /*!
   * \brief Tears down the consumer.
   */
  virtual void teardownConsumer()
  {
  }
  /*!
   * \brief Stops the consumer.
   */
  virtual void stopConsumer()
  {
  }
  /*!
   * \brief Handles timeouts.
   */
  virtual void onTimeout()
  {
  }

  /*!
   * \brief Consumes a package and pushes it into a queue if it is an ErrorCodeMessage package.
   *
   * \param product The package to consume
   *
   * \returns True, if the package was consumed correctly
   */
  virtual bool consume(std::shared_ptr<primary_interface::PrimaryPackage> product);

  /*!
   *  \brief Retrieves a list of error codes from the queue if there are any.
   *
   *  \return A list of error codes
   */
  std::deque<primary_interface::ErrorCode> getErrorCodesFromQueue();

private:
  std::deque<primary_interface::ErrorCode> queue_;
  std::mutex queue_mutex_;
};
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_UR_ERROR_CODE_READER_H_INCLUDED
