// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
//
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_ERROR_CODE_MESSAGE_HANDLER_H_INCLUDED
#define UR_CLIENT_LIBRARY_ERROR_CODE_MESSAGE_HANDLER_H_INCLUDED

#include <ur_client_library/log.h>
#include <ur_client_library/primary/primary_package_handler.h>
#include <ur_client_library/primary/robot_message/error_code_message.h>

namespace urcl
{
namespace primary_interface
{
class ErrorCodeMessageHandler : public IPrimaryPackageHandler<ErrorCodeMessage>
{
public:
  ErrorCodeMessageHandler() = default;
  virtual ~ErrorCodeMessageHandler() = default;

  /*!
   * \brief Actual worker function
   *
   * \param pkg package that should be handled
   */
  virtual void handle(ErrorCodeMessage& pkg) override
  {
    std::stringstream out_ss;
    out_ss << "---ErrorCodeMessage---\n" << pkg.toString().c_str() << std::endl;
    switch (pkg.report_level_)
    {
      case ReportLevel::DEBUG:
      case ReportLevel::DEVL_DEBUG:
      {
        URCL_LOG_DEBUG("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::INFO:
      case ReportLevel::DEVL_INFO:
      {
        URCL_LOG_INFO("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::WARNING:
      {
        URCL_LOG_WARN("%s", out_ss.str().c_str());
        break;
      }
      case ReportLevel::VIOLATION:
      case ReportLevel::FAULT:
      case ReportLevel::DEVL_VIOLATION:
      case ReportLevel::DEVL_FAULT:
      {
        URCL_LOG_ERROR("%s", out_ss.str().c_str());
        break;
      }
      default:
      {
        std::stringstream ss;
        ss << "Unknown report level: " << static_cast<int>(pkg.report_level_) << std::endl << out_ss.str();
        URCL_LOG_ERROR("%s", ss.str().c_str());
      }
    }
    data_.reset(new ErrorCodeMessage(pkg));
  }

  /*!
   * \brief Get latest ErrorCodeMessage
   *
   * \return latest ErrorCodeMessage
   */
  virtual std::shared_ptr<ErrorCodeMessage> getData()
  {
    if (data_ == nullptr)
      throw UrException("A ErrorCodeMessage package has not been received yet");
    return data_;
  }

private:
  std::shared_ptr<ErrorCodeMessage> data_;
};
}  // namespace primary_interface
}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_ERROR_CODE_MESSAGE_HANDLER_H_INCLUDED
