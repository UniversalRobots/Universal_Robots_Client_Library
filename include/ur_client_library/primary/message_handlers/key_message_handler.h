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

#ifndef UR_CLIENT_LIBRARY_KEY_MESSAGE_HANDLER_H_INCLUDED
#define UR_CLIENT_LIBRARY_KEY_MESSAGE_HANDLER_H_INCLUDED

#include <ur_client_library/log.h>
#include <ur_client_library/primary/primary_package_handler.h>
#include <ur_client_library/primary/robot_message/key_message.h>

namespace urcl
{
namespace primary_interface
{
class KeyMessageHandler : public IPrimaryPackageHandler<KeyMessage>
{
public:
  KeyMessageHandler() = default;
  virtual ~KeyMessageHandler() = default;

  /*!
   * \brief Actual worker function
   *
   * \param pkg package that should be handled
   */
  virtual void handle(KeyMessage& pkg) override
  {
    // URCL_LOG_INFO("---KeyMessage---\n%s", pkg.toString().c_str());
    data_.reset(new KeyMessage(pkg));
  }

  /*!
   * \brief Get latest KeyMessage
   *
   * \return latest KeyMessage
   */
  virtual std::shared_ptr<KeyMessage> getData()
  {
    if (data_ == nullptr)
      throw UrException("A KeyMessage package has not been received yet");
    return data_;
  }

private:
  std::shared_ptr<KeyMessage> data_;
};
}  // namespace primary_interface
}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_KEY_MESSAGE_HANDLER_H_INCLUDED
