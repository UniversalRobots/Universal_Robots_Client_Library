// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
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
 * \date    2019-04-09
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_SHELL_CONSUMER_H_INCLUDED
#define UR_CLIENT_LIBRARY_SHELL_CONSUMER_H_INCLUDED

#include "ur_client_library/log.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/package.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief The ShellConsumer class is a simple consumer that writes a readable representation to
 * the logging info channel.
 *
 * @tparam HeaderT Header type of the packages to consume
 */
template <typename T>
class ShellConsumer : public IConsumer<T>
{
public:
  ShellConsumer() = default;
  virtual ~ShellConsumer() = default;

  /*!
   * \brief Consumes a package, writing a human readable representation to the logging.
   *
   * \param product The package to consume
   *
   * \returns True if the output was successful
   */
  virtual bool consume(std::shared_ptr<T> product)
  {
    URCL_LOG_INFO("%s", product->toString().c_str());
    return true;
  }

private:
  /* data */
};
}  // namespace comm
}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_SHELL_CONSUMER_H_INCLUDED
