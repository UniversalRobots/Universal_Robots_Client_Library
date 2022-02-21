// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Text 2.0 (the "License");
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
 * \date    20222-02-21
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_GLOBAL_VARIABLES_UPDATE_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_GLOBAL_VARIABLES_UPDATE_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/program_state_message.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief The GlobalVariablesUpdateMessage class handles the key messages sent via the primary UR interface.
 */
class GlobalVariablesUpdateMessage : public ProgramStateMessage
{
public:
  GlobalVariablesUpdateMessage() = delete;
  /*!
   * \brief Creates a new GlobalVariablesUpdateMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  GlobalVariablesUpdateMessage(const uint64_t timestamp)
    : ProgramStateMessage(timestamp, ProgramStateMessageType::GLOBAL_VARIABLES_UPDATE)
  {
  }
  virtual ~GlobalVariablesUpdateMessage() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized text of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);

  /*!
   * \brief Consume this package with a specific consumer.
   *
   * \param consumer Placeholder for the consumer calling this
   *
   * \returns true on success
   */
  virtual bool consumeWith(AbstractPrimaryConsumer& consumer);

  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  uint16_t start_index_;
  std::string variables_;

};
}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_TEXT_MESSAGE_H_INCLUDED

