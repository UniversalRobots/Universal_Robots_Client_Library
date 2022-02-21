// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner <exner@fzi.de>
 * \date    2022-02-21
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PROGRAM_STATE_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PROGRAM_STATE_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/primary_package.h"
#include "ur_client_library/primary/package_header.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Possible ProgramStateMessage types
 */
enum class ProgramStateMessageType : uint8_t
{
  GLOBAL_VARIABLES_SETUP = 0,
  GLOBAL_VARIABLES_UPDATE = 1,
  TYPE_VARIABLES_UPDATE = 2,
};

/*!
 * \brief The ProgramStateMessage class is a parent class for the different received robot messages.
 */
class ProgramStateMessage : public PrimaryPackage
{
public:
  /*!
   * \brief Creates a new ProgramStateMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   */
  ProgramStateMessage(const uint64_t timestamp) : timestamp_(timestamp)
  {
  }

  /*!
   * \brief Creates a new ProgramStateMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param message_type The package's message type
   */
  ProgramStateMessage(const uint64_t timestamp, const ProgramStateMessageType state_type)
    : timestamp_(timestamp), state_type_(state_type)
  {
  }
  virtual ~ProgramStateMessage() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
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

  uint64_t timestamp_;
  ProgramStateMessageType state_type_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif /* UR_CLIENT_LIBRARY_ROBOT_STATE_H_INCLUDED */
