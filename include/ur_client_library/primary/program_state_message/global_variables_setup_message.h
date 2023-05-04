// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_GLOBAL_VARIABLES_SETUP_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_GLOBAL_VARIABLES_SETUP_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/program_state_message.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief The GlobalVariablesSetupMessage class handles the key messages sent via the primary UR interface.
 */
class GlobalVariablesSetupMessage : public ProgramStateMessage
{
public:
  GlobalVariablesSetupMessage() = delete;
  /*!
   * \brief Creates a new GlobalVariablesSetupMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   */
  GlobalVariablesSetupMessage(const uint64_t timestamp)
    : ProgramStateMessage(timestamp, ProgramStateMessageType::GLOBAL_VARIABLES_SETUP)
  {
  }

  /*!
   * \brief Creates a copy of a GlobalVariablesSetupMessage object.
   *
   * \param pkg The GlobalVariablesSetupMessage object to be copied
   */
  GlobalVariablesSetupMessage(const GlobalVariablesSetupMessage& pkg)
    : ProgramStateMessage(pkg.timestamp_, ProgramStateMessageType::GLOBAL_VARIABLES_SETUP)
  {
    start_index_ = pkg.start_index_;
    variable_names_ = pkg.variable_names_;
  }
  virtual ~GlobalVariablesSetupMessage() = default;

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
  std::string variable_names_;
};
}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_GLOBAL_VARIABLES_SETUP_MESSAGE_H_INCLUDED
