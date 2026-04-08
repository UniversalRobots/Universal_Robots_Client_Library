// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author      Jacob Larsen jala@universal-robots.com
 * \maintainer  Felix Exner feex@universal-robots.com
 * \date        2026-03-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_SAFETY_MODE_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_SAFETY_MODE_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/robot_message.h"
#include "ur_client_library/ur/datatypes.h"
#include <variant>

namespace urcl
{
namespace primary_interface
{
/*!
 *  \brief Representation of the primary interface's safety mode message
 */
class SafetyModeMessage : public RobotMessage
{
public:
  SafetyModeMessage() = delete;

  /*!
   * \brief Creates a new SafetyModeMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  SafetyModeMessage(uint64_t timestamp, int8_t source)
    : RobotMessage(timestamp, source, RobotMessagePackageType::ROBOT_MESSAGE_SAFETY_MODE)
  {
  }
  SafetyModeMessage(const SafetyModeMessage& pkg);

  virtual ~SafetyModeMessage() = default;

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

  int32_t message_code_;
  int32_t message_argument_;
  SafetyMode safety_mode_type_;
  uint32_t report_data_type_;

  /* Content data type varies depending on report_data_type_
  Type 0 and 1: uint32_t
  Type 2: int32_t
  Type 3: float
  Type 4 : uint32_t (but should be displayed as hex, when printed) */
  std::variant<uint32_t, int32_t, float> report_data_;
};
}  // namespace primary_interface

}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_SAFETY_MODE_MESSAGE_H_INCLUDED
