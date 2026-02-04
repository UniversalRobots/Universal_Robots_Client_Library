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
 * \author  Felix Exner feex@universal-robots.com
 * \date    2026-02-02
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_KEY_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_KEY_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/robot_message.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Representation of the primary interface's KeyMessage
 */
class KeyMessage : public RobotMessage
{
public:
  KeyMessage() = delete;
  /*!
   * \brief Creates a new KeyMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  KeyMessage(uint64_t timestamp, int8_t source)
    : RobotMessage(timestamp, source, RobotMessagePackageType::ROBOT_MESSAGE_KEY)
  {
  }
  KeyMessage(const KeyMessage& pkg);

  virtual ~KeyMessage() = default;

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
  uint8_t title_length_;
  std::string title_;
  std::string text_;
};
}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_TEXT_MESSAGE_H_INCLUDED
