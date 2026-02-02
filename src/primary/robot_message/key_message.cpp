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

#include "ur_client_library/primary/robot_message/key_message.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
KeyMessage::KeyMessage(const KeyMessage& pkg)
  : RobotMessage(pkg.timestamp_, pkg.source_, RobotMessagePackageType::ROBOT_MESSAGE_KEY)
{
  message_code_ = pkg.message_code_;
  message_argument_ = pkg.message_argument_;
  title_length_ = pkg.title_length_;
  title_ = pkg.title_;
  text_ = pkg.text_;
}

bool KeyMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(message_code_);
  bp.parse(message_argument_);
  bp.parse(title_length_);
  bp.parse(title_, title_length_);
  bp.parseRemainder(text_);

  return true;  // not really possible to check dynamic size packets
}

bool KeyMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string KeyMessage::toString() const
{
  std::stringstream ss;
  ss << "KeyMessage" << std::endl;
  ss << "Message code: " << message_code_ << std::endl;
  ss << "Message argument: " << message_argument_ << std::endl;
  ss << "Title: " << title_ << std::endl;
  ss << "Text: " << text_;
  return ss.str();
}
}  // namespace primary_interface
}  // namespace urcl
