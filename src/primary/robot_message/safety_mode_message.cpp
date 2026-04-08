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

#include "ur_client_library/primary/robot_message/safety_mode_message.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
SafetyModeMessage::SafetyModeMessage(const SafetyModeMessage& pkg)
  : RobotMessage(pkg.timestamp_, pkg.source_, RobotMessagePackageType::ROBOT_MESSAGE_SAFETY_MODE)
{
  message_code_ = pkg.message_code_;
  message_argument_ = pkg.message_argument_;
  safety_mode_type_ = pkg.safety_mode_type_;
  report_data_type_ = pkg.report_data_type_;
  report_data_ = pkg.report_data_;
}

bool SafetyModeMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(message_code_);
  bp.parse(message_argument_);
  bp.parse(safety_mode_type_);
  bp.parse(report_data_type_);
  switch (report_data_type_)
  {
    case 2:
      bp.parse<int32_t>(report_data_);
      break;
    case 3:
      bp.parse<float>(report_data_);
      break;
    default:
      bp.parse<uint32_t>(report_data_);
      break;
  }
  return true;
}

bool SafetyModeMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string SafetyModeMessage::toString() const
{
  std::stringstream ss;
  ss << "SafetyModeMessage \n";
  ss << "Message code: " << message_code_ << "\n";
  ss << "Message argument: " << message_argument_ << "\n";
  ss << "Safety mode type: " << unsigned(safety_mode_type_) << "\n";
  ss << "Report data type: " << report_data_type_ << "\n";
  ss << "Report data: ";
  switch (report_data_type_)
  {
    case 2:
      ss << std::get<int32_t>(report_data_);
      break;
    case 3:
      ss << std::get<float>(report_data_);
      break;
    case 4:
      ss << std::hex << std::get<uint32_t>(report_data_);
      break;
    default:
      ss << std::get<uint32_t>(report_data_);
      break;
  }
  return ss.str();
}
}  // namespace primary_interface

}  // namespace urcl
