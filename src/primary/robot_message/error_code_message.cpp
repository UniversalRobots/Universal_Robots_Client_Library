// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \date    2020-04-30
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/ur/error_code_overrides.h"
#include "ur_client_library/ur/error_code_texts.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
bool ErrorCodeMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(message_code_);
  bp.parse(message_argument_);
  int32_t report_level;
  bp.parse(report_level);
  report_level_ = static_cast<ReportLevel>(report_level);
  bp.parse(data_type_);
  bp.parse(data_);
  bp.parseRemainder(text_);

  return true;  // not really possible to check dynamic size packets
}

bool ErrorCodeMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string ErrorCodeMessage::toString() const
{
  std::stringstream ss;
  ss << "C" << message_code_ << "A" << message_argument_;

  // 1. Dynamic C++ override (highest priority — can call arbitrary functions)
  if (auto text = getErrorCodeTextOverride(message_code_, message_argument_))
  {
    ss << ": " << *text;
  }
  else
  {
    const auto& map = getErrorCodeTexts();

    // 2. Exact match: (code, arg)
    uint64_t key = (uint64_t(uint32_t(message_code_)) << 32) | uint32_t(message_argument_);
    auto it = map.find(key);
    if (it != map.end())
    {
      ss << ": " << it->second;
    }
    else
    {
      // 3. Code-only match: entries with no arg in the JSON use 0xFFFFFFFF as sentinel.
      //    Note: message_argument_ == -1 also maps to 0xFFFFFFFF, so it hits here directly.
      key = (uint64_t(uint32_t(message_code_)) << 32) | 0xFFFFFFFFULL;
      it = map.find(key);
      if (it != map.end())
      {
        ss << ": " << it->second;
      }
    }
  }

  return ss.str();
}

}  // namespace primary_interface
}  // namespace urcl
