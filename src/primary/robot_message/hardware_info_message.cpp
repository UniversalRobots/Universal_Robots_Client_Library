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

#include "ur_client_library/primary/robot_message/hardware_info_message.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

#include <type_traits>

namespace urcl
{
namespace primary_interface
{
bool HardwareInfoMessage::parseWith(comm::BinParser& bp)
{
  uint16_t control_box_type;
  uint16_t tool_flange_type;
  int32_t robot_type;

  bp.parse(robot_type);
  bp.parse(reserved_1_);
  bp.parse(control_box_type);
  bp.parse(reserved_2_);
  bp.parse(tool_flange_type);

  control_box_type_ = static_cast<ControlBoxType>(control_box_type);
  tool_flange_type_ = static_cast<ToolFlangeType>(tool_flange_type);
  robot_type_ = static_cast<RobotType>(robot_type);
  return true;
}

bool HardwareInfoMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string HardwareInfoMessage::toString() const
{
  std::stringstream ss;
  ss << "HardwareInfoMessage:" << std::endl;
  ss << "robot type: " << robotTypeString(robot_type_) << std::endl;
  ss << "control box type: " << static_cast<uint16_t>(control_box_type_) << std::endl;
  ss << "tool flange type: " << static_cast<uint16_t>(tool_flange_type_) << std::endl;
  return ss.str();
}

}  // namespace primary_interface
}  // namespace urcl
