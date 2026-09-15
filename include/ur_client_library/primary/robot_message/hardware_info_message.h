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

#ifndef UR_CLIENT_LIBRARY_PRIMARY_HARDWARE_INFO_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_HARDWARE_INFO_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/robot_message.h"

namespace urcl
{
namespace primary_interface
{

/*!
 * \brief Hardware information sent once on the primary interface.
 *
 * This message was introduced in PolyScope 5.26 / 10.14.
 */
class HardwareInfoMessage : public RobotMessage
{
public:
  HardwareInfoMessage() = delete;

  /*!
   * \brief Creates a HardwareInfoMessage to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  HardwareInfoMessage(uint64_t timestamp, int8_t source)
    : RobotMessage(timestamp, source, RobotMessagePackageType::ROBOT_MESSAGE_HARDWARE_INFO)
  {
  }
  virtual ~HardwareInfoMessage() = default;

  virtual bool parseWith(comm::BinParser& bp);
  virtual bool consumeWith(AbstractPrimaryConsumer& consumer);
  virtual std::string toString() const;

  RobotType robot_type_{ 0 };
  uint16_t reserved_1_{ 0 };
  ControlBoxType control_box_type_{ ControlBoxType::UNKNOWN };
  uint16_t reserved_2_{ 0 };
  ToolFlangeType tool_flange_type_{ ToolFlangeType::UNKNOWN };
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_PRIMARY_HARDWARE_INFO_MESSAGE_H_INCLUDED
