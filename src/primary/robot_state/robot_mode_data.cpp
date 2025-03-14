// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include "ur_client_library/primary/robot_state/robot_mode_data.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

#include <iomanip>

namespace urcl
{
namespace primary_interface
{
RobotModeData::RobotModeData(const RobotModeData& pkg) : RobotState(RobotStateType::ROBOT_MODE_DATA)
{
  timestamp_ = pkg.timestamp_;
  is_real_robot_connected_ = pkg.is_real_robot_connected_;
  is_real_robot_enabled_ = pkg.is_real_robot_enabled_;
  is_robot_power_on_ = pkg.is_robot_power_on_;
  is_emergency_stopped_ = pkg.is_emergency_stopped_;
  is_protective_stopped_ = pkg.is_protective_stopped_;
  is_program_running_ = pkg.is_program_running_;
  is_program_paused_ = pkg.is_program_paused_;
  robot_mode_ = pkg.robot_mode_;
  control_mode_ = pkg.control_mode_;
  target_speed_fraction_ = pkg.target_speed_fraction_;
  speed_scaling_ = pkg.speed_scaling_;
  target_speed_fraction_limit_ = pkg.target_speed_fraction_limit_;
  reserved_ = pkg.reserved_;
}

bool RobotModeData::parseWith(comm::BinParser& bp)
{
  bp.parse(timestamp_);
  bp.parse(is_real_robot_connected_);
  bp.parse(is_real_robot_enabled_);
  bp.parse(is_robot_power_on_);
  bp.parse(is_emergency_stopped_);
  bp.parse(is_protective_stopped_);
  bp.parse(is_program_running_);
  bp.parse(is_program_paused_);
  bp.parse(robot_mode_);
  bp.parse(control_mode_);
  bp.parse(target_speed_fraction_);
  bp.parse(speed_scaling_);
  bp.parse(target_speed_fraction_limit_);
  bp.parseRemainder(reserved_);

  return true;
}

bool RobotModeData::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string RobotModeData::toString() const
{
  std::stringstream os;
  os << "Timestamp: " << timestamp_ << std::endl;
  os << "Is real robot connected: " << is_real_robot_connected_ << std::endl;
  os << "Is real robot enabled: " << is_real_robot_enabled_ << std::endl;
  os << "Is robot power on: " << is_robot_power_on_ << std::endl;
  os << "Is emergency stopped: " << is_emergency_stopped_ << std::endl;
  os << "Is protective stopped: " << is_protective_stopped_ << std::endl;
  os << "Is program running: " << is_program_running_ << std::endl;
  os << "Is program paused: " << is_program_paused_ << std::endl;
  os << "Robot mode: " << unsigned(robot_mode_) << std::endl;
  os << "Control mode: " << unsigned(control_mode_) << std::endl;
  os << "Target speed fraction: " << target_speed_fraction_ << std::endl;
  os << "Speed scaling: " << speed_scaling_ << std::endl;
  os << "Target speed fraction limit: " << target_speed_fraction_limit_ << std::endl;
  os << "Reserved: ( " << reserved_.length() << ")";
  for (const char& c : reserved_)
  {
    os << std::hex << static_cast<int>(c) << ", ";
  }
  os << std::endl;

  return os.str();
}

}  // namespace primary_interface
}  // namespace urcl
