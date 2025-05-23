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

#include "ur_client_library/primary/robot_state/configuration_data.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{

ConfigurationData::ConfigurationData(const ConfigurationData& pkg) : RobotState(RobotStateType::CONFIGURATION_DATA)
{
  joint_position_limits_ = pkg.joint_position_limits_;
  joint_motion_limits_ = pkg.joint_motion_limits_;
  v_joint_default_ = pkg.v_joint_default_;
  a_joint_default_ = pkg.a_joint_default_;
  v_tool_default_ = pkg.v_tool_default_;
  a_tool_default_ = pkg.a_tool_default_;
  eq_radius_ = pkg.eq_radius_;
  dh_a_ = pkg.dh_a_;
  dh_d_ = pkg.dh_d_;
  dh_alpha_ = pkg.dh_alpha_;
  dh_theta_ = pkg.dh_theta_;
  masterboard_version_ = pkg.masterboard_version_;
  controller_box_type_ = pkg.controller_box_type_;
  robot_type_ = pkg.robot_type_;
  robot_sub_type_ = pkg.robot_sub_type_;
}

bool ConfigurationData::parseWith(comm::BinParser& bp)
{
  for (auto& joint_limits : joint_position_limits_)
  {
    bp.parse(joint_limits.joint_min_limit);
    bp.parse(joint_limits.joint_max_limit);
  }
  for (auto& motion_limits : joint_motion_limits_)
  {
    bp.parse(motion_limits.joint_max_speed);
    bp.parse(motion_limits.joint_max_acceleration);
  }
  bp.parse(v_joint_default_);
  bp.parse(a_joint_default_);
  bp.parse(v_tool_default_);
  bp.parse(a_tool_default_);
  bp.parse(eq_radius_);
  bp.parse(dh_a_);
  bp.parse(dh_d_);
  bp.parse(dh_alpha_);
  bp.parse(dh_theta_);
  bp.parse(masterboard_version_);
  bp.parse(controller_box_type_);
  bp.parse(robot_type_);
  bp.parse(robot_sub_type_);

  return true;
}

bool ConfigurationData::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string ConfigurationData::toString() const
{
  std::stringstream os;
  os << "ConfigurationData:" << std::endl;
  os << "Joint position limits: [" << std::endl;
  for (auto& limits : joint_position_limits_)
  {
    os << " {min: " << limits.joint_min_limit << ", max " << limits.joint_max_limit << "} " << std::endl;
  }
  os << "]" << std::endl;
  os << "Joint motion limits: [" << std::endl;
  for (auto& limits : joint_motion_limits_)
  {
    os << "  {max joint speed: " << limits.joint_max_speed
       << ", max joint acceleration: " << limits.joint_max_acceleration << "} " << std::endl;
  }
  os << "]" << std::endl;
  os << "Velocity joint default: " << v_joint_default_ << std::endl;
  os << "Acceleration joint default: " << a_joint_default_ << std::endl;
  os << "Velocity tool default: " << v_tool_default_ << std::endl;
  os << "Acceleration tool default: " << a_tool_default_ << std::endl;
  os << "Equivalent radius: " << eq_radius_ << std::endl;
  os << "dh_a: [";
  for (size_t i = 0; i < dh_a_.size(); ++i)
  {
    os << dh_a_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_d: [";
  for (size_t i = 0; i < dh_d_.size(); ++i)
  {
    os << dh_d_[i] << " ";
  }
  os << "]" << std::endl;

  os << "dh_alpha: [";
  for (size_t i = 0; i < dh_alpha_.size(); ++i)
  {
    os << dh_alpha_[i] << " ";
  }
  os << "]" << std::endl;
  os << "dh_theta: [";
  for (size_t i = 0; i < dh_theta_.size(); ++i)
  {
    os << dh_theta_[i] << " ";
  }
  os << "]" << std::endl;

  os << "Masterboard version: " << masterboard_version_ << std::endl;
  os << "Controller box type: " << controller_box_type_ << std::endl;
  os << "Robot type: " << robot_type_ << std::endl;
  os << "Robot sub type: " << robot_sub_type_ << std::endl;

  return os.str();
}

}  // namespace primary_interface
}  // namespace urcl