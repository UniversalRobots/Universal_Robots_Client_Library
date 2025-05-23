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

#pragma once

#include "ur_client_library/types.h"
#include "ur_client_library/primary/robot_state.h"
#include <iostream>

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief The ConfigurationData class handles the configuration data sent via the primary UR interface.
 */
class ConfigurationData : public RobotState
{
public:
  struct JointPositionLimits
  {
    double joint_min_limit;
    double joint_max_limit;
  };

  struct JointMotionLimits
  {
    double joint_max_speed;
    double joint_max_acceleration;
  };

  ConfigurationData() = delete;

  /*!
   * \brief Creates a new ConfigurationData object.
   *
   * \param type The type of RobotState message received
   */
  ConfigurationData(const RobotStateType type) : RobotState(type)
  {
  }

  /*!
   * \brief Creates a copy of a ConfigurationData object.
   *
   * \param pkg The ConfigurationData object to be copied
   */
  ConfigurationData(const ConfigurationData& pkg);

  virtual ~ConfigurationData() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
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

  std::array<JointPositionLimits, 6> joint_position_limits_;
  std::array<JointMotionLimits, 6> joint_motion_limits_;
  double v_joint_default_;
  double a_joint_default_;
  double v_tool_default_;
  double a_tool_default_;
  double eq_radius_;
  urcl::vector6d_t dh_a_;
  urcl::vector6d_t dh_d_;
  urcl::vector6d_t dh_alpha_;
  urcl::vector6d_t dh_theta_;
  int32_t masterboard_version_;
  int32_t controller_box_type_;
  int32_t robot_type_;
  int32_t robot_sub_type_;
};

}  // namespace primary_interface
}  // namespace urcl
