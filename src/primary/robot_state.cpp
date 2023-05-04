// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik (ur_robot_driver)
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
#include <sstream>

#include "ur_client_library/primary/robot_state.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
bool RobotState::parseWith(comm::BinParser& bp)
{
  return PrimaryPackage::parseWith(bp);
}

bool RobotState::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string RobotState::toString() const
{
  std::stringstream out_ss;
  out_ss << "Type: " << static_cast<int>(state_type_) << std::endl;
  // ss << PrimaryPackage::toString();
  switch (state_type_)
  {
    case RobotStateType::ROBOT_MODE_DATA:
    {
      out_ss << RobotModeData(state_type_).toString();
      break;
    }
    case RobotStateType::JOINT_DATA:
    {
      out_ss << JointData(state_type_).toString();
      break;
    }
    case RobotStateType::TOOL_DATA:
    {
      URCL_LOG_INFO("TOOL_DATA");
      break;
    }
    case RobotStateType::MASTERBOARD_DATA:
    {
      URCL_LOG_INFO("MASTERBOARD_DATA");
      break;
    }
    case RobotStateType::CARTESIAN_INFO:
    {
      out_ss << CartesianInfo(state_type_).toString();
      break;
    }
    case RobotStateType::KINEMATICS_INFO:
    {
      out_ss << KinematicsInfo(state_type_).toString();
      break;
    }
    case RobotStateType::CONFIGURATION_DATA:
    {
      URCL_LOG_INFO("CONFIGURATION_DATA");
      break;
    }
    case RobotStateType::FORCE_MODE_DATA:
    {
      out_ss << ForceModeData(state_type_).toString();
      break;
    }
    case RobotStateType::ADDITIONAL_INFO:
    {
      out_ss << AdditionalInfo(state_type_).toString();
      break;
    }
    default:
    {
      std::stringstream ss;
      ss << "Unknown RobotStateType: " << static_cast<int>(state_type_) << std::endl << out_ss.str();
      URCL_LOG_ERROR("%s", ss.str().c_str());
    }
  }
  return out_ss.str().c_str();
}
}  // namespace primary_interface
}  // namespace urcl
