/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <vector>
#include "ur_client_library/comm/bin_parser.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/parser.h"
#include "ur_client_library/primary/package_header.h"
#include "ur_client_library/primary/robot_state.h"
#include "ur_client_library/primary/robot_message.h"
#include "ur_client_library/primary/program_state_message.h"
#include "ur_client_library/primary/robot_state/kinematics_info.h"
#include "ur_client_library/primary/robot_state/robot_mode_data.h"
#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/primary/robot_message/runtime_exception_message.h"
#include "ur_client_library/primary/robot_message/version_message.h"
#include "ur_client_library/primary/robot_message/key_message.h"
#include "ur_client_library/primary/robot_message/text_message.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief The primary specific parser. Interprets a given byte stream as serialized primary
 * packages and parses it accordingly.
 */
class PrimaryParser : public comm::Parser<PrimaryPackage>
{
public:
  PrimaryParser() = default;
  virtual ~PrimaryParser() = default;

  /*!
   * \brief Uses the given BinParser to create package objects from the contained serialization.
   *
   * \param bp A BinParser holding one or more serialized primary packages
   * \param results A vector of pointers to created primary package objects
   *
   * \returns True, if the byte stream could successfully be parsed as primary packages, false
   * otherwise
   */
  bool parse(comm::BinParser& bp, std::vector<std::unique_ptr<PrimaryPackage>>& results)
  {
    int32_t packet_size;
    RobotPackageType type;
    bp.parse(packet_size);
    bp.parse(type);

    switch (type)
    {
      case RobotPackageType::ROBOT_STATE:
      {
        while (!bp.empty())
        {
          if (!bp.checkSize(sizeof(uint32_t)))
          {
            URCL_LOG_ERROR("Failed to read sub-package length, there's likely a parsing error");
            return false;
          }
          uint32_t sub_size = bp.peek<uint32_t>();
          if (!bp.checkSize(static_cast<size_t>(sub_size)))
          {
            URCL_LOG_WARN("Invalid sub-package size of %" PRIu32 " received!", sub_size);
            return false;
          }

          // deconstruction of a sub parser will increment the position of the parent parser
          comm::BinParser sbp(bp, sub_size);
          sbp.consume(sizeof(sub_size));
          RobotStateType type;
          sbp.parse(type);

          std::unique_ptr<PrimaryPackage> packet(stateFromType(type));

          if (packet == nullptr)
          {
            sbp.consume();

            // TODO: create robot state type here
            continue;
          }

          if (!packet->parseWith(sbp))
          {
            URCL_LOG_ERROR("Sub-package parsing of type %d failed!", static_cast<int>(type));
            return false;
          }

          results.push_back(std::move(packet));

          if (!sbp.empty())
          {
            URCL_LOG_ERROR("Sub-package of type %d was not parsed completely!", static_cast<int>(type));
            sbp.debug();
            return false;
          }
        }

        break;
      }

      case RobotPackageType::ROBOT_MESSAGE:
      {
        uint64_t timestamp;
        int8_t source;
        RobotMessagePackageType message_type;

        bp.parse(timestamp);
        bp.parse(source);
        bp.parse(message_type);

        std::unique_ptr<PrimaryPackage> packet(messageFromType(message_type, timestamp, source));
        if (!packet->parseWith(bp))
        {
          URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(message_type));
          return false;
        }

        results.push_back(std::move(packet));
        return true;
        break;
      }

      case RobotPackageType::PROGRAM_STATE_MESSAGE:
      {
        uint64_t timestamp;
        ProgramStateMessageType state_type;
        URCL_LOG_DEBUG("ProgramStateMessage received");

        bp.parse(timestamp);
        bp.parse(state_type);

        URCL_LOG_DEBUG("ProgramStateMessage of type %d received", static_cast<int>(state_type));

        std::unique_ptr<PrimaryPackage> packet(programStateFromType(state_type, timestamp));
        if (!packet->parseWith(bp))
        {
          URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(state_type));
          return false;
        }

        results.push_back(std::move(packet));
        return true;
      }

      default:
      {
        URCL_LOG_DEBUG("Invalid robot package type recieved: %u", static_cast<uint8_t>(type));
        bp.consume();
        return true;
      }
    }
    return true;
  }

private:
  RobotState* stateFromType(RobotStateType type)
  {
    switch (type)
    {
      case RobotStateType::ROBOT_MODE_DATA:
        return new RobotModeData(type);
      case RobotStateType::KINEMATICS_INFO:
        return new KinematicsInfo(type);
      default:
        return new RobotState(type);
    }
  }

  RobotMessage* messageFromType(RobotMessagePackageType type, uint64_t timestamp, uint8_t source)
  {
    switch (type)
    {
      case RobotMessagePackageType::ROBOT_MESSAGE_KEY:
        return new KeyMessage(timestamp, source);
      case RobotMessagePackageType::ROBOT_MESSAGE_VERSION:
        return new VersionMessage(timestamp, source);
      case RobotMessagePackageType::ROBOT_MESSAGE_ERROR_CODE:
        return new ErrorCodeMessage(timestamp, source);
      case RobotMessagePackageType::ROBOT_MESSAGE_RUNTIME_EXCEPTION:
        return new RuntimeExceptionMessage(timestamp, source);
      case RobotMessagePackageType::ROBOT_MESSAGE_TEXT:
        return new TextMessage(timestamp, source);
      default:
        return new RobotMessage(timestamp, source, type);
    }
  }

  ProgramStateMessage* programStateFromType(ProgramStateMessageType type, uint64_t timestamp)
  {
    switch (type)
    {
      // case ProgramStateMessageType::GLOBAL_VARIABLES_SETUP:
      // return new GlobalVariablesSetupMessage(timestamp);
      // case ProgramStateMessageType::TYPE_VARIABLES_UPDATE:
      // return new TypeVariablesUpdateMessage(timestamp);
      default:
        return new ProgramStateMessage(timestamp, type);
    }
  }
};

}  // namespace primary_interface
}  // namespace urcl
