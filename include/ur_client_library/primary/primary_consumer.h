// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-
//
// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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

#ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED

#include "ur_client_library/log.h"
#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/primary/primary_package_handler.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"
#include "ur_client_library/primary/message_handlers/key_message_handler.h"
#include "ur_client_library/primary/message_handlers/version_message_handler.h"
#include "ur_client_library/primary/message_handlers/text_message_handler.h"
#include "ur_client_library/primary/message_handlers/runtime_exception_message_handler.h"
#include "ur_client_library/primary/message_handlers/error_code_message_handler.h"
#include "ur_client_library/primary/message_handlers/robot_mode_data_message_handler.h"
#include "ur_client_library/primary/message_handlers/joint_data_message_handler.h"
#include "ur_client_library/primary/message_handlers/cartesian_info_message_handler.h"
#include "ur_client_library/primary/message_handlers/force_mode_data_message_handler.h"
#include "ur_client_library/primary/message_handlers/addtional_info_message_handler.h"
#include "ur_client_library/primary/message_handlers/global_variables_update_message_handler.h"
#include "ur_client_library/primary/message_handlers/global_variables_setup_message_handler.h"
#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/primary/robot_message/key_message.h"
#include "ur_client_library/primary/robot_message/runtime_exception_message.h"
#include "ur_client_library/primary/robot_message/text_message.h"
#include "ur_client_library/primary/robot_message/version_message.h"
#include "ur_client_library/primary/robot_state/kinematics_info.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Primary consumer implementation
 *
 * This class implements an AbstractPrimaryConsumer such that it can consume all incoming primary
 * messages. However, actual work will be done by workers for each specific type.
 */
class PrimaryConsumer : public AbstractPrimaryConsumer
{
public:
  PrimaryConsumer()
  {
    URCL_LOG_INFO("Constructing primary consumer");
    key_message_worker_.reset(new KeyMessageHandler());
    text_message_worker_.reset(new TextMessageHandler());
    version_message_worker_.reset(new VersionMessageHandler());
    error_code_message_worker_.reset(new ErrorCodeMessageHandler());
    cartesian_info_message_worker_.reset(new CartesianInfoHandler());
    force_mode_data_message_worker_.reset(new ForceModeDataHandler());
    additional_info_message_worker_.reset(new AdditionalInfoHandler());
    robot_mode_data_message_worker_.reset(new RobotModeDataHandler());
    joint_data_message_worker_.reset(new JointDataHandler());
    runtime_exception_message_worker_.reset(new RuntimeExceptionHandler());
    global_variables_update_message_worker_.reset(new GlobalVariablesUpdateMessageHandler());
    global_variables_setup_message_worker_.reset(new GlobalVariablesSetupMessageHandler());
    URCL_LOG_INFO("Constructed primary consumer");
  }
  virtual ~PrimaryConsumer() = default;

  // These need to be overriden
  virtual bool consume(RobotMessage& msg) override
  {
    // URCL_LOG_INFO("---RobotMessage:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(RobotState& msg) override
  {
    // URCL_LOG_INFO("---RobotState:---\n%s", msg.toString().c_str());
    return true;
  }
  virtual bool consume(ProgramStateMessage& msg) override
  {
    // URCL_LOG_INFO("---ProgramStateMessage---\n%s", msg.toString().c_str());
    return true;
  }

  /*!
   * \brief Handle a GlobalVariablesSetupMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(GlobalVariablesSetupMessage& pkg) override
  {
    if (global_variables_setup_message_worker_ != nullptr)
    {
      global_variables_setup_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a VersionMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(VersionMessage& pkg) override
  {
    if (version_message_worker_ != nullptr)
    {
      version_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a RuntimeExceptionMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(RuntimeExceptionMessage& pkg) override
  {
    if (runtime_exception_message_worker_ != nullptr)
    {
      runtime_exception_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a TextMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(TextMessage& pkg) override
  {
    if (text_message_worker_ != nullptr)
    {
      text_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a GlobalVariablesUpdateMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(GlobalVariablesUpdateMessage& pkg) override
  {
    if (global_variables_update_message_worker_ != nullptr)
    {
      global_variables_update_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a RobotModeData
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(RobotModeData& pkg) override
  {
    if (robot_mode_data_message_worker_ != nullptr)
    {
      robot_mode_data_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a JointData
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(JointData& pkg) override
  {
    if (joint_data_message_worker_ != nullptr)
    {
      joint_data_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a AdditionalInfo
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(AdditionalInfo& pkg) override
  {
    if (additional_info_message_worker_ != nullptr)
    {
      additional_info_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a ForceModeData
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(ForceModeData& pkg) override
  {
    if (force_mode_data_message_worker_ != nullptr)
    {
      force_mode_data_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a CartesianInfo
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(CartesianInfo& pkg) override
  {
    if (cartesian_info_message_worker_ != nullptr)
    {
      cartesian_info_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a KinematicsInfo
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(KinematicsInfo& pkg) override
  {
    if (kinematics_info_message_worker_ != nullptr)
    {
      kinematics_info_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a KeyMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(KeyMessage& pkg) override
  {
    if (key_message_worker_ != nullptr)
    {
      key_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }

  /*!
   * \brief Handle a ErrorCodeMessage
   *
   * \returns True if there's a handler for this message type registered. False otherwise.
   */
  virtual bool consume(ErrorCodeMessage& pkg) override
  {
    if (error_code_message_worker_ != nullptr)
    {
      error_code_message_worker_->handle(pkg);
      return true;
    }
    return false;
  }
  void setKinematicsInfoHandler(const std::shared_ptr<IPrimaryPackageHandler<KinematicsInfo>>& handler)
  {
    kinematics_info_message_worker_ = handler;
  }

  std::shared_ptr<IPrimaryPackageHandler<KeyMessage>> key_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<TextMessage>> text_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<VersionMessage>> version_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<ErrorCodeMessage>> error_code_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<CartesianInfo>> cartesian_info_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<KinematicsInfo>> kinematics_info_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<ForceModeData>> force_mode_data_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<AdditionalInfo>> additional_info_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<RobotModeData>> robot_mode_data_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<JointData>> joint_data_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<RuntimeExceptionMessage>> runtime_exception_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<GlobalVariablesUpdateMessage>> global_variables_update_message_worker_;
  std::shared_ptr<IPrimaryPackageHandler<GlobalVariablesSetupMessage>> global_variables_setup_message_worker_;
};
}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_PRIMARY_CONSUMER_H_INCLUDED
