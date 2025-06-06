// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_ROBOT_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_ROBOT_MESSAGE_H_INCLUDED

#include "ur_client_library/primary/primary_package.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Possible RobotMessage types
 */
enum class RobotMessagePackageType : uint8_t
{
  ROBOT_MESSAGE_TEXT = 0,
  ROBOT_MESSAGE_PROGRAM_LABEL = 1,
  ROBOT_MESSAGE_POPUP = 2,
  ROBOT_MESSAGE_VERSION = 3,
  ROBOT_MESSAGE_SAFETY_MODE = 5,
  ROBOT_MESSAGE_ERROR_CODE = 6,
  ROBOT_MESSAGE_KEY = 7,
  ROBOT_MESSAGE_REQUEST_VALUE = 9,
  ROBOT_MESSAGE_RUNTIME_EXCEPTION = 10
};

/*!
 * \brief The RobotMessage class is a parent class for the different received robot messages.
 */
class RobotMessage : public PrimaryPackage
{
public:
  /*!
   * \brief Creates a new RobotMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   */
  RobotMessage(const uint64_t timestamp, const uint8_t source) : timestamp_(timestamp), source_(source)
  {
  }
  /*!
   * \brief Creates a new RobotMessage object to be filled from a package.
   *
   * \param timestamp Timestamp of the package
   * \param source The package's source
   * \param message_type The package's message type
   */
  RobotMessage(const uint64_t timestamp, const int8_t source, const RobotMessagePackageType message_type)
    : timestamp_(timestamp), source_(source), message_type_(message_type)
  {
  }
  virtual ~RobotMessage() = default;

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

  uint64_t timestamp_;
  uint8_t source_;
  RobotMessagePackageType message_type_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif /* UR_CLIENT_LIBRARY_ROBOT_MESSAGE_H_INCLUDED */
