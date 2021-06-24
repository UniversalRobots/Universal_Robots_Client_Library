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

#ifndef UR_CLIENT_LIBRARY_TEXT_MESSAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_TEXT_MESSAGE_H_INCLUDED

#include "ur_client_library/rtde/rtde_package.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief This class handles RTDE text messages sent by the robot.
 */
class TextMessage : public RTDEPackage
{
public:
  /*!
   * \brief Creates a new TextMessage object.
   *
   * \param protocol_version Protocol version used for the RTDE communication
   */
  TextMessage(uint16_t protocol_version)
    : RTDEPackage(PackageType::RTDE_TEXT_MESSAGE), protocol_version_(protocol_version)
  {
  }
  virtual ~TextMessage() = default;

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
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  uint8_t message_length_;
  std::string message_;
  uint8_t source_length_;
  std::string source_;
  uint8_t warning_level_;

  uint8_t message_type_;

  uint16_t protocol_version_;
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_TEXT_MESSAGE_H_INCLUDED
