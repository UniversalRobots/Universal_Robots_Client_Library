// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik (ur_robot_driver)
// Copyright 2017, 2018 Simon Rasmussen (refactor)
//
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
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

#include "ur_client_library/primary/robot_message/runtime_exception_message.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
RuntimeExceptionMessage::RuntimeExceptionMessage(const RuntimeExceptionMessage& pkg)
  : RobotMessage(pkg.timestamp_, pkg.source_, RobotMessagePackageType::ROBOT_MESSAGE_RUNTIME_EXCEPTION)
{
  line_number_ = pkg.line_number_;
  column_number_ = pkg.column_number_;
  text_ = pkg.text_;
}

bool RuntimeExceptionMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(line_number_);
  bp.parse(column_number_);
  bp.parseRemainder(text_);

  return true;  // not really possible to check dynamic size packets
}

bool RuntimeExceptionMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string RuntimeExceptionMessage::toString() const
{
  std::stringstream ss;
  ss << "Runtime error in line " << line_number_;
  ss << ", column " << column_number_ << std::endl;
  ss << "Error: " << text_;
  return ss.str();
}
}  // namespace primary_interface
}  // namespace urcl
