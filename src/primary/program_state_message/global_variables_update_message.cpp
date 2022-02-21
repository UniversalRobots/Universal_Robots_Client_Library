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
 * \date    2022-02-21
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/log.h"
#include "ur_client_library/primary/program_state_message/global_variables_update_message.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

namespace urcl
{
namespace primary_interface
{
bool GlobalVariablesUpdateMessage::parseWith(comm::BinParser& bp)
{
  bp.parse(start_index_);
  bp.parseRemainder(variables_);

  return true;  // not really possible to check dynamic size packets
}

bool GlobalVariablesUpdateMessage::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string GlobalVariablesUpdateMessage::toString() const
{
  std::stringstream ss;
  ss << "start index: " << start_index_ << std::endl;
  ss << "variables: (" << variables_.length() << ")";
  for (const char& c : variables_)
  {
    ss << std::hex << static_cast<int>(c) << " ";
  }
  return ss.str();
}
}  // namespace primary_interface
}  // namespace urcl

