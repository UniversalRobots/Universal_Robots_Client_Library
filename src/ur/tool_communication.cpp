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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-06
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/ur/tool_communication.h"

namespace urcl
{
ToolCommSetup::ToolCommSetup()
  : tool_voltage_(ToolVoltage::OFF)
  , parity_(Parity::ODD)
  , baud_rate_(9600)
  , stop_bits_(1, 2)
  , rx_idle_chars_(1.0, 40.0)
  , tx_idle_chars_(0.0, 40.0)
{
}

void ToolCommSetup::setToolVoltage(const ToolVoltage tool_voltage)
{
  switch (tool_voltage)
  {
    case ToolVoltage::OFF:
    case ToolVoltage::_12V:
    case ToolVoltage::_24V:
      tool_voltage_ = tool_voltage;
      break;
    default:
      throw std::runtime_error("Provided tool voltage is not allowed. The tool voltage should be 0, 12 or 24.");
  }
}

void ToolCommSetup::setToolVoltageT2(const ToolVoltage tool_voltage)
{
  switch (tool_voltage)
  {
    case ToolVoltage::OFF:
    case ToolVoltage::_24V:
    case ToolVoltage::_48V:
      tool_voltage_t2_ = tool_voltage;
      break;
    default:
      throw std::runtime_error("Provided tool T2 voltage is not allowed. The tool T2 voltage should be 0, 24 or 48.");
  }
}

void ToolCommSetup::setBaudRate(const uint32_t baud_rate)
{
  if (baud_rates_allowed_.find(baud_rate) != baud_rates_allowed_.end())
  {
    baud_rate_ = baud_rate;
  }
  else
  {
    throw std::runtime_error("Provided baud rate is not allowed");
  }
}
}  // namespace urcl
