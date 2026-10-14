// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

#include <gtest/gtest.h>

#include <stdexcept>

#include <ur_client_library/ur/tool_communication.h>

using namespace urcl;

TEST(ToolVoltageTest, rejection_message_lists_allowed_voltages)
{
  EXPECT_EQ(toString(ToolVoltage::_48V), "48");
  EXPECT_EQ(toString(static_cast<ToolVoltage>(5)), "5");
  EXPECT_EQ(toolVoltageNotAllowedMessage("tool", ToolVoltage::_48V, T0_TOOL_VOLTAGES), "The tool voltage should "
                                                                                       "be 0, 12 or 24. The "
                                                                                       "voltage is 48.");
  EXPECT_EQ(toolVoltageNotAllowedMessage("tool T2", static_cast<ToolVoltage>(5), T2_TOOL_VOLTAGES), "The tool T2 "
                                                                                                    "voltage should be "
                                                                                                    "0, 24 or 48. The "
                                                                                                    "voltage is 5.");
}

TEST(ToolCommSetupTest, set_tool_voltage_accepts_t0_voltages)
{
  ToolCommSetup setup;
  for (const ToolVoltage voltage : T0_TOOL_VOLTAGES)
  {
    EXPECT_NO_THROW(setup.setToolVoltage(voltage));
    EXPECT_EQ(setup.getToolVoltage(), voltage);
  }
}

// UrDriver renders the stored voltage as set_tool_voltage() for connector T0 at startup, so a voltage T0 does not
// support must never be stored.
TEST(ToolCommSetupTest, set_tool_voltage_rejects_voltages_t0_does_not_support)
{
  ToolCommSetup setup;
  setup.setToolVoltage(ToolVoltage::_24V);

  try
  {
    setup.setToolVoltage(ToolVoltage::_48V);
    FAIL() << "Expected std::runtime_error";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_EQ(std::string(error.what()), toolVoltageNotAllowedMessage("tool", ToolVoltage::_48V, T0_TOOL_VOLTAGES));
  }
  try
  {
    setup.setToolVoltage(static_cast<ToolVoltage>(5));
    FAIL() << "Expected std::runtime_error";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_EQ(std::string(error.what()),
              toolVoltageNotAllowedMessage("tool", static_cast<ToolVoltage>(5), T0_TOOL_VOLTAGES));
  }
  EXPECT_EQ(setup.getToolVoltage(), ToolVoltage::_24V);
}

TEST(ToolCommSetupTest, tool_voltage_t2_is_unset_by_default)
{
  ToolCommSetup setup;
  EXPECT_FALSE(setup.getToolVoltageT2().has_value());
}

TEST(ToolCommSetupTest, set_tool_voltage_t2_accepts_t2_voltages)
{
  ToolCommSetup setup;
  for (const ToolVoltage voltage : T2_TOOL_VOLTAGES)
  {
    EXPECT_NO_THROW(setup.setToolVoltageT2(voltage));
    EXPECT_EQ(setup.getToolVoltageT2(), voltage);
  }
}

TEST(ToolCommSetupTest, set_tool_voltage_t2_rejects_voltages_t2_does_not_support)
{
  ToolCommSetup setup;
  setup.setToolVoltageT2(ToolVoltage::_48V);

  try
  {
    setup.setToolVoltageT2(ToolVoltage::_12V);
    FAIL() << "Expected std::runtime_error";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_EQ(std::string(error.what()), toolVoltageNotAllowedMessage("tool T2", ToolVoltage::_12V, T2_TOOL_VOLTAGES));
  }
  try
  {
    setup.setToolVoltageT2(static_cast<ToolVoltage>(5));
    FAIL() << "Expected std::runtime_error";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_EQ(std::string(error.what()),
              toolVoltageNotAllowedMessage("tool T2", static_cast<ToolVoltage>(5), T2_TOOL_VOLTAGES));
  }
  EXPECT_EQ(setup.getToolVoltageT2(), ToolVoltage::_48V);
}
