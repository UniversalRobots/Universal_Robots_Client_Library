// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

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
//    * Neither the name of the copyright holder nor the names of its
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

//----------------------------------------------------------------------
/*!\file
 *
 * Unit tests for ErrorCodeMessage::toString(), the generated
 * error_code_texts lookup table, and the dynamic override hook.
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2026-09-04
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <sstream>

#include <gtest/gtest.h>

#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/ur/error_code_overrides.h"
#include "ur_client_library/ur/error_code_texts.h"

using namespace urcl;
using namespace urcl::primary_interface;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace
{
/// Build a minimal ErrorCodeMessage with the given code and argument.
std::unique_ptr<ErrorCodeMessage> makeMsg(int32_t code, int32_t arg)
{
  auto msg = std::make_unique<ErrorCodeMessage>(/*timestamp=*/0, /*source=*/0);
  msg->message_code_ = code;
  msg->message_argument_ = arg;
  msg->report_level_ = ReportLevel::INFO;
  msg->data_type_ = 0;
  msg->data_ = 0;
  return msg;
}

/// Returns the raw "CxxAyy" fallback string for the given code and argument.
std::string fallback(int32_t code, int32_t arg)
{
  std::stringstream ss;
  ss << "C" << code;
  if (arg != -1)
  {
    ss << "A" << arg;
  }
  return ss.str();
}
}  // namespace

// ---------------------------------------------------------------------------
// ERROR_CODE_JSON_VERSION
// ---------------------------------------------------------------------------

TEST(ErrorCodeTextsTest, version_constant_is_not_empty)
{
  EXPECT_FALSE(ERROR_CODE_JSON_VERSION.empty());
}

TEST(ErrorCodeTextsTest, version_constant_looks_like_semver)
{
  // Expect at least one dot (e.g. "40.121.0")
  EXPECT_NE(ERROR_CODE_JSON_VERSION.find('.'), std::string_view::npos);
}

// ---------------------------------------------------------------------------
// getErrorCodeTexts() – static lookup map
// ---------------------------------------------------------------------------

TEST(ErrorCodeTextsTest, code_with_no_arg_is_present)
{
  // Code 0 has no "arg" in the JSON; its key uses the no-arg sentinel 0xFFFFFFFF.
  const auto& map = getErrorCodeTexts();
  constexpr uint64_t no_arg_sentinel = 0xFFFFFFFFULL;
  uint64_t key = (uint64_t(uint32_t(0)) << 32) | no_arg_sentinel;
  EXPECT_NE(map.find(key), map.end());
}

TEST(ErrorCodeTextsTest, code_with_arg_is_present)
{
  // Code 4, arg 1: "Communication issue: Lost communication with Controller"
  const auto& map = getErrorCodeTexts();
  uint64_t key = (uint64_t(uint32_t(4)) << 32) | uint32_t(1);
  EXPECT_NE(map.find(key), map.end());
}

TEST(ErrorCodeTextsTest, absent_code_is_not_present)
{
  const auto& map = getErrorCodeTexts();
  // 0x00FFFFFF is not a real UR error code
  uint64_t key = (uint64_t(uint32_t(0x00FFFFFF)) << 32) | uint32_t(0);
  EXPECT_EQ(map.find(key), map.end());
}

// ---------------------------------------------------------------------------
// getErrorCodeTextOverride() – dynamic C++ hook
// ---------------------------------------------------------------------------

TEST(ErrorCodeTextOverrideTest, code_100_known_robot_mode)
{
  // arg 7 == RobotMode::RUNNING
  auto result = getErrorCodeTextOverride(100, 7);
  ASSERT_TRUE(result.has_value());
  EXPECT_NE(result->find("RUNNING"), std::string::npos);
}

TEST(ErrorCodeTextOverrideTest, code_100_all_defined_robot_modes)
{
  // Every defined RobotMode value should produce a non-empty override string.
  const std::vector<std::pair<int32_t, std::string>> modes = {
    { -1, "NO_CONTROLLER" }, { 0, "DISCONNECTED" },      { 1, "CONFIRM_SAFETY" }, { 2, "BOOTING" },
    { 3, "POWER_OFF" },      { 4, "POWER_ON" },          { 5, "IDLE" },           { 6, "BACKDRIVE" },
    { 7, "RUNNING" },        { 8, "UPDATING_FIRMWARE" },
  };

  for (const auto& [arg, mode_name] : modes)
  {
    auto result = getErrorCodeTextOverride(100, arg);
    ASSERT_TRUE(result.has_value()) << "Expected override for arg=" << arg;
    EXPECT_NE(result->find(mode_name), std::string::npos)
        << "Expected '" << mode_name << "' in override for arg=" << arg << ", got: " << *result;
  }
}

TEST(ErrorCodeTextOverrideTest, code_100_unknown_robot_mode_returns_value)
{
  // An arg that does not correspond to any RobotMode should still return a
  // value (the fallback branch in error_code_overrides.cpp), not nullopt.
  auto result = getErrorCodeTextOverride(100, 127);
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->empty());
}

TEST(ErrorCodeTextOverrideTest, unknown_code_returns_nullopt)
{
  EXPECT_FALSE(getErrorCodeTextOverride(99999, 0).has_value());
  EXPECT_FALSE(getErrorCodeTextOverride(0, 0).has_value());
  EXPECT_FALSE(getErrorCodeTextOverride(1, 1).has_value());
}

// ---------------------------------------------------------------------------
// ErrorCodeMessage::toString() – full lookup chain
// ---------------------------------------------------------------------------

TEST(ErrorCodeMessageTest, toString_uses_cpp_override_for_code_100)
{
  // The C++ override (step 1) takes priority over the static map.
  // Code 100, arg 7 (RUNNING) must come from the override, not the fallback.
  auto msg = makeMsg(100, 7);
  const std::string result = msg->toString();
  EXPECT_NE(result.find("RUNNING"), std::string::npos);
  EXPECT_NE(result, fallback(100, 7));
}

TEST(ErrorCodeMessageTest, toString_uses_static_map_exact_match)
{
  // Code 4, arg 1 is in the generated map; override returns nullopt.
  auto msg = makeMsg(4, 1);
  const std::string result = msg->toString();
  EXPECT_NE(result, fallback(4, 1)) << "Expected map lookup, got fallback string";
  EXPECT_FALSE(result.empty());
}

TEST(ErrorCodeMessageTest, toString_uses_static_map_code_only_with_no_arg)
{
  // Code 0 has no "arg" in the JSON (no-arg sentinel = 0xFFFFFFFF).
  // message_argument_ == -1 casts to uint32_t 0xFFFFFFFF, so it hits the
  // exact-match path directly (step 2) rather than the code-only path (step 3).
  // message_argument_ == -1 is not printed (it signals "no argument").
  auto msg = makeMsg(0, -1);
  EXPECT_EQ(msg->toString(), "C0: No error");
}

TEST(ErrorCodeMessageTest, toString_uses_static_map_code_only_for_arbitrary_arg)
{
  // Code 0 with a non-sentinel arg falls through to the code-only lookup (step 3).
  // toString() always keeps the C<code>A<arg> prefix for traceability.
  auto msg = makeMsg(0, 42);
  EXPECT_EQ(msg->toString(), "C0A42: No error");
}

TEST(ErrorCodeMessageTest, toString_falls_back_for_unknown_code)
{
  auto msg = makeMsg(99999, 0);
  EXPECT_EQ(msg->toString(), fallback(99999, 0));
}

TEST(ErrorCodeMessageTest, toString_omits_argument_when_minus_one)
{
  // A message_argument_ of -1 means "no argument"; the Axx suffix is suppressed.
  auto msg = makeMsg(99999, -1);
  EXPECT_EQ(msg->toString(), "C99999");
}

TEST(ErrorCodeMessageTest, toString_falls_back_for_unknown_code_and_arg)
{
  auto msg = makeMsg(99999, 999);
  EXPECT_EQ(msg->toString(), fallback(99999, 999));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
