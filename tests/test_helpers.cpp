// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <ur_client_library/helpers.h>
#include <ur_client_library/exceptions.h>

using namespace urcl;

TEST(TestHelpers, test_parse_boolean)
{
  EXPECT_TRUE(parseBoolean("true"));
  EXPECT_TRUE(parseBoolean("True"));
  EXPECT_TRUE(parseBoolean("TRUE"));
  EXPECT_TRUE(parseBoolean("on"));
  EXPECT_TRUE(parseBoolean("On"));
  EXPECT_TRUE(parseBoolean("ON"));
  EXPECT_TRUE(parseBoolean("yes"));
  EXPECT_TRUE(parseBoolean("Yes"));
  EXPECT_TRUE(parseBoolean("YES"));
  EXPECT_TRUE(parseBoolean("1"));
  EXPECT_FALSE(parseBoolean("false"));
  EXPECT_FALSE(parseBoolean("False"));
  EXPECT_FALSE(parseBoolean("FALSE"));
  EXPECT_FALSE(parseBoolean("off"));
  EXPECT_FALSE(parseBoolean("Off"));
  EXPECT_FALSE(parseBoolean("OFF"));
  EXPECT_FALSE(parseBoolean("no"));
  EXPECT_FALSE(parseBoolean("No"));
  EXPECT_FALSE(parseBoolean("NO"));
  EXPECT_FALSE(parseBoolean("0"));
  EXPECT_THROW(parseBoolean("notabool"), urcl::UrException);
}
