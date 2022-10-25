// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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
#include <ur_client_library/rtde/control_package_start.h>

using namespace urcl;

TEST(rtde_control_package_start, generate_serialized_start_request)
{
  uint8_t buffer[4096];
  size_t expected_size = 3;
  size_t actual_size = rtde_interface::ControlPackageStartRequest::generateSerializedRequest(buffer);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x03, 0x53 };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_control_package_start, parse_accepted_start_request)
{
  rtde_interface::ControlPackageStart start_package;

  uint8_t start_answer[] = { 0x01 };
  comm::BinParser bp(start_answer, sizeof(start_answer));

  EXPECT_TRUE(start_package.parseWith(bp));

  uint8_t expected_answer = 1;
  uint8_t actual_answer = start_package.accepted_;

  EXPECT_EQ(expected_answer, actual_answer);
}

TEST(rtde_control_package_start, parse_not_accepted_start_request)
{
  rtde_interface::ControlPackageStart start_package;

  uint8_t start_answer[] = { 0x00 };
  comm::BinParser bp(start_answer, sizeof(start_answer));

  EXPECT_TRUE(start_package.parseWith(bp));

  uint8_t expected_answer = 0;
  uint8_t actual_answer = start_package.accepted_;

  EXPECT_EQ(expected_answer, actual_answer);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
