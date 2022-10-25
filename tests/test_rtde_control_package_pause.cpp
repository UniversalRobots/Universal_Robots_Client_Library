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
#include <ur_client_library/rtde/control_package_pause.h>

using namespace urcl;

TEST(rtde_control_package_pause, generate_serialized_pause_request)
{
  uint8_t buffer[4096];
  size_t expected_size = 3;
  size_t actual_size = rtde_interface::ControlPackagePauseRequest::generateSerializedRequest(buffer);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x03, 0x50 };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_control_package_pause, parse_accepted_pause_request)
{
  rtde_interface::ControlPackagePause pause_package;

  uint8_t pause_answer[] = { 0x01 };
  comm::BinParser bp(pause_answer, sizeof(pause_answer));

  EXPECT_TRUE(pause_package.parseWith(bp));

  uint8_t expected_answer = 1;
  uint8_t actual_answer = pause_package.accepted_;

  EXPECT_EQ(expected_answer, actual_answer);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}