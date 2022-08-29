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
#include <ur_client_library/rtde/request_protocol_version.h>

using namespace urcl;

TEST(rtde_request_protocol_version, generate_serialized_protocol_v2_request)
{
  uint8_t buffer[4096];
  uint16_t protocol_version = 2;
  size_t expected_size = 5;
  size_t actual_size =
      rtde_interface::RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x05, 0x56, 0x00, 0x02 };

  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_request_protocol_version, generate_serialized_protocol_v1_request)
{
  uint8_t buffer[4096];
  uint16_t protocol_version = 1;
  size_t expected_size = 5;
  size_t actual_size =
      rtde_interface::RequestProtocolVersionRequest::generateSerializedRequest(buffer, protocol_version);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x05, 0x56, 0x00, 0x01 };

  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_request_protocol_version, parse_accepted_request)
{
  uint8_t request_answer[] = { 0x01 };
  comm::BinParser bp(request_answer, sizeof(request_answer));
  rtde_interface::RequestProtocolVersion protocol_version;

  EXPECT_TRUE(protocol_version.parseWith(bp));

  uint8_t expected_anwser = 1;

  EXPECT_EQ(expected_anwser, protocol_version.accepted_);
}

TEST(rtde_request_protocol_version, parse_not_accepted_request)
{
  uint8_t request_answer[] = { 0x00 };
  comm::BinParser bp(request_answer, sizeof(request_answer));
  rtde_interface::RequestProtocolVersion protocol_version;

  EXPECT_TRUE(protocol_version.parseWith(bp));

  uint8_t expected_anwser = 0;

  EXPECT_EQ(expected_anwser, protocol_version.accepted_);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}