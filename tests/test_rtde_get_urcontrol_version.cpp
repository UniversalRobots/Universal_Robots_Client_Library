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
#include <ur_client_library/rtde/get_urcontrol_version.h>

using namespace urcl;

TEST(rtde_get_urcontrol_version, generate_serialized_get_urcontrol_version_request)
{
  uint8_t buffer[4096];
  size_t expected_size = 3;
  size_t actual_size = rtde_interface::GetUrcontrolVersionRequest::generateSerializedRequest(buffer);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x03, 0x76 };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_get_urcontrol_version, parse_get_urcontrol_version)
{
  uint8_t urcontrol_version_answer[] = { 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x0c,
                                         0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(urcontrol_version_answer, sizeof(urcontrol_version_answer));
  rtde_interface::GetUrcontrolVersion ur_control_version;

  EXPECT_TRUE(ur_control_version.parseWith(bp));

  VersionInformation expected_version_information;
  expected_version_information.major = 5;
  expected_version_information.minor = 12;
  expected_version_information.bugfix = 2;
  expected_version_information.build = 0;

  EXPECT_EQ(expected_version_information.major, ur_control_version.version_information_.major);
  EXPECT_EQ(expected_version_information.minor, ur_control_version.version_information_.minor);
  EXPECT_EQ(expected_version_information.bugfix, ur_control_version.version_information_.bugfix);
  EXPECT_EQ(expected_version_information.build, ur_control_version.version_information_.build);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
