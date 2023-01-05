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
#include <ur_client_library/comm/package_serializer.h>

using namespace urcl;

TEST(package_serializer, serialize_string)
{
  uint8_t buffer[4096];
  std::string message = "serialized string";
  size_t expected_size = message.size();
  size_t actual_size = comm::PackageSerializer::serialize(buffer, message);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected_buffer[] = { 0x73, 0x65, 0x72, 0x69, 0x61, 0x6c, 0x69, 0x7a, 0x65,
                                0x64, 0x20, 0x73, 0x74, 0x72, 0x69, 0x6e, 0x67 };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected_buffer[i], buffer[i]);
  }
}

TEST(package_serializer, serialize_double)
{
  uint8_t buffer[sizeof(double)];
  size_t expected_size = sizeof(double);
  size_t actual_size = comm::PackageSerializer::serialize(buffer, 2.341);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected_buffer[] = { 0x40, 0x02, 0xba, 0x5e, 0x35, 0x3f, 0x7c, 0xee };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected_buffer[i], buffer[i]);
  }
}

TEST(package_serializer, serialize_int32)
{
  uint8_t buffer[sizeof(int32_t)];
  size_t expected_size = sizeof(int32_t);
  size_t actual_size = comm::PackageSerializer::serialize<int32_t>(buffer, 2341);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected_buffer[] = { 0x00, 0x00, 0x09, 0x25 };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected_buffer[i], buffer[i]);
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
