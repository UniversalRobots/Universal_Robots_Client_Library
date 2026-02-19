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
#include <ur_client_library/comm/bin_parser.h>

using namespace urcl;

TEST(bin_parser, parse_string)
{
  uint8_t buffer[] = { 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20,
                       0x62, 0x65, 0x20, 0x70, 0x61, 0x72, 0x73, 0x65, 0x64 };
  comm::BinParser bp(buffer, sizeof(buffer));

  std::string expected_message = "String to be parsed";
  std::string parsed_message;
  bp.parse(parsed_message, sizeof(buffer));

  EXPECT_EQ(expected_message, parsed_message);

  // Parse partial string
  comm::BinParser bp1(buffer, sizeof(buffer));
  size_t partial_string = 6;
  parsed_message.clear();
  bp1.parse(parsed_message, partial_string);

  EXPECT_EQ("String", parsed_message);

  // Parse the rest of the string
  parsed_message.clear();
  bp1.parseRemainder(parsed_message);

  EXPECT_EQ(" to be parsed", parsed_message);
}

TEST(bin_parser, parse_array)
{
  // Parse vector3d_t
  uint8_t buffer3d[] = { 0x40, 0x01, 0x99, 0x99, 0x99, 0x99, 0x99, 0x9a, 0xc0, 0x08, 0xcc, 0xcc,
                         0xcc, 0xcc, 0xcc, 0xcd, 0x40, 0x11, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcd };
  comm::BinParser bp3d(buffer3d, sizeof(buffer3d));

  vector3d_t expected_values3d = { 2.2, -3.1, 4.45 };
  vector3d_t parsed_values3d;
  bp3d.parse(parsed_values3d);

  for (unsigned int i = 0; i < parsed_values3d.size(); ++i)
  {
    EXPECT_EQ(expected_values3d[i], parsed_values3d[i]);
  }

  // Parse vector6d_t
  uint8_t buffer6d[] = {
    0xc0, 0x01, 0x99, 0x99, 0x99, 0x99, 0x99, 0x9a, 0x40, 0x08, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcd,
    0x40, 0x11, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcd, 0x3f, 0xf1, 0x99, 0x99, 0x99, 0x99, 0x99, 0x9a,
    0x40, 0x08, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcd, 0xc0, 0x10, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcd
  };
  comm::BinParser bp6d(buffer6d, sizeof(buffer6d));

  vector6d_t expected_values6d = { -2.2, 3.1, 4.45, 1.1, 3.1, -4.2 };
  vector6d_t parsed_values6d;
  bp6d.parse(parsed_values6d);

  for (unsigned int i = 0; i < parsed_values6d.size(); ++i)
  {
    EXPECT_EQ(expected_values6d[i], parsed_values6d[i]);
  }

  // Parse vector6int32_t
  uint8_t buffer6i[] = { 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff,
                         0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x05 };
  comm::BinParser bp6i(buffer6i, sizeof(buffer6i));

  vector6int32_t expected_values6i = { -2, 3, -1, 4, 14, 5 };
  vector6int32_t parsed_values6i;
  bp6i.parse(parsed_values6i);

  for (unsigned int i = 0; i < parsed_values6i.size(); ++i)
  {
    EXPECT_EQ(expected_values6i[i], parsed_values6i[i]);
  }

  // Parse vector6uint32_t
  uint8_t buffer6u[] = { 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x05 };
  comm::BinParser bp6u(buffer6u, sizeof(buffer6u));

  vector6uint32_t expected_values6u = { 2, 3, 1, 4, 14, 5 };
  vector6int32_t parsed_values6u;
  bp6u.parse(parsed_values6u);

  for (unsigned int i = 0; i < parsed_values6u.size(); ++i)
  {
    EXPECT_EQ(expected_values6u[i], parsed_values6u[i]);
  }

  // Parse std::array
  uint8_t buffer4i[] = { 0x00, 0x00, 0x00, 0x02, 0x00,  0x00,  0x00,  0x05,
                         0x00, 0x00, 0x00, 0x03, 0x0ff, 0x0ff, 0x0ff, 0x0ff };
  comm::BinParser bp4i(buffer4i, sizeof(buffer4i));

  std::array<int32_t, 4> expected_values4i = { 2, 5, 3, -1 };
  std::array<int32_t, 4> parsed_values4i;
  bp4i.parse(parsed_values4i);

  for (unsigned int i = 0; i < parsed_values4i.size(); ++i)
  {
    EXPECT_EQ(expected_values4i[i], parsed_values4i[i]);
  }
}

TEST(bin_parser, parse_vector6int32_signed)
{
  // Big-endian 6 x int32_t: -1, -256, 255, 0x7fffffff, -0x80000000, 42
  uint8_t buffer[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff,
                       0x7f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2a };
  comm::BinParser bp(buffer, sizeof(buffer));

  vector6int32_t expected = { -1, -256, 255, 2147483647, static_cast<int32_t>(0x80000000u), 42 };
  vector6int32_t parsed;
  bp.parse(parsed);

  for (size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(expected[i], parsed[i]) << "at index " << i;
  }
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, parse_vector6uint32_unsigned)
{
  // Big-endian 6 x uint32_t: 0, 1, 255, 0xffffffff, 0x80000000, 0x7fffffff
  uint8_t buffer[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xff,
                       0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff };
  comm::BinParser bp(buffer, sizeof(buffer));

  vector6uint32_t expected = { 0u, 1u, 255u, 4294967295u, 2147483648u, 2147483647u };
  vector6uint32_t parsed;
  bp.parse(parsed);

  for (size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(expected[i], parsed[i]) << "at index " << i;
  }
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, parse_data_types)
{
  // Parse float value
  uint8_t buffer_float[] = { 0x41, 0xb2, 0xb8, 0x52 };
  comm::BinParser bp_float(buffer_float, sizeof(buffer_float));

  float expected_float = 22.34;
  float parsed_float;
  bp_float.parse(parsed_float);

  EXPECT_EQ(expected_float, parsed_float);

  // Parse float value from buffer (float overload reads only 4 bytes; remaining bytes stay in buffer)
  uint8_t buffer_float_4bytes[] = { 0x41, 0xb2, 0xb8, 0x52, 0xcc, 0x55, 0x00, 0x00 };
  comm::BinParser bp_float_4bytes(buffer_float_4bytes, sizeof(buffer_float_4bytes));

  float expected_float_4bytes = 22.34f;
  float parsed_float_4bytes;
  bp_float_4bytes.parse(parsed_float_4bytes);

  EXPECT_EQ(expected_float_4bytes, parsed_float_4bytes);
  EXPECT_FALSE(bp_float_4bytes.empty());  // 4 bytes remain unparsed

  // Parse double value (2.0 has exact IEEE 754 big-endian representation)
  uint8_t buffer_double[] = { 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp_double(buffer_double, sizeof(buffer_double));

  constexpr double kExpectedDouble = 2.0;
  double parsed_double;
  bp_double.parse(parsed_double);

  EXPECT_DOUBLE_EQ(kExpectedDouble, parsed_double);

  // Parse boolean
  uint8_t buffer_bool[] = { 0x01 };
  comm::BinParser bp_bool(buffer_bool, sizeof(buffer_bool));

  bool expected_bool = true;
  bool parsed_bool;
  bp_bool.parse(parsed_bool);

  EXPECT_EQ(expected_bool, parsed_bool);

  // Parse int32_t
  uint8_t buffer_int[] = { 0xff, 0xff, 0xff, 0xea };
  comm::BinParser bp_int(buffer_int, sizeof(buffer_int));

  int32_t expected_int = -22;
  int32_t parsed_int;
  bp_int.parse<int32_t>(parsed_int);

  EXPECT_EQ(expected_int, parsed_int);
}

TEST(bin_parser, check_size)
{
  uint8_t buffer[] = { 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20,
                       0x62, 0x65, 0x20, 0x70, 0x61, 0x72, 0x73, 0x65, 0x64 };
  comm::BinParser bp(buffer, sizeof(buffer));
  size_t len = sizeof(buffer);

  // The whole buffer length should remain unparsed in the buffer
  EXPECT_TRUE(bp.checkSize(len));

  std::string parsed_string;
  bp.parse(parsed_string, len - (len - 1));

  // Only one uint8_t should remain unparsed in the buffer
  EXPECT_FALSE(bp.checkSize(len));
  EXPECT_TRUE(bp.checkSize<uint8_t>());

  bp.parseRemainder(parsed_string);

  // No bytes should remain unparsed in the buffer
  EXPECT_FALSE(bp.checkSize<uint8_t>());
}

TEST(bin_parser, consume)
{
  // The buffer represents the string 'String to be parsed'
  uint8_t buffer[] = { 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20,
                       0x62, 0x65, 0x20, 0x70, 0x61, 0x72, 0x73, 0x65, 0x64 };
  comm::BinParser bp(buffer, sizeof(buffer));
  bp.consume();

  EXPECT_TRUE(bp.empty());

  comm::BinParser bp1(buffer, sizeof(buffer));
  bp1.consume(10);

  std::string expected_message = "be parsed";
  std::string parsed_message;
  bp1.parseRemainder(parsed_message);
  EXPECT_EQ(expected_message, parsed_message);

  // Once the rest of the message has been parsed the parser should be empty
  EXPECT_TRUE(bp1.empty());
}

TEST(bin_parser, raw_data_parser)
{
  uint8_t buffer[] = { 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20,
                       0x62, 0x65, 0x20, 0x70, 0x61, 0x72, 0x73, 0x65, 0x64 };
  comm::BinParser bp(buffer, sizeof(buffer));

  std::unique_ptr<uint8_t[]> raw_data;
  size_t size;
  bp.rawData(raw_data, size);

  // The buffer should be empty afterwards
  EXPECT_TRUE(bp.empty());

  for (unsigned int i = 0; i < size; ++i)
  {
    EXPECT_EQ(buffer[i], raw_data.get()[i]);
  }
}

TEST(bin_parser, bitset_parser)
{
  uint8_t buffer[] = { 0x00, 0x00, 0x00, 0x01 };
  comm::BinParser bp(buffer, sizeof(buffer));

  std::bitset<4> expected_set = std::bitset<4>(1);
  std::bitset<4> parsed_set;
  bp.parse<int32_t>(parsed_set);

  EXPECT_EQ(expected_set, parsed_set);
}

TEST(bin_parser, parse_outside_buffer_length)
{
  uint8_t buffer[] = { 0x00, 0x00, 0x00, 0x01 };
  comm::BinParser bp(buffer, sizeof(buffer));

  int32_t expected_int = 1;
  int32_t parsed_int;
  bp.parse<int32_t>(parsed_int);

  EXPECT_EQ(expected_int, parsed_int);

  // Parse outside buffer length
  EXPECT_THROW(bp.parse<int32_t>(parsed_int), UrException);
}

TEST(bin_parser, bin_parser_parent)
{
  // The buffer represents the string 'String to be parsed'
  uint8_t buffer[] = { 0x53, 0x74, 0x72, 0x69, 0x6e, 0x67, 0x20, 0x74, 0x6f, 0x20,
                       0x62, 0x65, 0x20, 0x70, 0x61, 0x72, 0x73, 0x65, 0x64 };
  comm::BinParser bp_parent(buffer, sizeof(buffer));

  comm::BinParser bp_child(bp_parent, sizeof(buffer) - 10);

  std::string expected_message = "String to";
  std::string parsed_message;
  bp_child.parseRemainder(parsed_message);

  EXPECT_EQ(expected_message, parsed_message);

  // Destroying the child parser, should move the parents parser
  bp_child.~BinParser();

  expected_message = " be parsed";
  parsed_message.clear();
  bp_parent.parseRemainder(parsed_message);

  EXPECT_EQ(expected_message, parsed_message);
}

// Tests for integer decoding (big-endian to host) - covers logic in bin_parser.cpp
TEST(bin_parser, decode_uint8)
{
  uint8_t buffer[] = { 0xab };
  comm::BinParser bp(buffer, sizeof(buffer));
  uint8_t val;
  bp.parse(val);
  EXPECT_EQ(val, 0xab);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, decode_uint16)
{
  // Big-endian 0x1234
  uint8_t buffer[] = { 0x12, 0x34 };
  comm::BinParser bp(buffer, sizeof(buffer));
  uint16_t val;
  bp.parse(val);
  EXPECT_EQ(val, 0x1234u);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, decode_uint32)
{
  // Big-endian 0x12345678
  uint8_t buffer[] = { 0x12, 0x34, 0x56, 0x78 };
  comm::BinParser bp(buffer, sizeof(buffer));
  uint32_t val;
  bp.parse(val);
  EXPECT_EQ(val, 0x12345678u);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, decode_uint64)
{
  // Big-endian 0x123456789abcdef0
  uint8_t buffer[] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 };
  comm::BinParser bp(buffer, sizeof(buffer));
  uint64_t val;
  bp.parse(val);
  EXPECT_EQ(val, 0x123456789abcdef0ULL);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, decode_int16)
{
  // Big-endian -1 (0xffff)
  uint8_t buffer[] = { 0xff, 0xff };
  comm::BinParser bp(buffer, sizeof(buffer));
  int16_t val;
  bp.parse(val);
  EXPECT_EQ(val, -1);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, decode_int64)
{
  // Big-endian -1
  uint8_t buffer[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
  comm::BinParser bp(buffer, sizeof(buffer));
  int64_t val;
  bp.parse(val);
  EXPECT_EQ(val, -1);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, peek_does_not_advance)
{
  uint8_t buffer[] = { 0x12, 0x34, 0x56, 0x78 };
  comm::BinParser bp(buffer, sizeof(buffer));

  uint32_t a = bp.peek<uint32_t>();
  uint32_t b = bp.peek<uint32_t>();
  EXPECT_EQ(a, 0x12345678u);
  EXPECT_EQ(b, 0x12345678u);

  uint32_t c;
  bp.parse(c);
  EXPECT_EQ(c, 0x12345678u);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, parse_string_with_length_prefix)
{
  // Length 5, then "hello"
  uint8_t buffer[] = { 0x05, 0x68, 0x65, 0x6c, 0x6c, 0x6f };
  comm::BinParser bp(buffer, sizeof(buffer));

  std::string s;
  bp.parse(s);
  EXPECT_EQ(s, "hello");
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, parse_string_with_length_prefix_empty)
{
  uint8_t buffer[] = { 0x00 };
  comm::BinParser bp(buffer, sizeof(buffer));

  std::string s;
  bp.parse(s);
  EXPECT_TRUE(s.empty());
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, parse_bool_false)
{
  uint8_t buffer[] = { 0x00 };
  comm::BinParser bp(buffer, sizeof(buffer));
  bool val;
  bp.parse(val);
  EXPECT_FALSE(val);
  EXPECT_TRUE(bp.empty());
}

TEST(bin_parser, peek_throws_when_past_end)
{
  uint8_t buffer[] = { 0x12, 0x34 };
  comm::BinParser bp(buffer, sizeof(buffer));

  EXPECT_THROW(bp.peek<uint32_t>(), UrException);
  uint16_t val;
  bp.parse(val);
  EXPECT_EQ(val, 0x1234u);
  EXPECT_THROW(bp.peek<uint16_t>(), UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
