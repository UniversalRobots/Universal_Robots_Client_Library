// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
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
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-09-11
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_client_library/rtde/data_package.h>

#include "rtde_test_helpers.h"

using namespace urcl;
using urcl::test::typedPackage;

TEST(rtde_data_package, serialize_pkg)
{
  std::vector<std::string> recipe{ "speed_slider_mask" };
  std::vector<std::string> types{ "UINT32" };
  auto package = typedPackage(recipe, types);

  uint32_t value = 1;
  package.setData("speed_slider_mask", value);

  uint8_t buffer[4096];
  package.setRecipeID(1);
  size_t size = package.serializePackage(buffer);

  EXPECT_EQ(size, 8);

  uint8_t expected[] = { 0x0, 0x08, 0x55, 0x01, 0x00, 0x00, 0x00, 0x01 };

  for (size_t i = 0; i < size; ++i)
  {
    EXPECT_EQ(buffer[i], expected[i]);
  }
}

TEST(rtde_data_package, parse_pkg_protocolv2)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };
  auto package = typedPackage(recipe, types);

  // Payload after the package header: recipe-id byte, then the fields.
  uint8_t data_package[] = { 0x01, 0x40, 0xd0, 0x75, 0x8c, 0x49, 0xba, 0x5e, 0x35, 0xbf, 0xf9, 0x9c, 0x77, 0xd1, 0x10,
                             0xb4, 0x60, 0xbf, 0xfb, 0xa2, 0x33, 0xd1, 0x10, 0xb4, 0x60, 0xc0, 0x01, 0x9f, 0xbe, 0x68,
                             0x88, 0x5a, 0x30, 0xbf, 0xe9, 0xdb, 0x22, 0xa2, 0x21, 0x68, 0xc0, 0x3f, 0xf9, 0x85, 0x87,
                             0xa0, 0x00, 0x00, 0x00, 0xbf, 0x9f, 0xbe, 0x74, 0x44, 0x2d, 0x18, 0x00 };

  comm::BinParser bp(data_package, sizeof(data_package));

  EXPECT_TRUE(package.parseWith(bp));

  vector6d_t expected_q = { -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031 };
  vector6d_t actual_q;
  package.getData("actual_q", actual_q);

  double abs = 1e-4;
  EXPECT_NEAR(expected_q[0], actual_q[0], abs);
  EXPECT_NEAR(expected_q[1], actual_q[1], abs);
  EXPECT_NEAR(expected_q[2], actual_q[2], abs);
  EXPECT_NEAR(expected_q[3], actual_q[3], abs);
  EXPECT_NEAR(expected_q[4], actual_q[4], abs);
  EXPECT_NEAR(expected_q[5], actual_q[5], abs);

  double expected_timestamp = 16854.1919;
  double actual_timestamp;
  package.getData("timestamp", actual_timestamp);

  EXPECT_NEAR(expected_timestamp, actual_timestamp, abs);
}

TEST(rtde_data_package, parse_pkg_protocolv1)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };
  auto package = typedPackage(recipe, types);
  package.setProtocolVersion(1);

  // Payload after the package header: fields only, no recipe-id.
  uint8_t data_package[] = { 0x40, 0xd0, 0x75, 0x8c, 0x49, 0xba, 0x5e, 0x35, 0xbf, 0xf9, 0x9c, 0x77, 0xd1, 0x10,
                             0xb4, 0x60, 0xbf, 0xfb, 0xa2, 0x33, 0xd1, 0x10, 0xb4, 0x60, 0xc0, 0x01, 0x9f, 0xbe,
                             0x68, 0x88, 0x5a, 0x30, 0xbf, 0xe9, 0xdb, 0x22, 0xa2, 0x21, 0x68, 0xc0, 0x3f, 0xf9,
                             0x85, 0x87, 0xa0, 0x00, 0x00, 0x00, 0xbf, 0x9f, 0xbe, 0x74, 0x44, 0x2d, 0x18, 0x00 };
  comm::BinParser bp(data_package, sizeof(data_package));

  EXPECT_TRUE(package.parseWith(bp));

  vector6d_t expected_q = { -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031 };
  vector6d_t actual_q;
  package.getData("actual_q", actual_q);

  double abs = 1e-4;
  EXPECT_NEAR(expected_q[0], actual_q[0], abs);
  EXPECT_NEAR(expected_q[1], actual_q[1], abs);
  EXPECT_NEAR(expected_q[2], actual_q[2], abs);
  EXPECT_NEAR(expected_q[3], actual_q[3], abs);
  EXPECT_NEAR(expected_q[4], actual_q[4], abs);
  EXPECT_NEAR(expected_q[5], actual_q[5], abs);

  double expected_timestamp = 16854.1919;
  double actual_timestamp;
  package.getData("timestamp", actual_timestamp);

  EXPECT_NEAR(expected_timestamp, actual_timestamp, abs);
}

TEST(rtde_data_package, serialize_pkg_protocolv1)
{
  std::vector<std::string> recipe{ "speed_slider_mask" };
  std::vector<std::string> types{ "UINT32" };
  auto package = typedPackage(recipe, types);

  uint32_t value = 1;
  package.setData("speed_slider_mask", value);

  uint8_t buffer[4096];
  package.setProtocolVersion(1);
  size_t size = package.serializePackage(buffer);

  EXPECT_EQ(size, 7);

  uint8_t expected[] = { 0x0, 0x07, 0x55, 0x00, 0x00, 0x00, 0x01 };

  for (size_t i = 0; i < size; ++i)
  {
    EXPECT_EQ(buffer[i], expected[i]);
  }
}

TEST(rtde_data_package, get_data_not_part_of_recipe)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };
  auto package = typedPackage(recipe, types);

  uint32_t speed_slider_mask;
  EXPECT_FALSE(package.getData("speed_slider_mask", speed_slider_mask));
}

TEST(rtde_data_package, set_data_not_part_of_recipe)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };
  auto package = typedPackage(recipe, types);

  uint32_t speed_slider_mask = 1;
  EXPECT_FALSE(package.setData("speed_slider_mask", speed_slider_mask));
}

TEST(rtde_data_package, parse_and_get_bitset_data)
{
  std::vector<std::string> recipe{ "robot_status_bits" };
  std::vector<std::string> types{ "UINT32" };
  auto package = typedPackage(recipe, types);

  uint8_t data_package[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x40, 0xb2, 0x3d, 0xa9, 0xfb, 0xe7, 0x6c, 0x8b };
  comm::BinParser bp(data_package, sizeof(data_package));

  EXPECT_TRUE(package.parseWith(bp));

  std::bitset<4> expected_robot_status_bits = 0000;
  std::bitset<4> actual_robot_status_bits;
  package.getData<uint32_t>("robot_status_bits", actual_robot_status_bits);

  EXPECT_EQ(expected_robot_status_bits, actual_robot_status_bits);
}

TEST(rtde_data_package, parse_incorrect_data_size)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  std::vector<std::string> types{ "DOUBLE", "VECTOR6D" };
  auto package = typedPackage(recipe, types);

  // Data package with incorrect size (should be 56 bytes for the given recipe)
  uint8_t data_package[] = { 0x01, 0x40, 0xd0, 0x75, 0x8c, 0x49, 0xba, 0x5e, 0x35, 0xbf };

  comm::BinParser bp(data_package, sizeof(data_package));

  EXPECT_THROW(package.parseWith(bp), UrException);
}

TEST(rtde_data_package, data_package_to_string)
{
  std::vector<std::string> recipe{ "speed_slider_mask", "speed_slider_fraction", "external_force_torque",
                                   "standard_digital_output_mask", "actual_digital_output_bits" };
  std::vector<std::string> types{ "UINT32", "DOUBLE", "VECTOR6D", "UINT8", "UINT64" };
  auto package = typedPackage(recipe, types);
  ASSERT_TRUE(package.setData<uint32_t>("speed_slider_mask", 1));
  ASSERT_TRUE(package.setData<double>("speed_slider_fraction", 0.5));
  ASSERT_TRUE(package.setData("external_force_torque", vector6d_t{ -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031 }));
  ASSERT_TRUE(package.setData<uint8_t>("standard_digital_output_mask", 1 << 7));
  ASSERT_TRUE(package.setData<uint64_t>("actual_digital_output_bits", 1 << 7));

  std::string pkg_str = package.toString();

  std::string expected_str = "speed_slider_mask: 1\n"
                             "speed_slider_fraction: 0.5\n"
                             "external_force_torque: [-1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031]\n"
                             "standard_digital_output_mask: 128\n"
                             "actual_digital_output_bits: 128\n";
  std::cout << "Package string:\n" << pkg_str << std::endl;
  EXPECT_EQ(expected_str, pkg_str);
}

TEST(rtde_data_package, every_rtde_data_type_can_be_applied)
{
  // The set of type names the robot may report is the only type knowledge the library still
  // carries, so check that each one maps onto the C++ type an application expects to read.
  std::vector<std::string> recipe{ "f_bool",   "f_uint8",    "f_uint32",   "f_uint64",  "f_int32",
                                   "f_double", "f_vector3d", "f_vector6d", "f_v6int32", "f_v6uint32" };
  std::vector<std::string> types{ "BOOL",   "UINT8",    "UINT32",   "UINT64",       "INT32",
                                  "DOUBLE", "VECTOR3D", "VECTOR6D", "VECTOR6INT32", "VECTOR6UINT32" };
  auto package = typedPackage(recipe, types);

  // Every field reports back the type the robot named for it
  for (size_t i = 0; i < recipe.size(); ++i)
  {
    const auto type = package.getDataType(recipe[i]);
    ASSERT_TRUE(type.has_value()) << "for field " << recipe[i];
    EXPECT_EQ(rtde_interface::toString(*type), types[i]) << "for field " << recipe[i];
  }

  bool bool_value;
  uint8_t uint8_value;
  uint32_t uint32_value;
  uint64_t uint64_value;
  int32_t int32_value;
  double double_value;
  vector3d_t vector3d_value;
  vector6d_t vector6d_value;
  vector6int32_t v6int32_value;
  vector6uint32_t v6uint32_value;

  EXPECT_TRUE(package.getData("f_bool", bool_value));
  EXPECT_TRUE(package.getData("f_uint8", uint8_value));
  EXPECT_TRUE(package.getData("f_uint32", uint32_value));
  EXPECT_TRUE(package.getData("f_uint64", uint64_value));
  EXPECT_TRUE(package.getData("f_int32", int32_value));
  EXPECT_TRUE(package.getData("f_double", double_value));
  EXPECT_TRUE(package.getData("f_vector3d", vector3d_value));
  EXPECT_TRUE(package.getData("f_vector6d", vector6d_value));
  EXPECT_TRUE(package.getData("f_v6int32", v6int32_value));
  EXPECT_TRUE(package.getData("f_v6uint32", v6uint32_value));

  // Each field holds exactly the type the robot named for it, and nothing else
  EXPECT_FALSE(package.getData("f_bool", double_value));
  EXPECT_FALSE(package.getData("f_uint8", uint32_value));
  EXPECT_FALSE(package.getData("f_uint32", int32_value));
  EXPECT_FALSE(package.getData("f_uint64", uint32_value));
  EXPECT_FALSE(package.getData("f_int32", uint32_value));
  EXPECT_FALSE(package.getData("f_double", uint64_value));
  EXPECT_FALSE(package.getData("f_vector3d", vector6d_value));
  EXPECT_FALSE(package.getData("f_vector6d", vector3d_value));
  EXPECT_FALSE(package.getData("f_v6int32", v6uint32_value));
  EXPECT_FALSE(package.getData("f_v6uint32", v6int32_value));
}

// The wire format of the rarer data types is otherwise only exercised against a real robot, so a
// serializer or parser that got one of them wrong would pass every other test here. Values are
// chosen to be asymmetric, so a byte-order mistake cannot round-trip by accident.
TEST(rtde_data_package, every_rtde_data_type_survives_a_serialize_parse_round_trip)
{
  const std::vector<std::string> recipe{ "f_bool",   "f_uint8",    "f_uint32",   "f_uint64",  "f_int32",
                                         "f_double", "f_vector3d", "f_vector6d", "f_v6int32", "f_v6uint32" };
  const std::vector<std::string> types{ "BOOL",   "UINT8",    "UINT32",   "UINT64",       "INT32",
                                        "DOUBLE", "VECTOR3D", "VECTOR6D", "VECTOR6INT32", "VECTOR6UINT32" };

  const bool bool_value = true;
  const uint8_t uint8_value = 0xa5;
  const uint32_t uint32_value = 0x12345678;
  const uint64_t uint64_value = 0x0123456789abcdef;
  const int32_t int32_value = -123456789;
  const double double_value = -1234.5678;
  const vector3d_t vector3d_value{ 1.5, -2.5, 3.5 };
  const vector6d_t vector6d_value{ -1.6007, -1.7271, -2.203, -0.808, 1.5951, -0.031 };
  const vector6int32_t v6int32_value{ -1, 2, -3, 4, -5, 6 };
  const vector6uint32_t v6uint32_value{ 1u, 2u, 3u, 4u, 5u, 0xffffffffu };

  auto sent = typedPackage(recipe, types);
  sent.setRecipeID(1);
  ASSERT_TRUE(sent.setData("f_bool", bool_value));
  ASSERT_TRUE(sent.setData("f_uint8", uint8_value));
  ASSERT_TRUE(sent.setData("f_uint32", uint32_value));
  ASSERT_TRUE(sent.setData("f_uint64", uint64_value));
  ASSERT_TRUE(sent.setData("f_int32", int32_value));
  ASSERT_TRUE(sent.setData("f_double", double_value));
  ASSERT_TRUE(sent.setData("f_vector3d", vector3d_value));
  ASSERT_TRUE(sent.setData("f_vector6d", vector6d_value));
  ASSERT_TRUE(sent.setData("f_v6int32", v6int32_value));
  ASSERT_TRUE(sent.setData("f_v6uint32", v6uint32_value));

  uint8_t buffer[4096];
  const size_t size = sent.serializePackage(buffer);

  // A two byte size and a one byte package type, then the recipe id and one entry per field
  const size_t header_size = 3;
  const size_t expected_payload = sizeof(uint8_t) + sizeof(bool) + sizeof(uint8_t) + sizeof(uint32_t) +
                                  sizeof(uint64_t) + sizeof(int32_t) + sizeof(double) + sizeof(vector3d_t) +
                                  sizeof(vector6d_t) + sizeof(vector6int32_t) + sizeof(vector6uint32_t);
  EXPECT_EQ(size, header_size + expected_payload);

  // Round-tripping on its own would still pass if both directions agreed on the wrong byte order,
  // so pin the integers to the network order the protocol uses. -123456789 is 0xf8a432eb.
  const uint8_t expected_integers[] = { 0x01,                                            // recipe id
                                        0x01,                                            // f_bool
                                        0xa5,                                            // f_uint8
                                        0x12, 0x34, 0x56, 0x78,                          // f_uint32
                                        0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,  // f_uint64
                                        0xf8, 0xa4, 0x32, 0xeb };                        // f_int32
  for (size_t i = 0; i < sizeof(expected_integers); ++i)
  {
    EXPECT_EQ(buffer[header_size + i], expected_integers[i]) << "at payload byte " << i;
  }

  // serializePackage() writes the v2 recipe-id after the header; parseWith() consumes it too.
  comm::BinParser bp(buffer + header_size, size - header_size);
  auto received = typedPackage(recipe, types);
  ASSERT_TRUE(received.parseWith(bp));
  EXPECT_TRUE(bp.empty()) << "the parser did not consume exactly what was serialized";

  bool bool_read;
  uint8_t uint8_read;
  uint32_t uint32_read;
  uint64_t uint64_read;
  int32_t int32_read;
  double double_read;
  vector3d_t vector3d_read;
  vector6d_t vector6d_read;
  vector6int32_t v6int32_read;
  vector6uint32_t v6uint32_read;

  ASSERT_TRUE(received.getData("f_bool", bool_read));
  ASSERT_TRUE(received.getData("f_uint8", uint8_read));
  ASSERT_TRUE(received.getData("f_uint32", uint32_read));
  ASSERT_TRUE(received.getData("f_uint64", uint64_read));
  ASSERT_TRUE(received.getData("f_int32", int32_read));
  ASSERT_TRUE(received.getData("f_double", double_read));
  ASSERT_TRUE(received.getData("f_vector3d", vector3d_read));
  ASSERT_TRUE(received.getData("f_vector6d", vector6d_read));
  ASSERT_TRUE(received.getData("f_v6int32", v6int32_read));
  ASSERT_TRUE(received.getData("f_v6uint32", v6uint32_read));

  EXPECT_EQ(bool_read, bool_value);
  EXPECT_EQ(uint8_read, uint8_value);
  EXPECT_EQ(uint32_read, uint32_value);
  EXPECT_EQ(uint64_read, uint64_value);
  EXPECT_EQ(int32_read, int32_value);
  EXPECT_EQ(double_read, double_value);
  EXPECT_EQ(vector3d_read, vector3d_value);
  EXPECT_EQ(vector6d_read, vector6d_value);
  EXPECT_EQ(v6int32_read, v6int32_value);
  EXPECT_EQ(v6uint32_read, v6uint32_value);
}

TEST(rtde_data_package, unknown_data_types_are_rejected)
{
  std::vector<std::string> recipe{ "timestamp" };
  rtde_interface::DataPackage package(recipe);

  // A field the robot doesn't know about is reported as NOT_FOUND, one that is already used by
  // another recipe as IN_USE. Neither is a data type.
  EXPECT_THROW(package.setTypes({ "NOT_FOUND" }), UrException);
  EXPECT_THROW(package.setTypes({ "IN_USE" }), UrException);
  EXPECT_THROW(package.setTypes({ "double" }), UrException);
}

TEST(rtde_data_package, failed_set_types_leaves_the_package_unchanged)
{
  auto package = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));
  const uint64_t layout = package.layoutHash();

  EXPECT_THROW(package.setTypes({ "UINT64", "NOT_A_TYPE" }), UrException);

  EXPECT_EQ(package.layoutHash(), layout);
  EXPECT_EQ(package.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  EXPECT_EQ(package.getDataType("actual_q"), rtde_interface::DataType::VECTOR6D);
  double timestamp = 0.0;
  ASSERT_TRUE(package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);

  auto other = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(other.setData("timestamp", 1.0));
  ASSERT_TRUE(package.copyFrom(other));
  ASSERT_TRUE(package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 1.0);
}

TEST(rtde_data_package, type_count_has_to_match_recipe)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage package(recipe);
  EXPECT_THROW(package.setTypes({ "DOUBLE" }), UrException);
  EXPECT_THROW(package.setTypes({ "DOUBLE", "VECTOR6D", "DOUBLE" }), UrException);
}

TEST(rtde_data_package, untyped_package_cannot_be_parsed_or_serialized)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage package(recipe);

  EXPECT_FALSE(package.getDataType("timestamp").has_value());

  double timestamp = 0.0;
  EXPECT_FALSE(package.getData("timestamp", timestamp));

  uint8_t buffer[4096];
  EXPECT_EQ(package.serializePackage(buffer), 0);

  uint8_t data_package[] = { 0x01, 0x40, 0xd0, 0x75, 0x8c, 0x49, 0xba, 0x5e, 0x35 };
  comm::BinParser bp(data_package, sizeof(data_package));
  EXPECT_FALSE(package.parseWith(bp));
}

TEST(rtde_data_package, untyped_package_gets_typed_by_assignment)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage untyped_package(recipe);
  auto typed_package = typedPackage(recipe, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(typed_package.setData("timestamp", 42.0));

  untyped_package = typed_package;

  EXPECT_EQ(untyped_package.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  double timestamp = 0.0;
  ASSERT_TRUE(untyped_package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

// Applying the robot's answer to a package an application is already holding is what lets that
// application allocate the package wherever it likes, including before the connection exists.
TEST(rtde_data_package, applying_types_makes_the_package_usable)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage package(recipe);

  double timestamp = 0.0;
  ASSERT_FALSE(package.getData("timestamp", timestamp));

  package.setTypes({ "DOUBLE", "VECTOR6D" });

  EXPECT_EQ(package.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  ASSERT_TRUE(package.setData("timestamp", 42.0));
  ASSERT_TRUE(package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

// An input package is written before the recipe has been acknowledged, so setData() has to be able
// to decide a field's type itself. Whether it matches the robot is checked when the package is sent.
TEST(rtde_data_package, set_data_establishes_the_type_of_an_untyped_field)
{
  rtde_interface::DataPackage package({ "speed_slider_mask", "speed_slider_fraction" });

  ASSERT_TRUE(package.setData("speed_slider_fraction", 0.5));

  double speed_slider_fraction = 0.0;
  ASSERT_TRUE(package.getData("speed_slider_fraction", speed_slider_fraction));
  EXPECT_DOUBLE_EQ(speed_slider_fraction, 0.5);

  EXPECT_EQ(package.getDataType("speed_slider_fraction"), rtde_interface::DataType::DOUBLE);

  // The field that was never written keeps no type at all
  uint32_t speed_slider_mask = 1;
  EXPECT_FALSE(package.getData("speed_slider_mask", speed_slider_mask));
  EXPECT_FALSE(package.getDataType("speed_slider_mask").has_value());
}

TEST(rtde_data_package, get_data_type_reports_unknown_fields_and_untyped_fields)
{
  auto package = typedPackage({ "timestamp" }, { "DOUBLE" });

  EXPECT_FALSE(package.getDataType("not_in_the_recipe").has_value());

  // Asking about a field of a package the robot hasn't acknowledged yet is the other way to get a
  // negative answer, and it is what tells an application the package isn't usable yet.
  rtde_interface::DataPackage untyped_package({ "timestamp" });
  EXPECT_FALSE(untyped_package.getDataType("timestamp").has_value());
}

// Once a field has a type, whether from the robot or from an earlier write, a differently typed
// write is a mistake rather than a retype.
TEST(rtde_data_package, set_data_checks_against_an_established_type)
{
  rtde_interface::DataPackage package({ "speed_slider_fraction" });
  ASSERT_TRUE(package.setData("speed_slider_fraction", 0.5));

  EXPECT_FALSE(package.setData("speed_slider_fraction", static_cast<uint32_t>(1)));

  auto typed_package = typedPackage({ "timestamp" }, { "DOUBLE" });
  EXPECT_FALSE(typed_package.setData("timestamp", static_cast<uint32_t>(1)));
}

// Whether a package knows its types is answered by the fields themselves rather than by a flag
// recording that the robot answered, so writing every field of an untyped package is enough to make
// it serializable. Values written this way are still checked against the robot when the package is
// handed to RTDEWriter::sendPackage().
TEST(rtde_data_package, writing_every_field_makes_a_package_serializable)
{
  rtde_interface::DataPackage package({ "speed_slider_mask", "speed_slider_fraction" });
  package.setRecipeID(1);

  uint8_t buffer[4096];
  ASSERT_EQ(package.serializePackage(buffer), 0) << "no field has a type yet";

  ASSERT_TRUE(package.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  EXPECT_EQ(package.serializePackage(buffer), 0) << "speed_slider_fraction still has no type";

  ASSERT_TRUE(package.setData("speed_slider_fraction", 0.5));
  // A two byte size, a one byte package type and the one byte recipe id, then the two fields
  const size_t header_size = 4;
  EXPECT_EQ(package.serializePackage(buffer), header_size + sizeof(uint32_t) + sizeof(double));
}

// Overwriting the send buffer with a complete package belongs to RTDEWriter, so it is covered
// by the sendPackage() tests in test_rtde_writer.cpp.

// Zeroing a package has to keep the types intact, otherwise the next serialization would use the
// wrong field sizes.
TEST(rtde_data_package, init_empty_keeps_types)
{
  auto package = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));

  package.initEmpty();

  EXPECT_EQ(package.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  double timestamp = 1.0;
  ASSERT_TRUE(package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 0.0);
}

// emptyCopy() is the layout of this package with every value taken from zeros_, so writing here
// must not leak into the copy and the copy must keep the same hashes.
TEST(rtde_data_package, empty_copy_keeps_the_layout_and_zeroes_the_values)
{
  auto package = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));
  const uint64_t recipe = package.recipeHash();
  const uint64_t layout = package.layoutHash();

  const rtde_interface::DataPackage copy = package.emptyCopy();

  EXPECT_TRUE(copy.isTyped());
  EXPECT_EQ(copy.recipeHash(), recipe);
  EXPECT_EQ(copy.layoutHash(), layout);
  EXPECT_EQ(copy.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  EXPECT_EQ(copy.getDataType("actual_q"), rtde_interface::DataType::VECTOR6D);
  double timestamp = 1.0;
  ASSERT_TRUE(copy.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 0.0);
  vector6d_t actual_q{ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
  ASSERT_TRUE(copy.getData("actual_q", actual_q));
  EXPECT_EQ(actual_q, vector6d_t({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }));

  timestamp = 0.0;
  ASSERT_TRUE(package.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

TEST(rtde_data_package, empty_copy_of_an_untyped_package_is_untyped)
{
  rtde_interface::DataPackage package({ "timestamp", "actual_q" });

  const rtde_interface::DataPackage copy = package.emptyCopy();

  EXPECT_FALSE(package.isTyped());
  EXPECT_FALSE(copy.isTyped());
}

TEST(rtde_data_package, copy_keeps_types_and_values)
{
  auto package = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));

  rtde_interface::DataPackage copy(package);

  EXPECT_EQ(copy.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
  double timestamp = 0.0;
  ASSERT_TRUE(copy.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

TEST(rtde_data_package, get_data_with_wrong_type_fails)
{
  auto package = typedPackage({ "timestamp" }, { "DOUBLE" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));

  // The robot dictates the types, so asking for the wrong one has to fail gracefully instead of
  // throwing std::bad_variant_access.
  uint32_t timestamp = 0;
  EXPECT_FALSE(package.getData("timestamp", timestamp));
}

TEST(rtde_data_package, layout_hash_changes_when_types_are_set)
{
  rtde_interface::DataPackage package({ "timestamp", "actual_q" });
  const uint64_t untyped = package.layoutHash();
  const uint64_t recipe = package.recipeHash();

  package.setTypes({ "DOUBLE", "VECTOR6D" });

  EXPECT_EQ(package.recipeHash(), recipe);
  EXPECT_NE(package.layoutHash(), untyped);
}

TEST(rtde_data_package, layout_hash_changes_on_first_set_data_to_an_untyped_field)
{
  rtde_interface::DataPackage package({ "timestamp", "actual_q" });
  const uint64_t untyped = package.layoutHash();

  ASSERT_TRUE(package.setData("timestamp", 1.0));
  const uint64_t after_first = package.layoutHash();
  EXPECT_NE(after_first, untyped);

  ASSERT_TRUE(package.setData("timestamp", 2.0));
  EXPECT_EQ(package.layoutHash(), after_first);
}

TEST(rtde_data_package, layout_hash_does_not_change_on_reset_init_empty_or_parse)
{
  auto package = typedPackage({ "timestamp", "target_speed_fraction" }, { "DOUBLE", "DOUBLE" });
  ASSERT_TRUE(package.setData("timestamp", 42.0));
  const uint64_t hash = package.layoutHash();

  ASSERT_TRUE(package.resetData("timestamp"));
  EXPECT_EQ(package.layoutHash(), hash);

  package.initEmpty();
  EXPECT_EQ(package.layoutHash(), hash);

  uint8_t data[] = { 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a, 0x9f, 0xbe,
                     0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(data, sizeof(data));
  ASSERT_TRUE(package.parseWith(bp));
  EXPECT_EQ(package.layoutHash(), hash);
}

TEST(rtde_data_package, copy_from_overwrites_every_field)
{
  auto destination = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  auto source = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(source.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.5));
  ASSERT_TRUE(destination.copyFrom(source));

  double fraction = 0.0;
  uint32_t mask = 0;
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  ASSERT_TRUE(destination.getData("speed_slider_mask", mask));
  EXPECT_DOUBLE_EQ(fraction, 0.5);
  EXPECT_EQ(mask, 1u);

  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.7));
  ASSERT_TRUE(source.setData("speed_slider_mask", static_cast<uint32_t>(0)));
  ASSERT_TRUE(destination.copyFrom(source));
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  ASSERT_TRUE(destination.getData("speed_slider_mask", mask));
  EXPECT_DOUBLE_EQ(fraction, 0.7);
  EXPECT_EQ(mask, 0u);
}

TEST(rtde_data_package, copy_from_rejects_a_source_whose_types_changed)
{
  auto destination = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  auto source = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(source.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.5));
  ASSERT_TRUE(destination.copyFrom(source));

  auto wrong = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT8", "DOUBLE" });
  ASSERT_TRUE(wrong.setData("speed_slider_mask", static_cast<uint8_t>(1)));
  ASSERT_TRUE(wrong.setData("speed_slider_fraction", 0.9));
  EXPECT_FALSE(destination.copyFrom(wrong));

  double fraction = 0.0;
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  EXPECT_DOUBLE_EQ(fraction, 0.5);
}

// An application may write only the fields it cares about, which leaves the rest of its package
// untyped. Those fields are taken over as zeros rather than making the copy fail.
TEST(rtde_data_package, copy_from_zeros_the_fields_the_source_did_not_write)
{
  auto destination = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(destination.setData("speed_slider_mask", static_cast<uint32_t>(7)));

  rtde_interface::DataPackage source({ "speed_slider_mask", "speed_slider_fraction" });
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.5));

  ASSERT_TRUE(destination.copyFrom(source));

  double fraction = 0.0;
  uint32_t mask = 0;
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  ASSERT_TRUE(destination.getData("speed_slider_mask", mask));
  EXPECT_DOUBLE_EQ(fraction, 0.5);
  EXPECT_EQ(mask, 0u);
}

TEST(rtde_data_package, copy_from_rejects_when_the_destination_is_retyped)
{
  auto destination = typedPackage({ "timestamp" }, { "DOUBLE" });
  auto source = typedPackage({ "timestamp" }, { "DOUBLE" });
  ASSERT_TRUE(source.setData("timestamp", 1.0));
  ASSERT_TRUE(destination.copyFrom(source));

  destination.setTypes({ "UINT32" });
  EXPECT_FALSE(destination.copyFrom(source));
}

TEST(rtde_data_package, failed_copy_from_does_not_overwrite)
{
  auto destination = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(destination.setData("speed_slider_fraction", 0.5));

  auto wrong = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT8", "DOUBLE" });
  ASSERT_TRUE(wrong.setData("speed_slider_fraction", 0.9));
  EXPECT_FALSE(destination.copyFrom(wrong));

  auto source = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.25));
  ASSERT_TRUE(destination.copyFrom(source));

  double fraction = 0.0;
  ASSERT_TRUE(destination.getData("speed_slider_fraction", fraction));
  EXPECT_DOUBLE_EQ(fraction, 0.25);
}

TEST(rtde_data_package, copy_from_a_different_recipe_fails_after_a_successful_copy)
{
  auto destination = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  auto source = typedPackage({ "speed_slider_mask", "speed_slider_fraction" }, { "UINT32", "DOUBLE" });
  ASSERT_TRUE(source.setData("speed_slider_fraction", 0.5));
  ASSERT_TRUE(destination.copyFrom(source));

  rtde_interface::DataPackage other({ "standard_analog_output_0" });
  ASSERT_TRUE(other.setData("standard_analog_output_0", 0.1));
  EXPECT_FALSE(destination.copyFrom(other));
}

TEST(rtde_data_package, same_recipe_assignment_keeps_name_lookup)
{
  auto source = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  ASSERT_TRUE(source.setData("timestamp", 42.0));

  auto destination = typedPackage({ "timestamp", "actual_q" }, { "DOUBLE", "VECTOR6D" });
  destination = source;

  double timestamp = 0.0;
  ASSERT_TRUE(destination.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
  EXPECT_EQ(destination.getDataType("timestamp"), rtde_interface::DataType::DOUBLE);
}

TEST(rtde_data_package, assignment_from_a_different_recipe_rebuilds_name_lookup)
{
  auto source = typedPackage({ "actual_q" }, { "VECTOR6D" });
  ASSERT_TRUE(source.setData("actual_q", vector6d_t{ 1, 2, 3, 4, 5, 6 }));

  auto destination = typedPackage({ "timestamp" }, { "DOUBLE" });
  destination = source;

  vector6d_t actual_q{};
  ASSERT_TRUE(destination.getData("actual_q", actual_q));
  EXPECT_DOUBLE_EQ(actual_q[0], 1.0);
  EXPECT_FALSE(destination.getDataType("timestamp").has_value());
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
