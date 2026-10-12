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
 * \date    2020-07-09
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>
#include <cstring>
#include <initializer_list>
#include <utility>

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/comm/package_serializer.h>
#include <ur_client_library/rtde/read_properties.h>
#include <ur_client_library/rtde/rtde_parser.h>
#include <ur_client_library/ur/version_information.h>

#include "rtde_test_helpers.h"

using namespace urcl;
using urcl::rtde_interface::DataType;

TEST(rtde_parser, request_protocol_version)
{
  // Accepted request protocol version
  unsigned char raw_data[] = { 0x00, 0x04, 0x56, 0x01 };
  rtde_interface::RTDEParser parser({ "" });

  // test a non-preallocated product
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  {
    comm::BinParser bp(raw_data, sizeof(raw_data));
    parser.parse(bp, product);
  }

  if (rtde_interface::RequestProtocolVersion* data =
          dynamic_cast<rtde_interface::RequestProtocolVersion*>(product.get()))
  {
    EXPECT_EQ(data->accepted_, true);
  }
  else
  {
    std::cout << "Failed to get request protocol version data" << std::endl;
    GTEST_FAIL();
  }

  // test a preallocated product
  std::unique_ptr<rtde_interface::RTDEPackage> product2 = std::make_unique<rtde_interface::RequestProtocolVersion>();
  {
    comm::BinParser bp(raw_data, sizeof(raw_data));
    parser.parse(bp, product2);
  }
  if (rtde_interface::RequestProtocolVersion* data =
          dynamic_cast<rtde_interface::RequestProtocolVersion*>(product2.get()))
  {
    EXPECT_EQ(data->accepted_, true);
  }
  else
  {
    std::cout << "Failed to get request protocol version data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, get_urcontrol_version)
{
  // URControl version 5.8.0-0
  unsigned char raw_data[] = { 0x00, 0x13, 0x76, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                               0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, product);

  if (rtde_interface::GetUrcontrolVersion* data = dynamic_cast<rtde_interface::GetUrcontrolVersion*>(product.get()))
  {
    EXPECT_EQ(data->version_information_.major, 5);
    EXPECT_EQ(data->version_information_.minor, 8);
    EXPECT_EQ(data->version_information_.bugfix, 0);
    EXPECT_EQ(data->version_information_.build, 0);
  }
  else
  {
    std::cout << "Failed to get urcontrol version data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, control_package_pause)
{
  // Accepted control package pause
  unsigned char raw_data[] = { 0x00, 0x04, 0x50, 0x01 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, product);

  if (rtde_interface::ControlPackagePause* data = dynamic_cast<rtde_interface::ControlPackagePause*>(product.get()))
  {
    EXPECT_EQ(data->accepted_, true);
  }
  else
  {
    std::cout << "Failed to get control package pause data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, control_package_start)
{
  // Accepted control package start
  unsigned char raw_data[] = { 0x00, 0x04, 0x53, 0x01 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, product);

  if (rtde_interface::ControlPackageStart* data = dynamic_cast<rtde_interface::ControlPackageStart*>(product.get()))
  {
    EXPECT_EQ(data->accepted_, true);
  }
  else
  {
    std::cout << "Failed to get control package start data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, control_package_setup_inputs)
{
  // Accepted control package setup inputs, variable types are uint32 and double
  unsigned char raw_data[] = { 0x00, 0x11, 0x49, 0x01, 0x55, 0x49, 0x4e, 0x54, 0x33,
                               0x32, 0x2c, 0x44, 0x4f, 0x55, 0x42, 0x4c, 0x45 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, product);

  if (rtde_interface::ControlPackageSetupInputs* data =
          dynamic_cast<rtde_interface::ControlPackageSetupInputs*>(product.get()))
  {
    EXPECT_EQ(data->input_recipe_id_, 1);
    EXPECT_EQ(data->variable_types_, "UINT32,DOUBLE");
  }
  else
  {
    std::cout << "Failed to get control package setup inputs data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, control_package_setup_outputs)
{
  // Accepted control package setup outputs, variable types are double and vector6d
  unsigned char raw_data[] = { 0x00, 0x11, 0x4f, 0x01, 0x44, 0x4f, 0x55, 0x42, 0x4c, 0x45,
                               0x2c, 0x56, 0x45, 0x43, 0x54, 0x4f, 0x52, 0x36, 0x44 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.setProtocolVersion(2);
  parser.parse(bp, product);

  if (rtde_interface::ControlPackageSetupOutputs* data =
          dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(product.get()))
  {
    EXPECT_EQ(data->output_recipe_id_, 1);
    EXPECT_EQ(data->variable_types_, "DOUBLE,VECTOR6D");
  }
  else
  {
    std::cout << "Failed to get control package setup outputs data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, data_package)
{
  // received data package,
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setExpectedDataPackage(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }));
  parser.setProtocolVersion(2);
  parser.parse(bp, product);

  if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(product.get()))
  {
    double timestamp, target_speed_fraction;
    data->getData("timestamp", timestamp);
    data->getData("target_speed_fraction", target_speed_fraction);

    EXPECT_DOUBLE_EQ(timestamp, 16412.206);
    EXPECT_EQ(target_speed_fraction, 1);
  }
  else
  {
    std::cout << "Failed to get data package data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, data_package_without_recipe_types_fails)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  // Without the types from the robot's acknowledgement the payload cannot be interpreted
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "timestamp", "target_speed_fraction" });
  parser.setProtocolVersion(2);

  EXPECT_FALSE(parser.parse(bp, product));
}

// DataPackage types are owned by the client and must be applied before parsing.
TEST(rtde_parser, untyped_pre_allocated_data_package_is_rejected)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }).layoutHash());

  std::unique_ptr<rtde_interface::RTDEPackage> product = std::make_unique<rtde_interface::DataPackage>(recipe);
  const rtde_interface::RTDEPackage* package_address = product.get();

  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), package_address);
}

// A package with a different typed layout is rejected before payload parsing.
TEST(rtde_parser, wrongly_typed_pre_allocated_package_is_rejected)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }).layoutHash());

  auto package = std::make_unique<rtde_interface::DataPackage>(recipe);
  ASSERT_TRUE(package->setData("timestamp", static_cast<uint64_t>(1)));
  ASSERT_TRUE(package->setData("target_speed_fraction", static_cast<uint64_t>(2)));

  std::unique_ptr<rtde_interface::RTDEPackage> product = std::move(package);
  const rtde_interface::RTDEPackage* package_address = product.get();

  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), package_address);
}

TEST(rtde_parser, pre_allocated_package_with_a_different_recipe_is_rejected)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }).layoutHash());

  std::unique_ptr<rtde_interface::RTDEPackage> product =
      std::make_unique<rtde_interface::DataPackage>(std::vector<std::string>{ "foo", "bar" });
  const rtde_interface::RTDEPackage* package_address = product.get();

  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), package_address);
}

TEST(rtde_parser, typed_pre_allocated_data_package_takes_protocol_version_1)
{
  // Same payload as data_package, but without the recipe-id byte that only version 2 uses.
  unsigned char raw_data[] = { 0x00, 0x13, 0x55, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a, 0x9f,
                               0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  auto expected_package = test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE });
  expected_package.setProtocolVersion(1);
  parser.setExpectedLayoutHash(expected_package.layoutHash());
  parser.setProtocolVersion(1);

  auto package = std::make_unique<rtde_interface::DataPackage>(recipe);
  package->setTypes({ DataType::DOUBLE, DataType::DOUBLE });
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::move(package);

  ASSERT_TRUE(parser.parse(bp, product));

  rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
  ASSERT_NE(data, nullptr);
  double timestamp = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 16412.206);
}

TEST(rtde_parser, test_to_string)
{
  // Non-existent type
  unsigned char raw_data[] = { 0x00, 0x05, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, product);

  std::stringstream expected;
  expected << "Type: 2" << std::endl;
  expected << "Raw byte stream: 0 0 " << std::endl;

  EXPECT_EQ(product->toString(), expected.str());
}

TEST(rtde_parser, test_buffer_too_short)
{
  // Non-existent type with false size information
  unsigned char raw_data[] = { 0x00, 0x06, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  EXPECT_FALSE(parser.parse(bp, product));
}

TEST(rtde_parser, test_buffer_too_long)
{
  // Non-existent type with false size information
  unsigned char raw_data[] = { 0x00, 0x04, 0x56, 0x01, 0x02, 0x01, 0x02 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  rtde_interface::RTDEParser parser({ "" });
  EXPECT_FALSE(parser.parse(bp, product));
}

// The single-pointer parse consumes one package. Two concatenated data packages therefore leave
// leftover bytes, which the parser reports as a failure rather than silently dropping the second.
TEST(rtde_parser, two_data_packages_in_one_buffer_leave_leftover_bytes)
{
  unsigned char first[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                            0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char second[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xc3, 0x88, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char raw_data[sizeof(first) + sizeof(second)];
  std::memcpy(raw_data, first, sizeof(first));
  std::memcpy(raw_data + sizeof(first), second, sizeof(second));
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setExpectedDataPackage(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }));
  parser.setProtocolVersion(2);

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  EXPECT_FALSE(parser.parse(bp, product));
}

TEST(rtde_parser, test_deprecated_parse_method)
{
  // received data package,
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setExpectedDataPackage(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }));
  parser.setProtocolVersion(2);

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  {
    comm::BinParser bp(raw_data, sizeof(raw_data));
    URCL_SILENCE_DEPRECATED_BEGIN
    ASSERT_TRUE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
  }

  ASSERT_EQ(products.size(), 1);

  if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(products[0].get()))
  {
    double timestamp, target_speed_fraction;
    data->getData("timestamp", timestamp);
    data->getData("target_speed_fraction", target_speed_fraction);

    EXPECT_DOUBLE_EQ(timestamp, 16412.206);
    EXPECT_EQ(target_speed_fraction, 1);
  }
  else
  {
    std::cout << "Failed to get data package data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, deprecated_parse_without_registration_rejects_typed_package)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto package = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  package.setProtocolVersion(2);
  ASSERT_TRUE(package.setData("timestamp", 42.0));
  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  products.push_back(std::make_unique<rtde_interface::DataPackage>(package));
  const auto* original = products.back().get();

  comm::BinParser bp(raw_data, sizeof(raw_data));
  URCL_SILENCE_DEPRECATED_BEGIN
  EXPECT_FALSE(parser.parse(bp, products));
  URCL_SILENCE_DEPRECATED_END
  ASSERT_EQ(products.size(), 1u);
  EXPECT_EQ(products.back().get(), original);
  auto* data = dynamic_cast<rtde_interface::DataPackage*>(products.back().get());
  ASSERT_NE(data, nullptr);
  double timestamp = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

TEST(rtde_parser, deprecated_hash_only_parse_rejects_empty_vector)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  expected.setProtocolVersion(2);
  parser.setExpectedLayoutHash(expected.layoutHash());
  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;

  comm::BinParser bp(raw_data, sizeof(raw_data));
  URCL_SILENCE_DEPRECATED_BEGIN
  EXPECT_FALSE(parser.parse(bp, products));
  URCL_SILENCE_DEPRECATED_END
  EXPECT_TRUE(products.empty());
}

TEST(rtde_parser, deprecated_hash_only_parse_rejects_null_or_non_data_last_entry)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  for (const bool null_last_entry : { false, true })
  {
    SCOPED_TRACE(null_last_entry);
    rtde_interface::RTDEParser parser({ "timestamp" });
    parser.setProtocolVersion(2);
    auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
    expected.setProtocolVersion(2);
    parser.setExpectedLayoutHash(expected.layoutHash());
    ASSERT_TRUE(expected.setData("timestamp", 42.0));
    std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
    // A matching earlier entry must not be used in place of an invalid last entry.
    products.push_back(std::make_unique<rtde_interface::DataPackage>(expected));
    const auto* first = products.front().get();
    if (null_last_entry)
    {
      products.push_back(nullptr);
    }
    else
    {
      auto control = std::make_unique<rtde_interface::ControlPackageStart>();
      control->accepted_ = true;
      products.push_back(std::move(control));
    }
    const auto* last = products.back().get();

    comm::BinParser bp(raw_data, sizeof(raw_data));
    URCL_SILENCE_DEPRECATED_BEGIN
    EXPECT_FALSE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    ASSERT_EQ(products.size(), 2u);
    EXPECT_EQ(products.front().get(), first);
    EXPECT_EQ(products.back().get(), last);
    if (!null_last_entry)
    {
      auto* start = dynamic_cast<rtde_interface::ControlPackageStart*>(products.back().get());
      ASSERT_NE(start, nullptr);
      EXPECT_TRUE(start->accepted_);
    }
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(products.front().get());
    ASSERT_NE(data, nullptr);
    double timestamp = 0.0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_DOUBLE_EQ(timestamp, 42.0);
  }
}

TEST(rtde_parser, deprecated_hash_only_parse_reuses_last_package_repeatedly)
{
  // Complete v2 packets: (timestamp, target_speed_fraction) = (1, 0.5), then (2, 1).
  unsigned char packets[][20] = { { 0x00, 0x14, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
                                  { 0x00, 0x14, 0x55, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } };
  const std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  auto expected = test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE });
  expected.setProtocolVersion(2);
  parser.setExpectedLayoutHash(expected.layoutHash());
  ASSERT_TRUE(expected.setData("timestamp", 42.0));
  ASSERT_TRUE(expected.setData("target_speed_fraction", 0.25));
  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  products.push_back(std::make_unique<rtde_interface::DataPackage>(expected));
  products.push_back(std::make_unique<rtde_interface::DataPackage>(expected));
  const auto* first = products.front().get();
  const auto* last = products.back().get();

  for (size_t i = 0; i < 2; ++i)
  {
    SCOPED_TRACE(i);
    comm::BinParser bp(packets[i], sizeof(packets[i]));
    URCL_SILENCE_DEPRECATED_BEGIN
    ASSERT_TRUE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    EXPECT_TRUE(bp.empty());
    ASSERT_EQ(products.size(), 2u);
    EXPECT_EQ(products.front().get(), first);
    EXPECT_EQ(products.back().get(), last);
    for (size_t j = 0; j < products.size(); ++j)
    {
      auto* data = dynamic_cast<rtde_interface::DataPackage*>(products[j].get());
      ASSERT_NE(data, nullptr);
      EXPECT_EQ(data->layoutHash(), expected.layoutHash());
      double timestamp = 0.0;
      double target_speed_fraction = 0.0;
      ASSERT_TRUE(data->getData("timestamp", timestamp));
      ASSERT_TRUE(data->getData("target_speed_fraction", target_speed_fraction));
      EXPECT_DOUBLE_EQ(timestamp, j == 0 ? 42.0 : (i == 0 ? 1.0 : 2.0));
      EXPECT_DOUBLE_EQ(target_speed_fraction, j == 0 ? 0.25 : (i == 0 ? 0.5 : 1.0));
    }
  }
}

TEST(rtde_parser, deprecated_hash_only_parse_repairs_protocol_only_mismatch)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  for (const uint16_t version : std::initializer_list<uint16_t>{ 1, 2 })
  {
    SCOPED_TRACE(version);
    rtde_interface::RTDEParser parser({ "timestamp" });
    parser.setProtocolVersion(version);
    auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
    expected.setProtocolVersion(version);
    parser.setExpectedLayoutHash(expected.layoutHash());
    auto package = std::make_unique<rtde_interface::DataPackage>(expected);
    package->setProtocolVersion(version == 1 ? 2 : 1);
    ASSERT_TRUE(package->setData("timestamp", 42.0));
    ASSERT_NE(package->layoutHash(), expected.layoutHash());
    std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
    products.push_back(std::move(package));
    const auto* original = products.back().get();

    comm::BinParser bp(version == 1 ? version1 : version2, version == 1 ? sizeof(version1) : sizeof(version2));
    URCL_SILENCE_DEPRECATED_BEGIN
    ASSERT_TRUE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    EXPECT_TRUE(bp.empty());
    ASSERT_EQ(products.size(), 1u);
    EXPECT_EQ(products.back().get(), original);
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(products.back().get());
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data->layoutHash(), expected.layoutHash());
    double timestamp = 0.0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_DOUBLE_EQ(timestamp, 1.0);
  }
}

TEST(rtde_parser, deprecated_hash_only_parse_rejects_wrong_recipe_after_protocol_resync)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  for (const uint16_t version : std::initializer_list<uint16_t>{ 1, 2 })
  {
    SCOPED_TRACE(version);
    rtde_interface::RTDEParser parser({ "timestamp" });
    parser.setProtocolVersion(version);
    auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
    expected.setProtocolVersion(version);
    parser.setExpectedLayoutHash(expected.layoutHash());
    auto wrong_recipe = test::typedPackage({ "target_speed_fraction" }, { DataType::DOUBLE });
    wrong_recipe.setProtocolVersion(version);
    ASSERT_NE(wrong_recipe.layoutHash(), expected.layoutHash());
    auto package = std::make_unique<rtde_interface::DataPackage>(wrong_recipe);
    package->setProtocolVersion(version == 1 ? 2 : 1);
    ASSERT_NE(package->layoutHash(), wrong_recipe.layoutHash());
    ASSERT_TRUE(package->setData("target_speed_fraction", 0.25));
    std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
    products.push_back(std::move(package));
    const auto* original = products.back().get();

    comm::BinParser bp(version == 1 ? version1 : version2, version == 1 ? sizeof(version1) : sizeof(version2));
    URCL_SILENCE_DEPRECATED_BEGIN
    EXPECT_FALSE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    ASSERT_EQ(products.size(), 1u);
    EXPECT_EQ(products.back().get(), original);
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(products.back().get());
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data->layoutHash(), wrong_recipe.layoutHash());
    EXPECT_NE(data->layoutHash(), expected.layoutHash());
    double target_speed_fraction = 0.0;
    ASSERT_TRUE(data->getData("target_speed_fraction", target_speed_fraction));
    EXPECT_DOUBLE_EQ(target_speed_fraction, 0.25);
  }
}

TEST(rtde_parser, deprecated_hash_only_parse_rejects_same_width_wrong_type_after_protocol_resync)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  for (const uint16_t version : std::initializer_list<uint16_t>{ 1, 2 })
  {
    SCOPED_TRACE(version);
    rtde_interface::RTDEParser parser({ "timestamp" });
    parser.setProtocolVersion(version);
    auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
    expected.setProtocolVersion(version);
    parser.setExpectedLayoutHash(expected.layoutHash());
    // UINT64 and DOUBLE both occupy eight wire bytes: size alone cannot detect the mismatch.
    auto wrong_type = test::typedPackage({ "timestamp" }, { DataType::UINT64 });
    wrong_type.setProtocolVersion(version);
    ASSERT_NE(wrong_type.layoutHash(), expected.layoutHash());
    auto package = std::make_unique<rtde_interface::DataPackage>(wrong_type);
    package->setProtocolVersion(version == 1 ? 2 : 1);
    ASSERT_NE(package->layoutHash(), wrong_type.layoutHash());
    ASSERT_TRUE(package->setData("timestamp", uint64_t{ 42 }));
    std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
    products.push_back(std::move(package));
    const auto* original = products.back().get();

    comm::BinParser bp(version == 1 ? version1 : version2, version == 1 ? sizeof(version1) : sizeof(version2));
    URCL_SILENCE_DEPRECATED_BEGIN
    EXPECT_FALSE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    ASSERT_EQ(products.size(), 1u);
    EXPECT_EQ(products.back().get(), original);
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(products.back().get());
    ASSERT_NE(data, nullptr);
    EXPECT_EQ(data->layoutHash(), wrong_type.layoutHash());
    EXPECT_NE(data->layoutHash(), expected.layoutHash());
    uint64_t timestamp = 0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_EQ(timestamp, uint64_t{ 42 });
  }
}

TEST(rtde_parser, hash_only_parse_rejects_non_data_pointer)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  expected.setProtocolVersion(2);
  parser.setExpectedLayoutHash(expected.layoutHash());
  auto control = std::make_unique<rtde_interface::ControlPackageStart>();
  control->accepted_ = true;
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::move(control);
  const auto* original = product.get();

  comm::BinParser bp(raw_data, sizeof(raw_data));
  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), original);
  auto* start = dynamic_cast<rtde_interface::ControlPackageStart*>(product.get());
  ASSERT_NE(start, nullptr);
  EXPECT_TRUE(start->accepted_);
}

TEST(rtde_parser, typed_template_replaces_a_non_data_package)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { DataType::DOUBLE }));
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::make_unique<rtde_interface::ControlPackageStart>();

  ASSERT_TRUE(parser.parse(bp, product));
  auto* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
  ASSERT_NE(data, nullptr);
  double timestamp = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 1.0);
}

TEST(rtde_parser, typed_template_follows_protocol_changes)
{
  unsigned char raw_data[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  for (const bool set_version_first : { false, true })
  {
    rtde_interface::RTDEParser parser({ "timestamp" });
    parser.setProtocolVersion(2);
    if (set_version_first)
    {
      parser.setProtocolVersion(1);
    }
    parser.setExpectedDataPackage(expected);
    parser.setProtocolVersion(1);
    comm::BinParser bp(raw_data, sizeof(raw_data));
    std::unique_ptr<rtde_interface::RTDEPackage> product;
    ASSERT_TRUE(parser.parse(bp, product));
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
    ASSERT_NE(data, nullptr);
    double timestamp = 0.0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_DOUBLE_EQ(timestamp, 1.0);
  }
}

TEST(rtde_parser, reused_package_follows_protocol_changes_in_place)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setExpectedDataPackage(expected);
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::make_unique<rtde_interface::DataPackage>(expected);
  auto* original = product.get();

  for (const uint16_t version : std::initializer_list<uint16_t>{ 1, 2, 1 })
  {
    parser.setProtocolVersion(version);
    comm::BinParser bp(version == 1 ? version1 : version2, version == 1 ? sizeof(version1) : sizeof(version2));
    ASSERT_TRUE(parser.parse(bp, product));
    EXPECT_EQ(product.get(), original);
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
    ASSERT_NE(data, nullptr);
    expected.setProtocolVersion(version);
    EXPECT_EQ(data->layoutHash(), expected.layoutHash());
    double timestamp = 0.0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_DOUBLE_EQ(timestamp, 1.0);
  }
}

TEST(rtde_parser, hash_only_registration_is_invalidated_when_protocol_changes)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  expected.setProtocolVersion(2);
  ASSERT_TRUE(expected.setData("timestamp", 42.0));
  const uint64_t original_hash = expected.layoutHash();

  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(original_hash);
  parser.setProtocolVersion(1);

  auto dest = expected;
  {
    comm::BinParser bp(version2, sizeof(version2));
    EXPECT_FALSE(parser.parseDataPackage(bp, dest));
  }
  EXPECT_EQ(dest.layoutHash(), original_hash);
  double timestamp = 0.0;
  ASSERT_TRUE(dest.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);

  {
    comm::BinParser bp(version1, sizeof(version1));
    EXPECT_FALSE(parser.parseDataPackage(bp, dest));
  }
  EXPECT_EQ(dest.layoutHash(), original_hash);
  timestamp = 0.0;
  ASSERT_TRUE(dest.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);

  std::unique_ptr<rtde_interface::RTDEPackage> product = std::make_unique<rtde_interface::DataPackage>(expected);
  const auto* original = product.get();
  comm::BinParser bp(version2, sizeof(version2));
  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), original);
  auto* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
  ASSERT_NE(data, nullptr);
  EXPECT_EQ(data->layoutHash(), original_hash);
  timestamp = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 42.0);
}

TEST(rtde_parser, hash_only_same_protocol_version_keeps_registration)
{
  unsigned char version2[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  expected.setProtocolVersion(2);
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(expected.layoutHash());
  parser.setProtocolVersion(2);

  auto dest = expected;
  comm::BinParser bp(version2, sizeof(version2));
  ASSERT_TRUE(parser.parseDataPackage(bp, dest));
  double timestamp = 0.0;
  ASSERT_TRUE(dest.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 1.0);
}

TEST(rtde_parser, hash_only_parse_succeeds_after_re_registering_new_protocol)
{
  unsigned char version1[] = { 0x00, 0x0b, 0x55, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  expected.setProtocolVersion(2);
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(expected.layoutHash());
  parser.setProtocolVersion(1);

  expected.setProtocolVersion(1);
  parser.setExpectedLayoutHash(expected.layoutHash());
  auto dest = expected;
  comm::BinParser bp(version1, sizeof(version1));
  ASSERT_TRUE(parser.parseDataPackage(bp, dest));
  double timestamp = 0.0;
  ASSERT_TRUE(dest.getData("timestamp", timestamp));
  EXPECT_DOUBLE_EQ(timestamp, 1.0);
}

TEST(rtde_parser, reused_pointer_handles_data_control_data_sequence)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char control[] = { 0x00, 0x04, 0x53, 0x01 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { DataType::DOUBLE }));
  std::unique_ptr<rtde_interface::RTDEPackage> product;

  for (int i = 0; i < 2; ++i)
  {
    comm::BinParser bp(raw_data, sizeof(raw_data));
    ASSERT_TRUE(parser.parse(bp, product));
    auto* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
    ASSERT_NE(data, nullptr);
    double timestamp = 0.0;
    ASSERT_TRUE(data->getData("timestamp", timestamp));
    EXPECT_DOUBLE_EQ(timestamp, 1.0);

    comm::BinParser control_bp(control, sizeof(control));
    ASSERT_TRUE(parser.parse(control_bp, product));
    auto* start = dynamic_cast<rtde_interface::ControlPackageStart*>(product.get());
    ASSERT_NE(start, nullptr);
    EXPECT_TRUE(start->accepted_);
  }
}

TEST(rtde_parser, hash_registration_clears_the_allocation_template)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto expected = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  parser.setExpectedDataPackage(expected);
  parser.setExpectedLayoutHash(expected.layoutHash());
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  comm::BinParser bp(raw_data, sizeof(raw_data));
  EXPECT_FALSE(parser.parse(bp, product));
  EXPECT_EQ(product, nullptr);
}

TEST(rtde_parser, untyped_template_is_rejected_without_changing_registration)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { DataType::DOUBLE }));
  EXPECT_THROW(parser.setExpectedDataPackage(rtde_interface::DataPackage({ "timestamp" })), UrException);
  comm::BinParser bp(raw_data, sizeof(raw_data));
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  EXPECT_TRUE(parser.parse(bp, product));
}

TEST(rtde_parser, foreign_typed_templates_leave_registration_unchanged)
{
  const std::vector<std::string> expected_recipe{ "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(expected_recipe);
  parser.setProtocolVersion(2);
  auto expected = test::typedPackage(expected_recipe, { DataType::DOUBLE, DataType::DOUBLE });
  parser.setExpectedDataPackage(expected);
  const std::vector<std::vector<std::string>> recipes{ { "actual_q" },
                                                       { "target_speed_fraction", "timestamp" },
                                                       { "timestamp", "other_double" } };
  for (const auto& recipe : recipes)
  {
    const std::vector<DataType> types = recipe.size() == 1 ?
                                            std::vector<DataType>{ DataType::VECTOR6D } :
                                            std::vector<DataType>{ DataType::DOUBLE, DataType::DOUBLE };
    EXPECT_THROW(parser.setExpectedDataPackage(test::typedPackage(recipe, types)), UrException);
    uint8_t bytes[128];
    const auto size = expected.serializePackage(bytes);
    comm::BinParser bp(bytes, size);
    std::unique_ptr<rtde_interface::RTDEPackage> result;
    ASSERT_TRUE(parser.parse(bp, result));
    EXPECT_TRUE(dynamic_cast<rtde_interface::DataPackage&>(*result).hasRecipe(expected_recipe));
  }
}

TEST(rtde_parser, borrowed_data_parse_rejects_other_frames_and_recovers)
{
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto output = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  ASSERT_TRUE(output.setData("timestamp", 42.0));
  parser.setExpectedDataPackage(output);
  std::vector<std::vector<uint8_t>> frames{
    { 0x00, 0x04, 0x53, 0x01 },                   // START acknowledgement
    { 0x00, 0x07, 0x4d, 0x01, 'x', 0x00, 0x01 },  // Valid text
    { 0x00, 0x04, 0x4d, 0xff },                   // Truncated text
    { 0x00, 0x04, 0x55, 0x01 },                   // Truncated data
    { 0x00 },                                     // Truncated header
  };
  for (auto& frame : frames)
  {
    comm::BinParser bp(frame.data(), frame.size());
    EXPECT_FALSE(parser.parseDataPackage(bp, output));
    double timestamp = 0;
    ASSERT_TRUE(output.getData("timestamp", timestamp));
    EXPECT_EQ(timestamp, 42.0);
  }
  uint8_t bytes[64];
  const auto size = output.serializePackage(bytes);
  comm::BinParser bp(bytes, size);
  EXPECT_TRUE(parser.parseDataPackage(bp, output));
}

TEST(rtde_parser, borrowed_data_parse_requires_known_matching_layout)
{
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  auto output = test::typedPackage({ "timestamp" }, { DataType::DOUBLE });
  uint8_t bytes[64];
  const auto size = output.serializePackage(bytes);
  comm::BinParser unknown(bytes, size);
  EXPECT_FALSE(parser.parseDataPackage(unknown, output));
  parser.setExpectedDataPackage(output);
  auto foreign = test::typedPackage({ "other" }, { DataType::DOUBLE });
  comm::BinParser mismatch(bytes, size);
  EXPECT_FALSE(parser.parseDataPackage(mismatch, foreign));
  bytes[size] = 0;
  comm::BinParser trailing(bytes, size + 1);
  EXPECT_FALSE(parser.parseDataPackage(trailing, output));
}

TEST(rtde_parser, deprecated_parse_appends_only_complete_packages)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { DataType::DOUBLE }));
  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  for (size_t i = 0; i < 2; ++i)
  {
    comm::BinParser bp(raw_data, sizeof(raw_data) - 1);
    URCL_SILENCE_DEPRECATED_BEGIN
    ASSERT_TRUE(parser.parse(bp, products));
    URCL_SILENCE_DEPRECATED_END
    ASSERT_EQ(products.size(), i + 1);
  }
  EXPECT_NE(products[0].get(), products[1].get());
  comm::BinParser bp(raw_data, sizeof(raw_data));
  URCL_SILENCE_DEPRECATED_BEGIN
  EXPECT_FALSE(parser.parse(bp, products));
  URCL_SILENCE_DEPRECATED_END
  EXPECT_EQ(products.size(), 2u);
}

// The robot reports problems with the connection as text messages, and RTDEClient acts on their
// content while negotiating, so the fields have to come out of the wire intact.
TEST(rtde_parser, text_message_protocol_v2)
{
  // size 0x000f, type 'M', message "hello", source "urcl", warning level 1
  unsigned char raw_data[] = { 0x00, 0x0f, 0x4d, 0x05, 'h', 'e', 'l', 'l', 'o', 0x04, 'u', 'r', 'c', 'l', 0x01 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  rtde_interface::RTDEParser parser({ "" });
  parser.setProtocolVersion(2);

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  ASSERT_TRUE(parser.parse(bp, product));

  auto* message = dynamic_cast<rtde_interface::TextMessage*>(product.get());
  ASSERT_NE(message, nullptr) << "the parser did not produce a TextMessage";
  EXPECT_EQ(message->message_, "hello");
  EXPECT_EQ(message->source_, "urcl");
  EXPECT_EQ(message->warning_level_, 1);
  EXPECT_EQ(message->toString(), "message: hello\nsource: urcl\nwarning level: 1");
}

// A second parse into a package that already has the negotiated layout must not replace it or
// re-apply types. That is the receive-path hash hit.
TEST(rtde_parser, already_typed_package_is_parsed_in_place_without_being_replaced)
{
  unsigned char raw_data[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xd0, 0x07, 0x0d, 0x2f, 0x1a,
                               0x9f, 0xbe, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { DataType::DOUBLE, DataType::DOUBLE }).layoutHash());

  auto package = std::make_unique<rtde_interface::DataPackage>(recipe);
  package->setTypes({ DataType::DOUBLE, DataType::DOUBLE });
  std::unique_ptr<rtde_interface::RTDEPackage> product = std::move(package);
  {
    comm::BinParser bp(raw_data, sizeof(raw_data));
    ASSERT_TRUE(parser.parse(bp, product));
  }
  const rtde_interface::RTDEPackage* package_address = product.get();

  unsigned char second[] = { 0x00, 0x14, 0x55, 0x01, 0x40, 0xc3, 0x88, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  comm::BinParser bp(second, sizeof(second));
  ASSERT_TRUE(parser.parse(bp, product));
  EXPECT_EQ(product.get(), package_address);

  rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(product.get());
  ASSERT_NE(data, nullptr);
  double timestamp = 0.0;
  double target_speed_fraction = 0.0;
  ASSERT_TRUE(data->getData("timestamp", timestamp));
  ASSERT_TRUE(data->getData("target_speed_fraction", target_speed_fraction));
  EXPECT_DOUBLE_EQ(timestamp, 10000.0);
  EXPECT_DOUBLE_EQ(target_speed_fraction, 0.5);
}

// Protocol version 1 puts a message type where version 2 has the lengths, and takes the rest of the
// package as the message.
TEST(rtde_parser, text_message_protocol_v1)
{
  // size 0x000a, type 'M', message type 3, message "legacy"
  unsigned char raw_data[] = { 0x00, 0x0a, 0x4d, 0x03, 'l', 'e', 'g', 'a', 'c', 'y' };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  rtde_interface::RTDEParser parser({ "" });

  std::unique_ptr<rtde_interface::RTDEPackage> product;
  ASSERT_TRUE(parser.parse(bp, product));

  auto* message = dynamic_cast<rtde_interface::TextMessage*>(product.get());
  ASSERT_NE(message, nullptr) << "the parser did not produce a TextMessage";
  EXPECT_EQ(message->message_type_, 3);
  EXPECT_EQ(message->message_, "legacy");
}

std::vector<uint8_t> serializePropertiesResponse(const std::string& types, const std::vector<uint8_t>& values)
{
  const uint16_t payload_size = static_cast<uint16_t>(sizeof(uint16_t) + types.size() + values.size());
  std::vector<uint8_t> buffer(sizeof(uint16_t) + sizeof(uint8_t) + payload_size);
  size_t size = rtde_interface::PackageHeader::serializeHeader(
      buffer.data(), rtde_interface::PackageType::RTDE_READ_PROPERTIES, payload_size);
  size += comm::PackageSerializer::serialize(buffer.data() + size, static_cast<uint16_t>(types.size()));
  size += comm::PackageSerializer::serialize(buffer.data() + size, types);
  if (!values.empty())
  {
    std::memcpy(buffer.data() + size, values.data(), values.size());
  }
  return buffer;
}

std::unique_ptr<rtde_interface::RTDEPackage> parseAnswer(rtde_interface::RTDEParser& parser, std::vector<uint8_t> raw)
{
  comm::BinParser bp(raw.data(), raw.size());
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  EXPECT_TRUE(parser.parse(bp, product));
  return product;
}

std::vector<uint8_t> serializeUint32(const uint32_t value)
{
  std::vector<uint8_t> bytes(sizeof(value));
  comm::PackageSerializer::serialize(bytes.data(), value);
  return bytes;
}

TEST(rtde_parser, control_box_property_to_string)
{
  rtde_interface::ControlBoxProperty box;
  box.type = ControlBoxType::CB5;
  box.subtype = 2;
  EXPECT_EQ(box.toString(), "type CB5");

  box.type = ControlBoxType::UNKNOWN;
  box.subtype = 0;
  EXPECT_EQ(box.toString(), "type UNKNOWN");
}

TEST(rtde_parser, tool_flange_property_to_string)
{
  rtde_interface::ToolFlangeProperty flange;
  flange.type = ToolFlangeType::V2;
  flange.revision = 4;
  EXPECT_EQ(flange.toString(), "type V2, revision 4");
}

TEST(rtde_parser, read_properties_success_response)
{
  uint8_t values[16];
  size_t values_size = 0;
  // 10.15.0, two bytes each from MSB to LSB.
  const uint64_t software = (static_cast<uint64_t>(10) << 48) | (static_cast<uint64_t>(15) << 32);
  // Type byte 5 (CB5), subtype 2.
  const uint32_t control_box = (static_cast<uint32_t>(5) << 24) | (static_cast<uint32_t>(2) << 16);
  // Type byte 2, revision 4.
  const uint32_t tool_flange = (static_cast<uint32_t>(2) << 24) | (static_cast<uint32_t>(4) << 16);
  values_size += comm::PackageSerializer::serialize(values + values_size, software);
  values_size += comm::PackageSerializer::serialize(values + values_size, control_box);
  values_size += comm::PackageSerializer::serialize(values + values_size, tool_flange);

  const std::string types = "UINT64,UINT32,UINT32";
  rtde_interface::RTDEParser parser({ "" });
  auto product =
      parseAnswer(parser, serializePropertiesResponse(types, std::vector<uint8_t>(values, values + values_size)));

  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);
  ASSERT_EQ(answer->size(), 3u);
  EXPECT_EQ(answer->dataType(0), rtde_interface::DataType::UINT64);
  EXPECT_EQ(answer->dataType(1), rtde_interface::DataType::UINT32);
  EXPECT_EQ(answer->dataType(2), rtde_interface::DataType::UINT32);
  EXPECT_TRUE(answer->hasValues());
  ASSERT_EQ(answer->values().size(), 3u);

  rtde_interface::ReadProperties properties(
      { "v1.software.version", "v1.control_box.type", "v1.robot_arm.tool_flange.type" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
  EXPECT_EQ(properties.getDataType("v1.software.version"), rtde_interface::DataType::UINT64);
  EXPECT_EQ(properties.getReportedType("v1.control_box.type"), rtde_interface::DataType::UINT32);
  const std::optional<uint64_t> raw_version = properties.getData<uint64_t>("v1.software.version");
  ASSERT_TRUE(raw_version.has_value());
  EXPECT_EQ(*raw_version, software);

  const std::optional<VersionInformation> version = properties.getSoftwareVersion();
  ASSERT_TRUE(version.has_value());
  EXPECT_EQ(version->major, 10u);
  EXPECT_EQ(version->minor, 15u);
  EXPECT_EQ(version->bugfix, 0u);

  const std::optional<rtde_interface::ControlBoxProperty> box = properties.getControlBoxType();
  ASSERT_TRUE(box.has_value());
  EXPECT_EQ(box->type, ControlBoxType::CB5);
  EXPECT_EQ(box->subtype, 2);

  const std::optional<rtde_interface::ToolFlangeProperty> flange = properties.getToolFlangeType();
  ASSERT_TRUE(flange.has_value());
  EXPECT_EQ(flange->type, ToolFlangeType::V2);
  EXPECT_EQ(flange->revision, 4);

  EXPECT_EQ(properties.toString(), "property data types: UINT64 UINT32 UINT32\nproperty values present: true\n");
}

TEST(rtde_parser, software_version_decodes_the_bugfix_word)
{
  const uint64_t wire_val =
      (static_cast<uint64_t>(5) << 48) | (static_cast<uint64_t>(27) << 32) | (static_cast<uint64_t>(3) << 16) | 0xBEEF;
  uint8_t val_bytes[8];
  comm::PackageSerializer::serialize(val_bytes, wire_val);

  rtde_interface::RTDEParser parser({ "" });
  auto product =
      parseAnswer(parser, serializePropertiesResponse("UINT64", std::vector<uint8_t>(val_bytes, val_bytes + 8)));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);

  rtde_interface::ReadProperties properties({ "v1.software.version" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
  const std::optional<VersionInformation> version = properties.getSoftwareVersion();
  ASSERT_TRUE(version.has_value());
  EXPECT_EQ(version->major, 5u);
  EXPECT_EQ(version->minor, 27u);
  EXPECT_EQ(version->bugfix, 3u);
  EXPECT_EQ(version->build, 0u);
}

TEST(rtde_parser, read_properties_getters_return_empty_for_missing_or_mismatched_type)
{
  const std::string types = "UINT32,UINT64,UINT64";
  uint8_t values[24];
  size_t values_size = 0;
  values_size += comm::PackageSerializer::serialize(values + values_size, static_cast<uint32_t>(10));
  values_size += comm::PackageSerializer::serialize(values + values_size, static_cast<uint64_t>(5));
  values_size += comm::PackageSerializer::serialize(values + values_size, static_cast<uint64_t>(2));

  rtde_interface::RTDEParser parser({ "" });
  auto product =
      parseAnswer(parser, serializePropertiesResponse(types, std::vector<uint8_t>(values, values + values_size)));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);

  rtde_interface::ReadProperties properties(
      { "v1.software.version", "v1.control_box.type", "v1.robot_arm.tool_flange.type" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));

  // Values exist but hold mismatched types (UINT32 instead of UINT64, etc.).
  EXPECT_FALSE(properties.getSoftwareVersion().has_value());
  EXPECT_FALSE(properties.getControlBoxType().has_value());
  EXPECT_FALSE(properties.getToolFlangeType().has_value());
  EXPECT_FALSE(properties.getDataType("nonexistent.property").has_value());

  // Values missing or name never requested.
  rtde_interface::ReadProperties empty_properties;
  EXPECT_FALSE(empty_properties.getSoftwareVersion().has_value());
  EXPECT_FALSE(empty_properties.getControlBoxType().has_value());
  EXPECT_FALSE(empty_properties.getToolFlangeType().has_value());

  rtde_interface::ReadProperties only_version({ "v1.software.version" });
  uint8_t ver_val[8];
  comm::PackageSerializer::serialize(ver_val, static_cast<uint64_t>(10));
  auto ver_product =
      parseAnswer(parser, serializePropertiesResponse("UINT64", std::vector<uint8_t>(ver_val, ver_val + 8)));
  auto* ver_answer = dynamic_cast<rtde_interface::ReadProperties*>(ver_product.get());
  ASSERT_NE(ver_answer, nullptr);
  ASSERT_TRUE(only_version.takeAnswer(std::move(*ver_answer)));
  EXPECT_TRUE(only_version.getSoftwareVersion().has_value());
  EXPECT_FALSE(only_version.getControlBoxType().has_value());
  EXPECT_FALSE(only_version.getToolFlangeType().has_value());
}

TEST(rtde_parser, read_properties_error_token_has_no_values)
{
  rtde_interface::RTDEParser parser({ "" });
  auto product = parseAnswer(parser, serializePropertiesResponse("UINT64,NOT_FOUND", {}));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);
  ASSERT_EQ(answer->size(), 2u);
  EXPECT_EQ(answer->dataType(0), rtde_interface::DataType::UINT64);
  EXPECT_FALSE(answer->dataType(1).has_value());
  EXPECT_FALSE(answer->hasValues());
  EXPECT_TRUE(answer->values().empty());

  rtde_interface::ReadProperties properties({ "v1.software.version", "v1.not.a.property" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
  EXPECT_EQ(properties.getReportedType("v1.software.version"), rtde_interface::DataType::UINT64);
  EXPECT_FALSE(properties.getReportedType("v1.not.a.property").has_value());
  EXPECT_FALSE(properties.getDataType("v1.software.version").has_value());
  EXPECT_FALSE(properties.getData<uint64_t>("v1.software.version").has_value());
  EXPECT_FALSE(properties.getSoftwareVersion().has_value());

  auto not_set = parseAnswer(parser, serializePropertiesResponse("NOT_SET", {}));
  auto* not_set_answer = dynamic_cast<rtde_interface::ReadProperties*>(not_set.get());
  ASSERT_NE(not_set_answer, nullptr);
  EXPECT_FALSE(not_set_answer->hasValues());

  EXPECT_EQ(properties.toString(), "property data types: UINT64 <not a data type>\nproperty values present: false\n");
}

TEST(rtde_parser, read_properties_answer_must_match_the_names)
{
  rtde_interface::RTDEParser parser({ "" });
  auto product = parseAnswer(parser, serializePropertiesResponse("UINT32", serializeUint32(1)));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);

  rtde_interface::ReadProperties properties({ "v1.control_box.type", "v1.robot_arm.tool_flange.type" });
  EXPECT_FALSE(properties.takeAnswer(std::move(*answer)));
  EXPECT_EQ(properties.size(), 0u);
  EXPECT_FALSE(properties.hasValues());
}

TEST(rtde_parser, read_properties_moved_from_has_no_answer)
{
  rtde_interface::RTDEParser parser({ "" });
  auto product = parseAnswer(parser, serializePropertiesResponse("UINT32", serializeUint32(1)));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);

  rtde_interface::ReadProperties properties({ "v1.control_box.type" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
  EXPECT_EQ(answer->size(), 0u);
  EXPECT_FALSE(answer->hasValues());
  EXPECT_TRUE(answer->values().empty());

  const rtde_interface::ReadProperties copied(properties);
  EXPECT_TRUE(properties.hasValues());
  EXPECT_TRUE(copied.hasValues());
  EXPECT_EQ(copied.names(), properties.names());

  rtde_interface::ReadProperties constructed(std::move(properties));
  EXPECT_TRUE(constructed.hasValues());
  EXPECT_EQ(properties.size(), 0u);
  EXPECT_FALSE(properties.hasValues());
  EXPECT_TRUE(properties.names().empty());

  rtde_interface::ReadProperties assigned;
  assigned = std::move(constructed);
  EXPECT_TRUE(assigned.hasValues());
  EXPECT_EQ(assigned.names(), copied.names());
  EXPECT_EQ(constructed.size(), 0u);
  EXPECT_FALSE(constructed.hasValues());
  EXPECT_TRUE(constructed.names().empty());
}

TEST(rtde_parser, read_properties_request_is_a_raw_name_list)
{
  uint8_t buffer[128];
  const rtde_interface::ReadProperties properties({ "v1.software.version", "v1.control_box.type" });
  const size_t size = properties.serializeRequest(buffer, sizeof(buffer));

  const std::string expected = "v1.software.version,v1.control_box.type";
  ASSERT_EQ(size, 3u + expected.size());
  EXPECT_EQ(buffer[0], 0);
  EXPECT_EQ(buffer[1], size);
  EXPECT_EQ(buffer[2], static_cast<uint8_t>(rtde_interface::PackageType::RTDE_READ_PROPERTIES));
  EXPECT_EQ(std::string(reinterpret_cast<char*>(buffer + 3), expected.size()), expected);

  EXPECT_EQ(rtde_interface::ReadProperties(std::vector<std::string>{}).serializeRequest(buffer, sizeof(buffer)), 0u);
  EXPECT_EQ(rtde_interface::ReadProperties({ "v1.software.version", " " }).serializeRequest(buffer, sizeof(buffer)),
            0u);
  EXPECT_EQ(rtde_interface::ReadProperties({ "v1.software.version,v1.control_box.type" })
                .serializeRequest(buffer, sizeof(buffer)),
            0u);
  EXPECT_EQ(rtde_interface::ReadProperties({ "v1.software.version", "," }).serializeRequest(buffer, sizeof(buffer)),
            0u);
}

TEST(rtde_parser, read_properties_request_that_does_not_fit_is_not_serialized)
{
  uint8_t buffer[16];
  const rtde_interface::ReadProperties properties({ "v1.software.version" });
  EXPECT_EQ(properties.serializeRequest(buffer, sizeof(buffer)), 0u);
}

TEST(rtde_parser, read_properties_answer_reuses_the_package_storage)
{
  const std::string types = "UINT64,UINT32,UINT32";
  std::vector<uint8_t> raw = serializePropertiesResponse(types, std::vector<uint8_t>(16, 0));
  rtde_interface::RTDEParser parser({ "" });
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  comm::BinParser first(raw.data(), raw.size());
  ASSERT_TRUE(parser.parse(first, product));
  const rtde_interface::RTDEPackage* address = product.get();
  comm::BinParser second(raw.data(), raw.size());
  ASSERT_TRUE(parser.parse(second, product));

  EXPECT_EQ(product.get(), address);
  auto* properties = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(properties, nullptr);
  ASSERT_EQ(properties->size(), 3u);
  EXPECT_EQ(properties->dataType(0), rtde_interface::DataType::UINT64);
  EXPECT_EQ(properties->dataType(2), rtde_interface::DataType::UINT32);
  EXPECT_EQ(properties->values().size(), 3u);
}

// Parses only the types of an RTDE_READ_PROPERTIES answer, as if no values followed.
bool parsePropertyTypes(rtde_interface::ReadProperties& package, const std::string& types)
{
  std::vector<uint8_t> raw = serializePropertiesResponse(types, {});
  const size_t header_size = sizeof(uint16_t) + sizeof(uint8_t);
  comm::BinParser bp(raw.data() + header_size, raw.size() - header_size);
  return package.parseWith(bp);
}

TEST(rtde_read_properties, entries_that_are_not_data_types_are_empty)
{
  using rtde_interface::DataType;
  using Types = std::vector<std::optional<DataType>>;
  const std::vector<std::pair<std::string, Types>> cases = {
    { "NOT_FOUND", { std::nullopt } },
    { "NOT_SET,UINT32", { std::nullopt, DataType::UINT32 } },
    { "UINT64,NOT_FOUND,UINT32", { DataType::UINT64, std::nullopt, DataType::UINT32 } },
    // Unknown names, including near misses of the error tokens, are not data types either.
    { "NOT_FOUNDED", { std::nullopt } },
    { "UINT64, UINT32", { DataType::UINT64, std::nullopt } },
    // Every comma-separated field is an entry, including the empty ones extra commas produce.
    { "UINT64,", { DataType::UINT64, std::nullopt } },
    { ",UINT64", { std::nullopt, DataType::UINT64 } },
    { "UINT64,,UINT32", { DataType::UINT64, std::nullopt, DataType::UINT32 } },
  };
  rtde_interface::ReadProperties package;
  for (const auto& [types, expected] : cases)
  {
    ASSERT_TRUE(parsePropertyTypes(package, types)) << types;
    EXPECT_FALSE(package.hasValues()) << types;
    ASSERT_EQ(package.size(), expected.size()) << types;
    for (size_t i = 0; i < expected.size(); ++i)
    {
      EXPECT_EQ(package.dataType(i), expected[i]) << types << " entry " << i;
    }
  }
}

TEST(rtde_data_type, parse_data_types_maps_every_data_type)
{
  using rtde_interface::DataType;
  std::vector<std::string_view> names;
  std::vector<std::optional<DataType>> types;
  EXPECT_TRUE(rtde_interface::parseDataTypes("BOOL,UINT8,UINT32,UINT64,INT32,DOUBLE,VECTOR3D,VECTOR6D,VECTOR6INT32,"
                                             "VECTOR6UINT32",
                                             names, types));
  const std::vector<std::optional<DataType>> expected = {
    DataType::BOOL,   DataType::UINT8,    DataType::UINT32,   DataType::UINT64,       DataType::INT32,
    DataType::DOUBLE, DataType::VECTOR3D, DataType::VECTOR6D, DataType::VECTOR6INT32, DataType::VECTOR6UINT32,
  };
  EXPECT_EQ(types, expected);
  ASSERT_EQ(names.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(names[i], rtde_interface::toString(*expected[i]));
  }
}

TEST(rtde_data_type, parse_data_types_keeps_the_words_that_are_not_data_types)
{
  using rtde_interface::DataType;
  std::vector<std::string_view> names;
  std::vector<std::optional<DataType>> types;
  EXPECT_FALSE(rtde_interface::parseDataTypes("NOT_FOUND,DOUBLE,IN_USE,NOT_SET,NOT_FOUNDED", names, types));
  const std::vector<std::optional<DataType>> expected_types = { std::nullopt, DataType::DOUBLE, std::nullopt,
                                                                std::nullopt, std::nullopt };
  const std::vector<std::string_view> expected_names = { rtde_interface::NOT_FOUND_NAME, "DOUBLE",
                                                         rtde_interface::IN_USE_NAME, rtde_interface::NOT_SET_NAME,
                                                         "NOT_FOUNDED" };
  EXPECT_EQ(types, expected_types);
  EXPECT_EQ(names, expected_names);
}

TEST(rtde_data_type, parse_data_types_of_an_empty_list_is_one_empty_entry)
{
  std::vector<std::string_view> names;
  std::vector<std::optional<rtde_interface::DataType>> types;
  EXPECT_FALSE(rtde_interface::parseDataTypes("", names, types));
  ASSERT_EQ(types.size(), 1u);
  EXPECT_FALSE(types[0].has_value());
  EXPECT_EQ(names, std::vector<std::string_view>{ "" });
}

TEST(rtde_data_type, parse_data_types_replaces_the_previous_result)
{
  using rtde_interface::DataType;
  std::vector<std::string_view> names;
  std::vector<std::optional<DataType>> types;
  rtde_interface::parseDataTypes("DOUBLE,UINT32,BOOL", names, types);
  EXPECT_TRUE(rtde_interface::parseDataTypes("INT32", names, types));
  EXPECT_EQ(types, std::vector<std::optional<DataType>>{ DataType::INT32 });
  EXPECT_EQ(names, std::vector<std::string_view>{ "INT32" });
}

TEST(rtde_data_type, known_type_names_lists_every_data_type)
{
  EXPECT_EQ(rtde_interface::knownTypeNames(), "BOOL, UINT8, UINT32, UINT64, INT32, DOUBLE, VECTOR3D, VECTOR6D, "
                                              "VECTOR6INT32, VECTOR6UINT32");
}

TEST(rtde_data_type, a_value_outside_the_enum_is_not_a_data_type)
{
  const auto invalid = static_cast<DataType>(0xff);
  EXPECT_FALSE(rtde_interface::isDataType(invalid));
  EXPECT_THROW(rtde_interface::makeValue(invalid), UrException);
  EXPECT_TRUE(rtde_interface::isDataType(DataType::VECTOR6UINT32));
}

TEST(rtde_read_properties, values_after_an_entry_that_is_not_a_data_type_are_rejected)
{
  rtde_interface::RTDEParser parser({ "" });
  for (const std::string types : { "NOT_FOUND", "UINT32,NOT_A_TYPE" })
  {
    std::vector<uint8_t> raw = serializePropertiesResponse(types, serializeUint32(1));
    comm::BinParser bp(raw.data(), raw.size());
    std::unique_ptr<rtde_interface::RTDEPackage> product;
    EXPECT_FALSE(parser.parse(bp, product)) << types;
  }
}

TEST(rtde_read_properties, set_names_replaces_the_names_and_drops_the_answer)
{
  rtde_interface::RTDEParser parser({ "" });
  auto product = parseAnswer(parser, serializePropertiesResponse("UINT32", serializeUint32(1)));
  auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
  ASSERT_NE(answer, nullptr);

  rtde_interface::ReadProperties properties({ "v1.control_box.type" });
  ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
  properties.setNames({ "v1.control_box.type" });
  EXPECT_FALSE(properties.hasValues());
  EXPECT_EQ(properties.names(), std::vector<std::string>{ "v1.control_box.type" });

  properties.setNames({ "v1.software.version", "v1.control_box.type" });
  const std::vector<std::string> names{ "v1.software.version", "v1.control_box.type" };
  EXPECT_EQ(properties.names(), names);
  EXPECT_FALSE(properties.getReportedType("v1.control_box.type").has_value());
}

TEST(rtde_read_properties, set_names_accepts_views_of_its_own_names)
{
  rtde_interface::ReadProperties properties({ "v1.software.version", "v1.control_box.type" });

  const std::vector<std::string_view> reversed{ properties.names()[1], properties.names()[0] };
  properties.setNames(reversed);
  const std::vector<std::string> expected_reversed{ "v1.control_box.type", "v1.software.version" };
  EXPECT_EQ(properties.names(), expected_reversed);

  const std::vector<std::string_view> first_only{ properties.names()[1] };
  properties.setNames(first_only);
  EXPECT_EQ(properties.names(), std::vector<std::string>{ "v1.software.version" });
}

TEST(rtde_read_properties, catalog_follows_each_product_line)
{
  VersionInformation polyscope_5_28;
  polyscope_5_28.major = 5;
  polyscope_5_28.minor = 28;
  VersionInformation polyscope_x_16;
  polyscope_x_16.major = 10;
  polyscope_x_16.minor = 16;

  std::vector<rtde_interface::PropertySpec> catalog = rtde_interface::builtinPropertyCatalog();
  catalog.push_back({ "v1.future.property", polyscope_5_28, polyscope_x_16 });

  const auto names_for = [&catalog](const uint32_t major, const uint32_t minor) {
    VersionInformation version;
    version.major = major;
    version.minor = minor;
    version.bugfix = 0;
    version.build = 0;
    return rtde_interface::propertyNamesForSoftwareVersion(version, catalog);
  };

  const std::vector<std::string_view> v3_names{ "v1.software.version", "v1.control_box.type",
                                                "v1.robot_arm.tool_flange.type" };
  EXPECT_EQ(names_for(5, 27), v3_names);
  EXPECT_EQ(names_for(10, 15), v3_names);

  std::vector<std::string_view> with_future = v3_names;
  with_future.push_back("v1.future.property");
  EXPECT_EQ(names_for(5, 28), with_future);
  EXPECT_EQ(names_for(10, 16), with_future);
}

TEST(rtde_read_properties, catalog_row_for_one_product_line_is_never_asked_on_the_other)
{
  VersionInformation polyscope_5_28;
  polyscope_5_28.major = 5;
  polyscope_5_28.minor = 28;
  VersionInformation polyscope_x_16;
  polyscope_x_16.major = 10;
  polyscope_x_16.minor = 16;

  const std::vector<rtde_interface::PropertySpec> catalog = {
    { "v1.ps5.only", polyscope_5_28, std::nullopt },
    { "v1.psx.only", std::nullopt, polyscope_x_16 },
  };

  const auto names_for = [&catalog](const uint32_t major, const uint32_t minor, const uint32_t bugfix) {
    VersionInformation version;
    version.major = major;
    version.minor = minor;
    version.bugfix = bugfix;
    version.build = 0;
    return rtde_interface::propertyNamesForSoftwareVersion(version, catalog);
  };

  EXPECT_EQ(names_for(5, 28, 0), (std::vector<std::string_view>{ "v1.ps5.only" }));
  EXPECT_EQ(names_for(10, 99, 0), (std::vector<std::string_view>{ "v1.psx.only" }));
  EXPECT_TRUE(names_for(5, 27, 9).empty());
  EXPECT_TRUE(names_for(10, 15, 9).empty());
}

TEST(rtde_read_properties, catalog_minimum_includes_the_bugfix_level)
{
  VersionInformation ps5_min;
  ps5_min.major = 5;
  ps5_min.minor = 28;
  ps5_min.bugfix = 1;

  VersionInformation psx_min;
  psx_min.major = 10;
  psx_min.minor = 16;
  psx_min.bugfix = 1;

  const std::vector<rtde_interface::PropertySpec> catalog = {
    { "v1.x", ps5_min, psx_min },
  };

  const auto names_for = [&catalog](const uint32_t major, const uint32_t minor, const uint32_t bugfix) {
    VersionInformation version;
    version.major = major;
    version.minor = minor;
    version.bugfix = bugfix;
    version.build = 0;
    return rtde_interface::propertyNamesForSoftwareVersion(version, catalog);
  };

  EXPECT_TRUE(names_for(5, 28, 0).empty());
  EXPECT_EQ(names_for(5, 28, 1), (std::vector<std::string_view>{ "v1.x" }));
  EXPECT_EQ(names_for(5, 29, 0), (std::vector<std::string_view>{ "v1.x" }));

  EXPECT_TRUE(names_for(10, 16, 0).empty());
  EXPECT_EQ(names_for(10, 16, 1), (std::vector<std::string_view>{ "v1.x" }));
  EXPECT_EQ(names_for(10, 17, 0), (std::vector<std::string_view>{ "v1.x" }));
}

TEST(rtde_read_properties, catalog_major_below_10_is_polyscope_5)
{
  VersionInformation ps5_min;
  ps5_min.major = 5;
  ps5_min.minor = 0;

  VersionInformation psx_min;
  psx_min.major = 10;
  psx_min.minor = 0;

  const std::vector<rtde_interface::PropertySpec> catalog = {
    { "v1.ps5", ps5_min, std::nullopt },
    { "v1.psx", std::nullopt, psx_min },
  };

  const auto names_for = [&catalog](const uint32_t major, const uint32_t minor, const uint32_t bugfix) {
    VersionInformation version;
    version.major = major;
    version.minor = minor;
    version.bugfix = bugfix;
    version.build = 0;
    return rtde_interface::propertyNamesForSoftwareVersion(version, catalog);
  };

  EXPECT_EQ(names_for(9, 0, 0), (std::vector<std::string_view>{ "v1.ps5" }));
  EXPECT_EQ(names_for(11, 0, 0), (std::vector<std::string_view>{ "v1.psx" }));
}

TEST(rtde_read_properties, control_box_decoder_accepts_both_numberings)
{
  const std::pair<uint8_t, ControlBoxType> cases[] = {
    { 1, ControlBoxType::CB5 }, { 5, ControlBoxType::CB5 },     { 2, ControlBoxType::CB7 },
    { 7, ControlBoxType::CB7 }, { 3, ControlBoxType::UNKNOWN },
  };
  rtde_interface::RTDEParser parser({ "" });
  for (const auto& [wire, expected] : cases)
  {
    auto product =
        parseAnswer(parser, serializePropertiesResponse("UINT32", serializeUint32(static_cast<uint32_t>(wire) << 24)));
    auto* answer = dynamic_cast<rtde_interface::ReadProperties*>(product.get());
    ASSERT_NE(answer, nullptr);
    rtde_interface::ReadProperties properties({ "v1.control_box.type" });
    ASSERT_TRUE(properties.takeAnswer(std::move(*answer)));
    const std::optional<rtde_interface::ControlBoxProperty> box = properties.getControlBoxType();
    ASSERT_TRUE(box.has_value());
    EXPECT_EQ(box->type, expected) << "wire value " << static_cast<int>(wire);
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
