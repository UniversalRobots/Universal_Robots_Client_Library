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

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/rtde/rtde_parser.h>

#include "rtde_test_helpers.h"

using namespace urcl;

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
  parser.setExpectedDataPackage(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }));
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
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }).layoutHash());
  parser.setProtocolVersion(2);

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
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }).layoutHash());
  parser.setProtocolVersion(2);

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
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }).layoutHash());
  parser.setProtocolVersion(2);

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
  auto expected_package = test::typedPackage(recipe, { "DOUBLE", "DOUBLE" });
  expected_package.setProtocolVersion(1);
  parser.setExpectedLayoutHash(expected_package.layoutHash());
  parser.setProtocolVersion(1);

  auto package = std::make_unique<rtde_interface::DataPackage>(recipe);
  package->setTypes({ "DOUBLE", "DOUBLE" });
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
  parser.setExpectedDataPackage(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }));
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
  parser.setExpectedDataPackage(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }));
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
  auto package = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
    auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
  auto expected = test::typedPackage(recipe, { "DOUBLE", "DOUBLE" });
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
    auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
    auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
    expected.setProtocolVersion(version);
    parser.setExpectedLayoutHash(expected.layoutHash());
    auto wrong_recipe = test::typedPackage({ "target_speed_fraction" }, { "DOUBLE" });
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
    auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
    expected.setProtocolVersion(version);
    parser.setExpectedLayoutHash(expected.layoutHash());
    // UINT64 and DOUBLE both occupy eight wire bytes: size alone cannot detect the mismatch.
    auto wrong_type = test::typedPackage({ "timestamp" }, { "UINT64" });
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
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { "DOUBLE" }));
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
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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

TEST(rtde_parser, reused_pointer_handles_data_control_data_sequence)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  unsigned char control[] = { 0x00, 0x04, 0x53, 0x01 };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { "DOUBLE" }));
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
  auto expected = test::typedPackage({ "timestamp" }, { "DOUBLE" });
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
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { "DOUBLE" }));
  EXPECT_THROW(parser.setExpectedDataPackage(rtde_interface::DataPackage({ "timestamp" })), UrException);
  comm::BinParser bp(raw_data, sizeof(raw_data));
  std::unique_ptr<rtde_interface::RTDEPackage> product;
  EXPECT_TRUE(parser.parse(bp, product));
}

TEST(rtde_parser, deprecated_parse_appends_only_complete_packages)
{
  unsigned char raw_data[] = { 0x00, 0x0c, 0x55, 0x01, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff };
  rtde_interface::RTDEParser parser({ "timestamp" });
  parser.setProtocolVersion(2);
  parser.setExpectedDataPackage(test::typedPackage({ "timestamp" }, { "DOUBLE" }));
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
  parser.setExpectedLayoutHash(test::typedPackage(recipe, { "DOUBLE", "DOUBLE" }).layoutHash());
  parser.setProtocolVersion(2);

  auto package = std::make_unique<rtde_interface::DataPackage>(recipe);
  package->setTypes({ "DOUBLE", "DOUBLE" });
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

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
