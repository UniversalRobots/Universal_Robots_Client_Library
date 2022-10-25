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

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/rtde/rtde_parser.h>

using namespace urcl;

TEST(rtde_parser, request_protocol_version)
{
  // Accepted request protocol version
  unsigned char raw_data[] = { 0x00, 0x04, 0x56, 0x01 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::RequestProtocolVersion* data =
          dynamic_cast<rtde_interface::RequestProtocolVersion*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::GetUrcontrolVersion* data = dynamic_cast<rtde_interface::GetUrcontrolVersion*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::ControlPackagePause* data = dynamic_cast<rtde_interface::ControlPackagePause*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::ControlPackageStart* data = dynamic_cast<rtde_interface::ControlPackageStart*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::ControlPackageSetupInputs* data =
          dynamic_cast<rtde_interface::ControlPackageSetupInputs*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.setProtocolVersion(2);
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::ControlPackageSetupOutputs* data =
          dynamic_cast<rtde_interface::ControlPackageSetupOutputs*>(products[0].get()))
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

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  std::vector<std::string> recipe = { "timestamp", "target_speed_fraction" };
  rtde_interface::RTDEParser parser(recipe);
  parser.setProtocolVersion(2);
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  if (rtde_interface::DataPackage* data = dynamic_cast<rtde_interface::DataPackage*>(products[0].get()))
  {
    double timestamp, target_speed_fraction;
    data->getData("timestamp", timestamp);
    data->getData("target_speed_fraction", target_speed_fraction);

    EXPECT_FLOAT_EQ(timestamp, 16412.2);
    EXPECT_EQ(target_speed_fraction, 1);
  }
  else
  {
    std::cout << "Failed to get data package data" << std::endl;
    GTEST_FAIL();
  }
}

TEST(rtde_parser, test_to_string)
{
  // Non-existent type
  unsigned char raw_data[] = { 0x00, 0x05, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  parser.parse(bp, products);

  EXPECT_EQ(products.size(), 1);

  std::stringstream expected;
  expected << "Type: 2" << std::endl;
  expected << "Raw byte stream: 0 0 " << std::endl;

  EXPECT_EQ(products[0]->toString(), expected.str());
}

TEST(rtde_parser, test_buffer_too_short)
{
  // Non-existent type with false size information
  unsigned char raw_data[] = { 0x00, 0x06, 0x02, 0x00, 0x00 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  EXPECT_FALSE(parser.parse(bp, products));
}

TEST(rtde_parser, test_buffer_too_long)
{
  // Non-existent type with false size information
  unsigned char raw_data[] = { 0x00, 0x04, 0x56, 0x01, 0x02, 0x01, 0x02 };
  comm::BinParser bp(raw_data, sizeof(raw_data));

  std::vector<std::unique_ptr<rtde_interface::RTDEPackage>> products;
  rtde_interface::RTDEParser parser({ "" });
  EXPECT_FALSE(parser.parse(bp, products));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
