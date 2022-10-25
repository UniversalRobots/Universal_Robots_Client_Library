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

using namespace urcl;

TEST(rtde_data_package, serialize_pkg)
{
  std::vector<std::string> recipe{ "speed_slider_mask" };
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

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
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

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
  rtde_interface::DataPackage package(recipe, 1);
  package.initEmpty();

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

TEST(rtde_data_package, get_data_not_part_of_recipe)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

  uint32_t speed_slider_mask;
  EXPECT_FALSE(package.getData("speed_slider_mask", speed_slider_mask));
}

TEST(rtde_data_package, set_data_not_part_of_recipe)
{
  std::vector<std::string> recipe{ "timestamp", "actual_q" };
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

  uint32_t speed_slider_mask = 1;
  EXPECT_FALSE(package.setData("speed_slider_mask", speed_slider_mask));
}

TEST(rtde_data_package, parse_and_get_bitset_data)
{
  std::vector<std::string> recipe{ "robot_status_bits" };
  rtde_interface::DataPackage package(recipe);
  package.initEmpty();

  uint8_t data_package[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x40, 0xb2, 0x3d, 0xa9, 0xfb, 0xe7, 0x6c, 0x8b };
  comm::BinParser bp(data_package, sizeof(data_package));

  EXPECT_TRUE(package.parseWith(bp));

  std::bitset<4> expected_robot_status_bits = 0000;
  std::bitset<4> actual_robot_status_bits;
  package.getData<uint32_t>("robot_status_bits", actual_robot_status_bits);

  EXPECT_EQ(expected_robot_status_bits, actual_robot_status_bits);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
