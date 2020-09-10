// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-07-09
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_client_library/comm/bin_parser.h>
#include <ur_client_library/rtde/rtde_parser.h>
#include <ur_client_library/rtde/request_protocol_version.h>

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

  if (rtde_interface::GetUrcontrolVersion* data =
          dynamic_cast<rtde_interface::GetUrcontrolVersion*>(products[0].get()))
  {
    EXPECT_EQ(data->version_information_.major, 5);
    EXPECT_EQ(data->version_information_.minor, 8);
    EXPECT_EQ(data->version_information_.bugfix, 0);
    EXPECT_EQ(data->version_information_.build, 0);
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
