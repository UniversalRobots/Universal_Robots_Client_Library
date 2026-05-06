// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <ur_client_library/helpers.h>
#include <ur_client_library/exceptions.h>
#include <ur_client_library/ur/datatypes.h>
#include <ur_client_library/ur/version_information.h>

using namespace urcl;

TEST(TestHelpers, test_parse_boolean)
{
  EXPECT_TRUE(parseBoolean("true"));
  EXPECT_TRUE(parseBoolean("True"));
  EXPECT_TRUE(parseBoolean("TRUE"));
  EXPECT_TRUE(parseBoolean("on"));
  EXPECT_TRUE(parseBoolean("On"));
  EXPECT_TRUE(parseBoolean("ON"));
  EXPECT_TRUE(parseBoolean("yes"));
  EXPECT_TRUE(parseBoolean("Yes"));
  EXPECT_TRUE(parseBoolean("YES"));
  EXPECT_TRUE(parseBoolean("1"));
  EXPECT_FALSE(parseBoolean("false"));
  EXPECT_FALSE(parseBoolean("False"));
  EXPECT_FALSE(parseBoolean("FALSE"));
  EXPECT_FALSE(parseBoolean("off"));
  EXPECT_FALSE(parseBoolean("Off"));
  EXPECT_FALSE(parseBoolean("OFF"));
  EXPECT_FALSE(parseBoolean("no"));
  EXPECT_FALSE(parseBoolean("No"));
  EXPECT_FALSE(parseBoolean("NO"));
  EXPECT_FALSE(parseBoolean("0"));
  EXPECT_THROW(parseBoolean("notabool"), urcl::UrException);
}

TEST(TestHelpers, splitString)
{
  std::vector<std::string> test_vec{ "this", "is", "very", "simple" };
  {
    std::string combined = test_vec[0];
    for (std::size_t i = 1; i < test_vec.size(); ++i)
    {
      combined += "," + test_vec[i];
    }

    EXPECT_EQ(test_vec, urcl::splitString(combined, ","));
  }
  {
    std::string combined = test_vec[0];
    for (std::size_t i = 1; i < test_vec.size(); ++i)
    {
      combined += "--?--" + test_vec[i];
    }

    EXPECT_EQ(test_vec, urcl::splitString(combined, "--?--"));
  }

  const std::string version_string1 = "5.12.0.1101319";
  std::vector<std::string> expected = { "5", "12", "0", "1101319" };
  EXPECT_EQ(expected, splitString(version_string1, "."));
}

TEST(TestHelpers, robotSeriesFromTypeAndVersion)
{
  const VersionInformation cb3_version = VersionInformation::fromString("3.15.0.0");
  const VersionInformation polyscope_5_version = VersionInformation::fromString("5.12.0.1101319");
  const VersionInformation polyscope_x_version = VersionInformation::fromString("10.0.0.0");

  // UR3/UR5/UR10: major >= 5 -> E_SERIES, otherwise CB3
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, cb3_version), RobotSeries::CB3);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, cb3_version), RobotSeries::CB3);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, cb3_version), RobotSeries::CB3);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, polyscope_5_version), RobotSeries::E_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, polyscope_x_version), RobotSeries::E_SERIES);

  // UR16: major >= 5 -> E_SERIES, otherwise UNDEFINED
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, cb3_version), RobotSeries::UNDEFINED);

  // UR15/UR18/UR20/UR30/UR8LONG: major >= 5 -> UR_SERIES, otherwise UNDEFINED
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, polyscope_x_version), RobotSeries::UR_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, polyscope_5_version), RobotSeries::UR_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, cb3_version), RobotSeries::UNDEFINED);

  // UNDEFINED robot type yields UNDEFINED series
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_5_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_x_version), RobotSeries::UNDEFINED);
}
