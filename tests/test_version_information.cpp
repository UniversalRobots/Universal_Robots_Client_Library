// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
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
 * \date    2022-11-22
 *
 */
//----------------------------------------------------------------------

#include <gtest/gtest.h>

#include <ur_client_library/ur/version_information.h>
#include <ur_client_library/exceptions.h>

using namespace urcl;

TEST(version_information, test_split)
{
  const std::string version_string1 = "5.12.0.1101319";
  std::vector<std::string> expected = { "5", "12", "0", "1101319" };

  EXPECT_EQ(expected, splitString(version_string1));
}

TEST(version_information, string_parsing)
{
  const std::string version_string_full = "5.12.1.1234";
  const std::string version_3 = "5.12.1";
  const std::string version_2 = "5.12";
  auto expected = VersionInformation();
  expected.major = 5;
  expected.minor = 12;
  expected.bugfix = 1;
  expected.build = 1234;

  EXPECT_EQ(expected, VersionInformation::fromString(version_string_full));
  expected.build = 0;
  EXPECT_EQ(expected, VersionInformation::fromString(version_3));
  expected.bugfix = 0;
  EXPECT_EQ(expected, VersionInformation::fromString(version_2));

  const std::string illegal_string("asdy");
  EXPECT_THROW(VersionInformation::fromString(illegal_string), UrException);
  const std::string illegal_string_2("1");
  EXPECT_THROW(VersionInformation::fromString(illegal_string_2), UrException);
}

TEST(version_information, test_relations)
{
  auto v1 = VersionInformation::fromString("5.5.0.1101319");
  auto v2 = VersionInformation::fromString("5.5.0.1101318");
  auto v3 = VersionInformation::fromString("5.5.1");
  auto v4 = VersionInformation::fromString("3.12.0.1234");

  EXPECT_EQ(v1, v1);
  EXPECT_LT(v2, v1);
  EXPECT_LE(v2, v1);
  EXPECT_LE(v1, v1);
  EXPECT_GT(v1, v2);
  EXPECT_GE(v1, v1);
  EXPECT_LT(v1, v3);
  EXPECT_LT(v4, v1);
  EXPECT_TRUE(v1 != v2);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
