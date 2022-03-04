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

#include <ur_client_library/rtde/rtde_client.h>

using namespace urcl;

std::string ROBOT_IP = "192.168.56.101";

TEST(UrRobotDriver, rtde_handshake)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe);

  EXPECT_TRUE(client.init());
}

/*
* Currently these tests wont work, since we no longer throw an exception at a wrong IP address
* TODO fix these tests
TEST(UrRobotDriver, rtde_handshake_wrong_ip)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client("192.168.56.123", notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}

TEST(UrRobotDriver, rtde_handshake_illegal_ip)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/rtde_output_recipe.txt";
  std::string input_recipe = "resources/rtde_input_recipe.txt";
  rtde_interface::RTDEClient client("abcd", notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}*/

TEST(UrRobotDriver, no_recipe)
{
  comm::INotifier notifier;
  std::string output_recipe = "";
  std::string input_recipe = "";
  EXPECT_THROW(rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe), UrException);
}

TEST(UrRobotDriver, empty_recipe)
{
  comm::INotifier notifier;
  std::string output_recipe = "resources/empty.txt";
  std::string input_recipe = "resources/empty.txt";
  rtde_interface::RTDEClient client(ROBOT_IP, notifier, output_recipe, input_recipe);

  EXPECT_THROW(client.init(), UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      ROBOT_IP = argv[i + 1];
      break;
    }
  }

  return RUN_ALL_TESTS();
}
