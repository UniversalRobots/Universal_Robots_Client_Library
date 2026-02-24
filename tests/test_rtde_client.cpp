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

#include <algorithm>
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <unordered_map>
#include <utility>
#include <iostream>
#include "ur_client_library/comm/tcp_server.h"
#include "ur_client_library/exceptions.h"

#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/version_information.h>

#include "fake_rtde_server.h"
#include "ur_client_library/helpers.h"

using namespace urcl;

std::string g_ROBOT_IP = "192.168.56.101";
uint32_t g_FAKE_RTDE_PORT = 13875;

class RTDEClientTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    client_.reset(
        new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, resources_output_recipe_, resources_input_recipe_));
  }

  void TearDown()
  {
    client_.reset();
    // If we don't sleep we can get a conflict between two tests controlling the same rtde inputs
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::string output_recipe_file_ = "resources/rtde_output_recipe.txt";
  std::string exhaustive_output_recipe_file_ = "resources/exhaustive_rtde_output_recipe.txt";
  std::string docs_output_recipe_file_ = "resources/docs_rtde_output_recipe.txt";
  std::string input_recipe_file_ = "resources/rtde_input_recipe.txt";
  comm::INotifier notifier_;
  std::unique_ptr<rtde_interface::RTDEClient> client_;

  std::vector<std::string> resources_output_recipe_ = { "timestamp",
                                                        "actual_q",
                                                        "actual_qd",
                                                        "speed_scaling",
                                                        "target_speed_fraction",
                                                        "runtime_state",
                                                        "actual_TCP_force",
                                                        "actual_TCP_pose",
                                                        "actual_digital_input_bits",
                                                        "actual_digital_output_bits",
                                                        "standard_analog_input0",
                                                        "standard_analog_input1",
                                                        "standard_analog_output0",
                                                        "standard_analog_output1",
                                                        "analog_io_types",
                                                        "tool_mode",
                                                        "tool_analog_input_types",
                                                        "tool_analog_input0",
                                                        "tool_analog_input1",
                                                        "tool_output_voltage",
                                                        "tool_output_current",
                                                        "tool_temperature",
                                                        "robot_mode",
                                                        "safety_mode",
                                                        "robot_status_bits",
                                                        "safety_status_bits",
                                                        "actual_current",
                                                        "tcp_offset" };

  std::vector<std::string> resources_input_recipe_ = { "speed_slider_mask",
                                                       "speed_slider_fraction",
                                                       "standard_digital_output_mask",
                                                       "standard_digital_output",
                                                       "configurable_digital_output_mask",
                                                       "configurable_digital_output",
                                                       "tool_digital_output_mask",
                                                       "tool_digital_output",
                                                       "standard_analog_output_mask",
                                                       "standard_analog_output_type",
                                                       "standard_analog_output_0",
                                                       "standard_analog_output_1" };
};

TEST_F(RTDEClientTest, rtde_handshake)
{
  EXPECT_TRUE(client_->init());
}

TEST_F(RTDEClientTest, no_recipe)
{
  std::string output_recipe_file = "";
  std::string input_recipe_file = "";
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file, input_recipe_file)),
      UrException);

  // Only input recipe is unconfigured
  EXPECT_NO_THROW(
      client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file)));

  EXPECT_THROW(client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_,
                                                            "/i/do/not/exist/urclrtdetest.txt")),
               UrException);
}

TEST_F(RTDEClientTest, empty_recipe_file)
{
  std::string output_recipe_file = "resources/empty.txt";
  std::string input_recipe_file = "resources/empty.txt";
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file, input_recipe_file)),
      UrException);

  // Only input recipe is empty
  EXPECT_THROW(
      client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file)),
      UrException);
}

TEST_F(RTDEClientTest, invalid_target_frequency)
{
  // Setting target frequency below 0 or above 500, should throw an exception
  client_.reset(
      new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_, -1.0, false));

  EXPECT_THROW(client_->init(), UrException);

  client_.reset(
      new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_, 1000, false));

  EXPECT_THROW(client_->init(), UrException);
}

TEST_F(RTDEClientTest, unconfigured_target_frequency)
{
  // When the target frequency is unconfigured, it should be zero before the client has been initialized
  double expected_target_frequency = 0.0;
  EXPECT_EQ(client_->getTargetFrequency(), expected_target_frequency);

  client_->init();

  // When the target frequency is unconfigured, it should be equal to the maximum frequency after initialization
  EXPECT_EQ(client_->getTargetFrequency(), client_->getMaxFrequency());
}

TEST_F(RTDEClientTest, set_target_frequency)
{
  // Set a target frequency that is different from the maximum frequency but a factor of it.
  // Since we check timestamp differences, we need to make sure that the target frequency is
  // achievable. 25 Hz is a factor of both 125 Hz and 500 Hz.
  const double target_frequency = 25.0;
  client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, input_recipe_file_,
                                               target_frequency, false));
  client_->init();

  // Maximum frequency should still be equal to the robot's maximum frequency
  if (client_->getVersion().major >= 5)
  {
    double expected_max_frequency = 500;
    EXPECT_EQ(client_->getMaxFrequency(), expected_max_frequency);
  }
  else
  {
    double expected_max_frequency = 125;
    EXPECT_EQ(client_->getMaxFrequency(), expected_max_frequency);
  }

  EXPECT_EQ(client_->getTargetFrequency(), target_frequency);

  EXPECT_TRUE(client_->start(false));

  // Test that we receive packages with a frequency of `target_frequency` Hz
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));

  double first_time_stamp = 0.0;
  data_pkg->getData("timestamp", first_time_stamp);

  ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));
  double second_time_stamp = 0.0;
  data_pkg->getData("timestamp", second_time_stamp);

  // There should be 0.1 second between each timestamp
  EXPECT_NEAR(second_time_stamp - first_time_stamp, 1.0 / target_frequency, 1e-6);

  client_->pause();
}

TEST_F(RTDEClientTest, start_uninitialized_client)
{
  // It shouldn't be possible to start an uninitialized client
  EXPECT_FALSE(client_->start());
}

TEST_F(RTDEClientTest, start_client)
{
  client_->init();
  EXPECT_TRUE(client_->start());

  client_->pause();

  // We should be able to start the client again after it has been paused
  EXPECT_TRUE(client_->start());

  client_->pause();
}

TEST_F(RTDEClientTest, pause_client_before_it_was_started)
{
  // We shouldn't be able to pause the client before it has been initialized
  EXPECT_FALSE(client_->pause());

  // We shouldn't be able to pause the client after it has been initialized
  client_->init();
  EXPECT_FALSE(client_->pause());
}

TEST_F(RTDEClientTest, pause_client)
{
  client_->init();
  client_->start();

  EXPECT_TRUE(client_->pause());
}

TEST_F(RTDEClientTest, output_recipe_file)
{
  std::vector<std::string> actual_output_recipe = client_->getOutputRecipe();
  // Verify that the size is the same
  ASSERT_EQ(resources_output_recipe_.size(), actual_output_recipe.size());

  // Verify that the order and contect is equal
  for (unsigned int i = 0; i < resources_output_recipe_.size(); ++i)
  {
    EXPECT_EQ(resources_output_recipe_[i], actual_output_recipe[i]);
  }
}

TEST_F(RTDEClientTest, input_recipe_with_invalid_key)
{
  std::vector<std::string> actual_input_recipe = resources_input_recipe_;
  actual_input_recipe.push_back("i_do_not_exist");

  EXPECT_THROW(client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, resources_output_recipe_,
                                                            actual_input_recipe)),
               RTDEInvalidKeyException);
}

TEST_F(RTDEClientTest, recipe_comparison)
{
  // Check that vectorized constructor provides same recipes as from file
  auto client = rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, resources_output_recipe_, resources_input_recipe_);

  std::vector<std::string> output_recipe_from_file = client_->getOutputRecipe();
  std::vector<std::string> output_recipe_from_vector = client.getOutputRecipe();
  for (unsigned int i = 0; i < output_recipe_from_file.size(); ++i)
  {
    EXPECT_EQ(output_recipe_from_file[i], output_recipe_from_vector[i]);
  }
}

TEST_F(RTDEClientTest, get_data_package_w_background_deprecated)
{
  client_->init();
  client_->start();

  // Test that we can receive a package and extract data from the received package
  const std::chrono::milliseconds read_timeout{ 100 };
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(read_timeout);
#pragma GCC diagnostic pop
  if (data_pkg == nullptr)
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  urcl::vector6d_t actual_q;
  EXPECT_TRUE(data_pkg->getData("actual_q", actual_q));

  client_->pause();
}

TEST_F(RTDEClientTest, get_data_package_w_background)
{
  client_->init();
  client_->start();

  // Test that we can receive a package and extract data from the received package
  const std::chrono::milliseconds read_timeout{ 100 };

  // Create an empty data package. Its timestamp should be 0.0
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  double timestamp;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));
  ASSERT_TRUE(data_pkg.setData("timestamp", 0.0));

  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));

  // Verify that we actually got data from the robot
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));
  // Trying to get data with a very short timeout should fail
  ASSERT_FALSE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(1)));

  // Check the second signature
  auto data_pkg_ptr = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(data_pkg_ptr->setData("timestamp", 0.0));
  ASSERT_TRUE(client_->getDataPackage(data_pkg_ptr, std::chrono::milliseconds(100)));
  EXPECT_TRUE(data_pkg_ptr->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  // Blocking call while packages are fetched in background should fail
  ASSERT_FALSE(client_->getDataPackageBlocking(data_pkg_ptr));

  // starting the background read twice should be fine (no effect)
  ASSERT_NO_THROW(client_->startBackgroundRead());

  ASSERT_NO_THROW(client_->stopBackgroundRead());
  EXPECT_FALSE(client_->getDataPackage(data_pkg_ptr, std::chrono::milliseconds(100)));
  ASSERT_NO_THROW(client_->startBackgroundRead());
  EXPECT_TRUE(client_->getDataPackage(data_pkg_ptr, std::chrono::milliseconds(100)));

  client_->pause();
}

TEST_F(RTDEClientTest, get_data_package_wo_background)
{
  client_->init();
  client_->start(false);

  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(data_pkg->setData("timestamp", 0.0));
  ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));

  urcl::vector6d_t actual_q;
  EXPECT_TRUE(data_pkg->getData("actual_q", actual_q));
  double timestamp;
  EXPECT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  // Non-blocking call should fail since we are not reading in background
  ASSERT_FALSE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)));
  ASSERT_FALSE(client_->getDataPackage(*data_pkg, std::chrono::milliseconds(100)));

  // We should be able to start background reading while the client is started without background
  // reading and then query packages using the non-blocking call
  client_->startBackgroundRead();
  ASSERT_TRUE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)));
  double timestamp_2;
  EXPECT_TRUE(data_pkg->getData("timestamp", timestamp_2));
  EXPECT_GT(timestamp_2, timestamp);
  EXPECT_FALSE(client_->getDataPackageBlocking(data_pkg));

  client_->pause();
}

TEST_F(RTDEClientTest, get_data_package_fake_server)
{
  auto fake_rtde_server = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  // Skip the bootup check. If uptime is less then 40 seconds, data is read for one second to
  // check for safety reset.
  fake_rtde_server->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
  client_.reset(new rtde_interface::RTDEClient("localhost", notifier_, resources_output_recipe_,
                                               resources_input_recipe_, 100, false, g_FAKE_RTDE_PORT));
  client_->init();
  client_->start();

  // Test that we can receive a package and extract data from the received package
  const std::chrono::milliseconds read_timeout{ 100 };
  auto data_pkg = rtde_interface::DataPackage(client_->getOutputRecipe());
  if (!client_->getDataPackage(data_pkg, read_timeout))
  {
    std::cout << "Failed to get data package from robot" << std::endl;
    GTEST_FAIL();
  }

  urcl::vector6d_t actual_q;
  EXPECT_TRUE(data_pkg.getData("actual_q", actual_q));

  URCL_LOG_INFO("Received data package from fake server: %s", data_pkg.toString().c_str());
  client_.reset();
}

TEST_F(RTDEClientTest, reconnect_fake_server_background_read)
{
  auto fake_rtde_server = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  // Skip the bootup check. If uptime is less then 40 seconds, data is read for one second to
  // check for safety reset.
  fake_rtde_server->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
  client_.reset(new rtde_interface::RTDEClient("localhost", notifier_, resources_output_recipe_,
                                               resources_input_recipe_, 100, false, g_FAKE_RTDE_PORT));
  client_->init(0, std::chrono::milliseconds(123), 3, std::chrono::milliseconds(100));
  URCL_LOG_INFO("Client initiliazed");
  client_->start();

  std::atomic<bool> keep_running = true;
  std::thread data_consumer_thread([this, &keep_running]() {
    rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
    const std::chrono::milliseconds read_timeout{ 100 };
    while (keep_running)
    {
      if (client_->getDataPackage(data_pkg, read_timeout))
      {
        // URCL_LOG_INFO(data_pkg.toString().c_str());
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  fake_rtde_server.reset();
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10) &&
         client_->getClientState() != rtde_interface::ClientState::UNINITIALIZED)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  URCL_LOG_INFO("Resetting rtde_server");
  fake_rtde_server = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  fake_rtde_server->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(52));

  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10) &&
         client_->getClientState() != rtde_interface::ClientState::RUNNING)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(client_->getClientState(), rtde_interface::ClientState::RUNNING);

  if (data_consumer_thread.joinable())
  {
    keep_running = false;
    data_consumer_thread.join();
  }
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, std::chrono::milliseconds(100)));
  URCL_LOG_INFO(data_pkg.toString().c_str());

  client_.reset();
  URCL_LOG_INFO("Done");
}

TEST_F(RTDEClientTest, reconnect_fake_server_blocking_read)
{
  auto fake_rtde_server = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  // Skip the bootup check. If uptime is less then 40 seconds, data is read for one second to
  // check for safety reset.
  fake_rtde_server->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
  client_.reset(new rtde_interface::RTDEClient("localhost", notifier_, resources_output_recipe_,
                                               resources_input_recipe_, 100, false, g_FAKE_RTDE_PORT));
  client_->init(0, std::chrono::milliseconds(123), 3, std::chrono::milliseconds(100));
  URCL_LOG_INFO("Client initiliazed");
  client_->start(false);

  std::atomic<bool> keep_running = true;
  std::thread data_consumer_thread([this, &keep_running]() {
    auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
    while (keep_running)
    {
      if (client_->getDataPackageBlocking(data_pkg))
      {
        URCL_LOG_INFO(data_pkg->toString().c_str());
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  fake_rtde_server.reset();
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10) &&
         client_->getClientState() != rtde_interface::ClientState::UNINITIALIZED)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  URCL_LOG_INFO("Resetting rtde_server");
  fake_rtde_server = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  fake_rtde_server->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(52));

  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(10) &&
         client_->getClientState() != rtde_interface::ClientState::RUNNING)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(client_->getClientState(), rtde_interface::ClientState::RUNNING);

  if (data_consumer_thread.joinable())
  {
    keep_running = false;
    data_consumer_thread.join();
  }
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));
  URCL_LOG_INFO(data_pkg->toString().c_str());

  client_.reset();
  URCL_LOG_INFO("Done");
}

TEST_F(RTDEClientTest, write_rtde_data)
{
  client_->init();
  client_->start();

  bool send_digital_output = true;
  EXPECT_TRUE(client_->getWriter().sendStandardDigitalOutput(0, send_digital_output));

  // Make sure that the data has been written to the robot
  const std::chrono::milliseconds read_timeout{ 100 };

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));

  std::bitset<18> actual_dig_out_bits;
  data_pkg.getData<uint64_t>("actual_digital_output_bits", actual_dig_out_bits);

  // If we get the data package to soon the digital output might not have been updated, therefore we get the package a
  // couple of times
  int max_tries = 100;
  int counter = 0;
  while (actual_dig_out_bits[0] != send_digital_output)
  {
    ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));
    data_pkg.getData<uint64_t>("actual_digital_output_bits", actual_dig_out_bits);
    if (counter == max_tries)
    {
      break;
    }
    counter++;
  }

  EXPECT_EQ(send_digital_output, actual_dig_out_bits[0]);

  client_->pause();
}

TEST_F(RTDEClientTest, output_recipe_without_timestamp)
{
  std::string output_recipe_file = "resources/rtde_output_recipe_without_timestamp.txt";
  client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file, input_recipe_file_));

  std::vector<std::string> actual_output_recipe_from_file = client_->getOutputRecipe();
  const std::string timestamp = "timestamp";
  auto it = std::find(actual_output_recipe_from_file.begin(), actual_output_recipe_from_file.end(), timestamp);
  EXPECT_FALSE(it == actual_output_recipe_from_file.end());

  // Verify that timestamp is added to the recipe when using the vectorized constructor
  std::vector<std::string> output_recipe = { "actual_q", "actual_qd" };
  auto client = rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe, resources_input_recipe_);
  std::vector<std::string> actual_output_recipe_from_vector = client.getOutputRecipe();
  it = std::find(actual_output_recipe_from_vector.begin(), actual_output_recipe_from_vector.end(), timestamp);
  EXPECT_FALSE(it == actual_output_recipe_from_vector.end());
}

TEST_F(RTDEClientTest, connect_non_running_robot)
{
  // Make sure that there's no simulator running exposing RTDE on localhost.
  client_.reset(
      new rtde_interface::RTDEClient("127.0.0.1", notifier_, resources_output_recipe_, resources_input_recipe_));
  auto start = std::chrono::system_clock::now();
  EXPECT_THROW(client_->init(2, std::chrono::milliseconds(50), 1), UrException);
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  // This is only a rough estimate, obviously.
  // Since this isn't done on the loopback device, trying to open a socket on a non-existing address
  // takes considerably longer.
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
            2 * comm::TCPSocket::DEFAULT_RECONNECTION_TIME.count());
}

TEST_F(RTDEClientTest, check_all_rtde_output_variables_exist)
{
  const char* env_var = std::getenv("URSIM_VERSION");
  if (env_var == nullptr)
  {
    std::cout << "No URSIM_VERSION environment variable set, skipping test." << std::endl;
    GTEST_SKIP();
  }
  const std::string env_ursim_version(env_var);

  if (env_ursim_version != "latest")
  {
    std::cout << "Not using the latest URSIM version, skipping test. URSIM_VERSION is set to '" << env_ursim_version
              << "'" << std::endl;
    GTEST_SKIP();
  }

  client_->init();

  // Ignore unknown output variables to account for variables not available in old urcontrol versions.
  client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, exhaustive_output_recipe_file_,
                                               input_recipe_file_, 0.0, false));

  EXPECT_TRUE(client_->init());
  client_->start();

  // Test that we can receive and parse the timestamp from the received package to prove the setup was successful
  const std::chrono::milliseconds read_timeout{ 100 };
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));

  double timestamp;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  client_->pause();
}

TEST_F(RTDEClientTest, check_rtde_data_fields_match_docs)
{
  std::ifstream docs_file(docs_output_recipe_file_);
  std::ifstream pkg_file(exhaustive_output_recipe_file_);
  std::vector<std::string> docs_outputs;
  std::string line;
  while (std::getline(docs_file, line))
  {
    docs_outputs.push_back(line);
  }
  std::vector<std::string> pkg_outputs;
  while (std::getline(pkg_file, line))
  {
    pkg_outputs.push_back(line);
  }
  std::sort(docs_outputs.begin(), docs_outputs.end());
  std::sort(pkg_outputs.begin(), pkg_outputs.end());
  if (!std::is_permutation(docs_outputs.begin(), docs_outputs.end(), pkg_outputs.begin(), pkg_outputs.end()))
  {
    std::cout << "Data package output fields do not match output fields in documentation" << std::endl;
    std::unordered_map<std::string, int> diff;
    std::cout << "Differences: " << std::endl;
    for (auto name : docs_outputs)
    {
      diff[name] += 1;
    }
    for (auto name : pkg_outputs)
    {
      diff[name] -= 1;
    }
    for (auto elem : diff)
    {
      if (elem.second > 0)
      {
        std::cout << elem.first << " exists in documentation, but not in data package dict." << std::endl;
      }
      if (elem.second < 0)
      {
        std::cout << elem.first << " exists in data package dict, but not in documentation." << std::endl;
      }
    }
    GTEST_FAIL();
  }
}

TEST_F(RTDEClientTest, check_unknown_rtde_output_variable)
{
  client_->init();

  std::vector<std::string> incorrect_output_recipe = client_->getOutputRecipe();
  incorrect_output_recipe.push_back("unknown_rtde_variable");

  // If unknown variables are not ignored, initialization should fail
  EXPECT_THROW(client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, incorrect_output_recipe,
                                                            resources_input_recipe_, 0.0, false)),
               RTDEInvalidKeyException);

  // Unknown variables (by the control box) can be ignored, so initialization should succeed
  if ((client_->getVersion().major == 5 && client_->getVersion().minor < 23) ||
      (client_->getVersion().major == 10 && client_->getVersion().minor < 11))
  {
    std::vector<std::string> output_recipe = client_->getOutputRecipe();
    output_recipe.push_back("actual_robot_energy_consumed");  // That has been added in 5.23.0 / 10.11.0
    client_.reset(
        new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe, resources_input_recipe_, 0.0, true));
    EXPECT_TRUE(client_->init());
  }

  // Passing a completely unknown variable should still lead to an exception, even if unknown
  // variables are ignored.
  EXPECT_THROW(client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, incorrect_output_recipe,
                                                            resources_input_recipe_, 0.0, true)),
               RTDEInvalidKeyException);
}

TEST_F(RTDEClientTest, empty_input_recipe)
{
  std::vector<std::string> empty_input_recipe = {};
  client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, resources_output_recipe_, empty_input_recipe));
  client_->init();
  client_->start();

  // Test that we can receive and parse the timestamp from the received package to prove the setup was successful
  const std::chrono::milliseconds read_timeout{ 100 };
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));

  double timestamp;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));

  EXPECT_FALSE(client_->getWriter().sendStandardDigitalOutput(1, false));

  client_->pause();

  client_.reset(new rtde_interface::RTDEClient(g_ROBOT_IP, notifier_, output_recipe_file_, ""));
  client_->init();
  client_->start();

  ASSERT_TRUE(client_->getDataPackage(data_pkg, read_timeout));
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));

  EXPECT_FALSE(client_->getWriter().sendStandardDigitalOutput(1, false));

  client_->pause();
}

TEST_F(RTDEClientTest, test_initialization)
{
  // Test that initialization fails with 0 initialization attempts
  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(100), 0), UrException);

  comm::TCPServer dummy_server(UR_RTDE_PORT);
  dummy_server.start();

  // Test that initialization fails when no RTDE interface is available on the robot
  // within the given initialization attempts
  // We use a dummy server here that doesn't implement the RTDE interface
  // to simulate this scenario.
  // The total time should be at least (initialization attempts - 1) * initialization timeout
  // since the last attempt doesn't wait after failing.

  URCL_LOG_INFO("Starting initialization timing test");
  client_.reset(new rtde_interface::RTDEClient("127.0.0.1", notifier_, resources_output_recipe_, {}));
  auto start = std::chrono::system_clock::now();
  EXPECT_THROW(client_->init(2, std::chrono::milliseconds(10), 2, std::chrono::milliseconds(10)), UrException);
  auto end = std::chrono::system_clock::now();
  auto elapsed = end - start;
  EXPECT_GE(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 20);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      g_ROBOT_IP = argv[i + 1];
      break;
    }
  }

  return RUN_ALL_TESTS();
}
