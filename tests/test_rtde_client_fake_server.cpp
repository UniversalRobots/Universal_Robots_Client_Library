// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

// Covers the public surface of RTDEClient against the fake RTDE server. The tests in
// test_rtde_client.cpp are more thorough but need a reachable robot, so they only run when
// INTEGRATION_TESTS is enabled; these run everywhere.

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include <ur_client_library/comm/stream.h>
#include <ur_client_library/exceptions.h>
#include <ur_client_library/log.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/rtde/request_protocol_version.h>

#include "fake_rtde_server.h"

using namespace urcl;

namespace
{
constexpr int g_FAKE_RTDE_PORT = 60006;
constexpr double g_RTDE_FREQUENCY = 125.0;
// The fake server answers the version query with 10.10.10.10, so the client takes the e-Series limit
constexpr double g_MAX_FREQUENCY = 500.0;
constexpr std::chrono::milliseconds g_READ_TIMEOUT{ 200 };

const std::vector<std::string> g_OUTPUT_RECIPE{ "timestamp", "actual_q", "target_speed_fraction", "runtime_state" };
const std::vector<std::string> g_INPUT_RECIPE{ "speed_slider_mask", "speed_slider_fraction" };
}  // namespace

class RTDEClientFakeServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
    // Skip the client's bootup check, which would otherwise read data for a second
    server_->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
    client_ = makeClient(g_OUTPUT_RECIPE, g_INPUT_RECIPE, g_RTDE_FREQUENCY);
  }

  void TearDown() override
  {
    client_.reset();
    server_.reset();
  }

  std::unique_ptr<rtde_interface::RTDEClient> makeClient(const std::vector<std::string>& output_recipe,
                                                         const std::vector<std::string>& input_recipe,
                                                         double target_frequency,
                                                         bool ignore_unavailable_outputs = false)
  {
    return std::make_unique<rtde_interface::RTDEClient>("localhost", notifier_, output_recipe, input_recipe,
                                                        target_frequency, ignore_unavailable_outputs, g_FAKE_RTDE_PORT);
  }

  comm::INotifier notifier_;
  std::unique_ptr<RTDEServer> server_;
  std::unique_ptr<rtde_interface::RTDEClient> client_;
};

// The address is the one the socket resolved to, so it only exists once the socket is connected.
TEST_F(RTDEClientFakeServerTest, get_ip)
{
  EXPECT_TRUE(client_->getIP().empty());

  ASSERT_TRUE(client_->init());

  EXPECT_EQ(client_->getIP(), "127.0.0.1");
}

TEST_F(RTDEClientFakeServerTest, recipes_are_reported_as_given)
{
  EXPECT_EQ(client_->getOutputRecipe(), g_OUTPUT_RECIPE);
  EXPECT_EQ(client_->getInputRecipe(), g_INPUT_RECIPE);
}

// The client needs the timestamp to tell whether the robot has finished booting, so it adds the
// field to recipes that don't ask for it.
TEST_F(RTDEClientFakeServerTest, timestamp_is_added_to_the_output_recipe)
{
  auto client = makeClient({ "actual_q" }, g_INPUT_RECIPE, g_RTDE_FREQUENCY);

  const std::vector<std::string> expected_recipe{ "actual_q", "timestamp" };
  EXPECT_EQ(client->getOutputRecipe(), expected_recipe);
}

TEST_F(RTDEClientFakeServerTest, read_recipe_from_file)
{
  const std::vector<std::string> recipe = rtde_interface::RTDEClient::readRecipe("resources/rtde_input_recipe.txt");

  EXPECT_FALSE(recipe.empty());
  EXPECT_EQ(recipe.front(), "speed_slider_mask");

  EXPECT_THROW(rtde_interface::RTDEClient::readRecipe("resources/there_is_no_such_recipe.txt"), UrException);
}

TEST_F(RTDEClientFakeServerTest, client_state_follows_the_communication)
{
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);

  ASSERT_TRUE(client_->init());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::INITIALIZED);

  ASSERT_TRUE(client_->start());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::RUNNING);

  ASSERT_TRUE(client_->pause());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::PAUSED);
}

TEST_F(RTDEClientFakeServerTest, init_is_idempotent)
{
  ASSERT_TRUE(client_->init());
  EXPECT_TRUE(client_->init());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::INITIALIZED);
}

TEST_F(RTDEClientFakeServerTest, start_and_pause_out_of_order)
{
  EXPECT_FALSE(client_->start());
  EXPECT_FALSE(client_->pause());

  ASSERT_TRUE(client_->init());
  EXPECT_FALSE(client_->pause());

  ASSERT_TRUE(client_->start());
  ASSERT_TRUE(client_->pause());
  // A paused client can be started again
  EXPECT_TRUE(client_->start());
  EXPECT_TRUE(client_->pause());
}

TEST_F(RTDEClientFakeServerTest, version_is_taken_from_the_robot)
{
  ASSERT_TRUE(client_->init());

  const VersionInformation version = client_->getVersion();
  EXPECT_EQ(version.major, 10);
  EXPECT_EQ(version.minor, 10);
}

// A PolyScope X simulator answers the version query with "SafetySetup has not been confirmed yet"
// on every connect until it has been switched on, so the client retries instead of giving up.
TEST_F(RTDEClientFakeServerTest, version_query_retries_past_a_safety_setup_text_message)
{
  server_->queueTextMessageBeforeVersionReply("SafetySetup has not been confirmed yet");

  ASSERT_TRUE(client_->init());

  const VersionInformation version = client_->getVersion();
  EXPECT_EQ(version.major, 10);
}

// Any other text message is worth reporting, but is still only a reason to retry.
TEST_F(RTDEClientFakeServerTest, version_query_retries_past_an_unexpected_text_message)
{
  server_->queueTextMessageBeforeVersionReply("Something else entirely");
  server_->queueTextMessageBeforeVersionReply("And again");

  ASSERT_TRUE(client_->init());

  EXPECT_EQ(client_->getVersion().major, 10);
}

// Retrying is bounded: MAX_REQUEST_RETRIES text messages in a row and the handshake fails rather
// than looping forever.
TEST_F(RTDEClientFakeServerTest, version_query_gives_up_after_too_many_text_messages)
{
  for (int i = 0; i < 10; ++i)
  {
    server_->queueTextMessageBeforeVersionReply("SafetySetup has not been confirmed yet");
  }

  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10)), UrException);
}

// A controller that does not know the newest protocol version refuses it, and the client works its
// way down instead of failing.
TEST_F(RTDEClientFakeServerTest, protocol_version_is_lowered_when_the_robot_refuses_it)
{
  server_->setHighestAcceptedProtocolVersion(1);

  // Whether the rest of the handshake completes over version 1 is beside the point here; what
  // matters is that being refused made the client ask for a lower version.
  try
  {
    client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10));
  }
  catch (const UrException&)
  {
  }

  const std::vector<uint16_t> requested = server_->requestedProtocolVersions();
  ASSERT_GE(requested.size(), 2u) << "the client never asked for a second protocol version";
  EXPECT_EQ(requested[0], 2) << "the client should try the newest version first";
  EXPECT_EQ(requested[1], 1) << "the client should fall back to the next version down";
}

TEST_F(RTDEClientFakeServerTest, init_succeeds_after_protocol_v1_fallback)
{
  server_->setHighestAcceptedProtocolVersion(1);

  ASSERT_TRUE(client_->init());
  const std::vector<uint16_t> requested = server_->requestedProtocolVersions();
  ASSERT_GE(requested.size(), 2u);
  EXPECT_EQ(requested[0], 2);
  EXPECT_EQ(requested[1], 1);

  ASSERT_TRUE(client_->start(true));
  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));
  double timestamp = 0.0;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  rtde_interface::DataPackage input_pkg = client_->createInputDataPackage();
  ASSERT_TRUE(input_pkg.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  ASSERT_TRUE(input_pkg.setData("speed_slider_fraction", 0.25));
  EXPECT_TRUE(client_->getWriter().sendPackage(input_pkg));

  client_->pause();
}

// TCP can deliver several RTDE packages in one recv(). The fake server used to handle only the
// first and drop the rest, which is what made a pause request vanish after a burst of input data.
TEST_F(RTDEClientFakeServerTest, two_requests_in_one_write_are_both_recorded)
{
  comm::URStream<rtde_interface::RTDEPackage> stream("localhost", g_FAKE_RTDE_PORT);
  ASSERT_TRUE(stream.connect(1, std::chrono::milliseconds(100)));

  uint8_t buffer[16];
  const size_t first = rtde_interface::RequestProtocolVersionRequest::generateSerializedRequest(buffer, 2);
  const size_t second = rtde_interface::RequestProtocolVersionRequest::generateSerializedRequest(buffer + first, 1);
  const size_t total = first + second;
  size_t written = 0;
  ASSERT_TRUE(stream.write(buffer, total, written));
  ASSERT_EQ(written, total);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  std::vector<uint16_t> requested;
  while (std::chrono::steady_clock::now() < deadline)
  {
    requested = server_->requestedProtocolVersions();
    if (requested.size() >= 2)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_EQ(requested.size(), 2u);
  EXPECT_EQ(requested[0], 2);
  EXPECT_EQ(requested[1], 1);
}

// If no version is acceptable the handshake has to fail, not spin.
TEST_F(RTDEClientFakeServerTest, init_fails_when_no_protocol_version_is_accepted)
{
  server_->setHighestAcceptedProtocolVersion(0);

  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10)), UrException);

  const std::vector<uint16_t> requested = server_->requestedProtocolVersions();
  EXPECT_FALSE(requested.empty());
  EXPECT_EQ(requested.back(), 1) << "the client should have tried every version down to the lowest";
}

// A thrown init() used to leave the client INITIALIZING, so a later call returned true without
// talking to the robot. After the handshake is allowed to succeed, the second init() has to
// actually finish it.
TEST_F(RTDEClientFakeServerTest, init_can_be_retried_after_a_failed_handshake)
{
  server_->setHighestAcceptedProtocolVersion(0);
  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10)), UrException);
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);

  server_->setHighestAcceptedProtocolVersion(2);
  ASSERT_TRUE(client_->init());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::INITIALIZED);
}

// isRobotBooted() reads packages until the reported uptime is 40 seconds or two seconds of data
// have arrived. Every other test skips that wait; this one leaves the fake server's clock at now
// and uses a low frequency so the loop body actually runs.
TEST_F(RTDEClientFakeServerTest, init_waits_out_the_bootup_period)
{
  client_.reset();
  server_.reset();
  server_ = std::make_unique<RTDEServer>(g_FAKE_RTDE_PORT);
  const double bootup_frequency = 2.0;
  client_ = makeClient(g_OUTPUT_RECIPE, g_INPUT_RECIPE, bootup_frequency);

  ASSERT_TRUE(client_->init());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::INITIALIZED);
}

// A robot that refuses to start RTDE must not leave the client believing it is streaming.
TEST_F(RTDEClientFakeServerTest, start_is_refused_when_the_robot_rejects_it)
{
  ASSERT_TRUE(client_->init());
  server_->setAcceptStart(false);

  EXPECT_FALSE(client_->start());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::INITIALIZED);
}

TEST_F(RTDEClientFakeServerTest, pause_is_refused_when_the_robot_rejects_it)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start());
  server_->setAcceptPause(false);

  EXPECT_FALSE(client_->pause());
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::RUNNING);
}

// setupOutputs() retries on unexpected replies and then gives up, rather than treating a text
// message as an acknowledgement.
TEST_F(RTDEClientFakeServerTest, init_fails_when_setup_outputs_never_gets_an_acknowledgement)
{
  for (unsigned i = 0; i < rtde_interface::MAX_REQUEST_RETRIES; ++i)
  {
    server_->queueTextMessageBeforeSetupOutputs("not a setup-outputs reply");
  }

  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10)), UrException);
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
}

TEST_F(RTDEClientFakeServerTest, init_fails_when_setup_inputs_never_gets_an_acknowledgement)
{
  for (unsigned i = 0; i < rtde_interface::MAX_REQUEST_RETRIES; ++i)
  {
    server_->queueTextMessageBeforeSetupInputs("not a setup-inputs reply");
  }

  EXPECT_THROW(client_->init(1, std::chrono::milliseconds(10), 1, std::chrono::milliseconds(10)), UrException);
  EXPECT_EQ(client_->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
}

TEST_F(RTDEClientFakeServerTest, target_frequency_defaults_to_the_maximum)
{
  auto client = makeClient(g_OUTPUT_RECIPE, g_INPUT_RECIPE, 0.0);
  EXPECT_EQ(client->getTargetFrequency(), 0.0);

  ASSERT_TRUE(client->init());

  EXPECT_EQ(client->getMaxFrequency(), g_MAX_FREQUENCY);
  EXPECT_EQ(client->getTargetFrequency(), client->getMaxFrequency());
}

TEST_F(RTDEClientFakeServerTest, configured_target_frequency_is_kept)
{
  ASSERT_TRUE(client_->init());

  EXPECT_EQ(client_->getMaxFrequency(), g_MAX_FREQUENCY);
  EXPECT_EQ(client_->getTargetFrequency(), g_RTDE_FREQUENCY);
}

TEST_F(RTDEClientFakeServerTest, target_frequency_outside_the_robots_range_throws)
{
  auto too_low = makeClient(g_OUTPUT_RECIPE, g_INPUT_RECIPE, -1.0);
  EXPECT_THROW(too_low->init(), UrException);
  EXPECT_EQ(too_low->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  // A thrown init() must leave the client usable for another attempt, not stuck INITIALIZING.
  EXPECT_THROW(too_low->init(), UrException);
  EXPECT_EQ(too_low->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  // The fake server allows only one client; drop the first connection before opening another.
  too_low.reset();

  auto too_high = makeClient(g_OUTPUT_RECIPE, g_INPUT_RECIPE, g_MAX_FREQUENCY + 1.0);
  EXPECT_THROW(too_high->init(), UrException);
  EXPECT_EQ(too_high->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
}

TEST_F(RTDEClientFakeServerTest, unknown_field_is_accepted_by_the_constructor)
{
  EXPECT_NO_THROW(makeClient({ "timestamp", "not_a_field_the_robot_knows" }, g_INPUT_RECIPE, g_RTDE_FREQUENCY));
}

TEST_F(RTDEClientFakeServerTest, receive_with_background_read)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg.getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  // Blocking reads would compete with the background thread for the same packages
  auto blocking_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  EXPECT_FALSE(client_->getDataPackageBlocking(blocking_pkg));

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, receive_with_background_read_into_a_unique_ptr)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, receive_without_background_read)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(false));

  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackageBlocking(data_pkg));

  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  // Without the background thread there is nothing for the non-blocking overload to read from
  rtde_interface::DataPackage other_pkg(client_->getOutputRecipe());
  EXPECT_FALSE(client_->getDataPackage(other_pkg, g_READ_TIMEOUT));

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, background_read_can_be_stopped_and_started)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  client_->stopBackgroundRead();
  EXPECT_FALSE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  client_->startBackgroundRead();
  EXPECT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, background_read_before_init_is_refused)
{
  client_->startBackgroundRead();

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  EXPECT_FALSE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));
}

TEST_F(RTDEClientFakeServerTest, deprecated_get_data_package_returns_a_usable_package)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  URCL_SILENCE_DEPRECATED_BEGIN
  std::unique_ptr<rtde_interface::DataPackage> data_pkg = client_->getDataPackage(g_READ_TIMEOUT);
  URCL_SILENCE_DEPRECATED_END

  ASSERT_NE(data_pkg, nullptr);
  double timestamp = 0.0;
  ASSERT_TRUE(data_pkg->getData("timestamp", timestamp));
  EXPECT_GT(timestamp, 0.0);

  client_->pause();
}

// The fake server echoes the speed slider fraction back as target_speed_fraction, which is enough to
// see a value travel all the way through the writer and back.
TEST_F(RTDEClientFakeServerTest, received_package_rejects_a_wrong_get_data_type)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));

  uint32_t timestamp_as_int = 0;
  EXPECT_FALSE(data_pkg.getData("timestamp", timestamp_as_int));
  double timestamp = 0.0;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, create_input_data_package_matches_the_robot_types)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  rtde_interface::DataPackage typed = client_->createInputDataPackage();
  EXPECT_EQ(typed.getDataType("speed_slider_mask"), rtde_interface::DataType::UINT32);
  EXPECT_EQ(typed.getDataType("speed_slider_fraction"), rtde_interface::DataType::DOUBLE);
  ASSERT_TRUE(typed.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  ASSERT_TRUE(typed.setData("speed_slider_fraction", 0.3));
  EXPECT_TRUE(client_->getWriter().sendPackage(typed));

  rtde_interface::DataPackage recipe_built(client_->getInputRecipe());
  ASSERT_TRUE(recipe_built.setData("speed_slider_mask", static_cast<uint32_t>(1)));
  ASSERT_TRUE(recipe_built.setData("speed_slider_fraction", 0.4));
  EXPECT_TRUE(client_->getWriter().sendPackage(recipe_built));

  rtde_interface::DataPackage wrong(client_->getInputRecipe());
  ASSERT_TRUE(wrong.setData("speed_slider_mask", static_cast<uint8_t>(1)));
  EXPECT_FALSE(client_->getWriter().sendPackage(wrong));

  client_->pause();
}

TEST_F(RTDEClientFakeServerTest, write_and_read_back_input_data)
{
  ASSERT_TRUE(client_->init());
  ASSERT_TRUE(client_->start(true));

  rtde_interface::DataPackage input_pkg(client_->getInputRecipe());
  ASSERT_TRUE(input_pkg.setData<uint32_t>("speed_slider_mask", 1));
  ASSERT_TRUE(input_pkg.setData("speed_slider_fraction", 0.25));
  ASSERT_TRUE(client_->getWriter().sendPackage(input_pkg));

  rtde_interface::DataPackage data_pkg(client_->getOutputRecipe());
  double target_speed_fraction = 0.0;
  for (int i = 0; i < 20 && target_speed_fraction == 0.0; ++i)
  {
    ASSERT_TRUE(client_->getDataPackage(data_pkg, g_READ_TIMEOUT));
    ASSERT_TRUE(data_pkg.getData("target_speed_fraction", target_speed_fraction));
  }
  EXPECT_DOUBLE_EQ(target_speed_fraction, 0.25);

  client_->pause();
}

// The robot is the authority on which fields exist, so a typo in a recipe is caught from the
// acknowledgement rather than from a table inside the library.
TEST_F(RTDEClientFakeServerTest, unknown_output_field_throws)
{
  auto client = makeClient({ "timestamp", "not_a_field_the_robot_knows" }, g_INPUT_RECIPE, g_RTDE_FREQUENCY);

  EXPECT_THROW(client->init(), RTDEInvalidKeyException);
  EXPECT_EQ(client->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  EXPECT_THROW(client->init(), RTDEInvalidKeyException);
}

TEST_F(RTDEClientFakeServerTest, unknown_output_field_can_be_ignored)
{
  auto client =
      makeClient({ "timestamp", "actual_q", "not_a_field_the_robot_knows" }, g_INPUT_RECIPE, g_RTDE_FREQUENCY, true);

  ASSERT_TRUE(client->init());

  const std::vector<std::string> expected_recipe{ "timestamp", "actual_q" };
  EXPECT_EQ(client->getOutputRecipe(), expected_recipe);

  ASSERT_TRUE(client->start(true));
  rtde_interface::DataPackage data_pkg(client->getOutputRecipe());
  EXPECT_TRUE(client->getDataPackage(data_pkg, g_READ_TIMEOUT));

  client->pause();
}

// Without a library-owned field table a typo and a field of a newer robot look the same. Both are
// stripped when ignore_unavailable_outputs is set.
TEST_F(RTDEClientFakeServerTest, ignore_unavailable_outputs_strips_a_typo_and_an_unknown_newer_field)
{
  auto client = makeClient({ "timestamp", "actual_q", "typo_output", "field_of_a_newer_robot" }, g_INPUT_RECIPE,
                           g_RTDE_FREQUENCY, true);

  ASSERT_TRUE(client->init());

  const std::vector<std::string> expected_recipe{ "timestamp", "actual_q" };
  EXPECT_EQ(client->getOutputRecipe(), expected_recipe);
}

TEST_F(RTDEClientFakeServerTest, unknown_input_field_throws)
{
  auto client = makeClient(g_OUTPUT_RECIPE, { "not_a_field_the_robot_knows" }, g_RTDE_FREQUENCY);

  EXPECT_THROW(client->init(), RTDEInvalidKeyException);
  EXPECT_EQ(client->getClientState(), rtde_interface::ClientState::UNINITIALIZED);
  EXPECT_THROW(client->init(), RTDEInvalidKeyException);
}

// The other constructor takes recipe files, and a missing or empty output recipe is rejected right
// away rather than at handshake time.
TEST_F(RTDEClientFakeServerTest, create_input_data_package_before_init_throws)
{
  EXPECT_THROW(client_->createInputDataPackage(), UrException);
}

TEST_F(RTDEClientFakeServerTest, empty_input_recipe_does_not_start_the_writer)
{
  auto client = makeClient(g_OUTPUT_RECIPE, {}, g_RTDE_FREQUENCY);

  ASSERT_TRUE(client->init());
  EXPECT_THROW(client->createInputDataPackage(), UrException);

  ASSERT_TRUE(client->start(true));
  rtde_interface::DataPackage data_pkg(client->getOutputRecipe());
  EXPECT_TRUE(client->getDataPackage(data_pkg, g_READ_TIMEOUT));
  double timestamp = 0.0;
  EXPECT_TRUE(data_pkg.getData("timestamp", timestamp));

  client->pause();
}

TEST_F(RTDEClientFakeServerTest, recipe_files)
{
  EXPECT_NO_THROW(rtde_interface::RTDEClient("localhost", notifier_, "resources/rtde_output_recipe.txt",
                                             "resources/rtde_input_recipe.txt", g_RTDE_FREQUENCY, false,
                                             g_FAKE_RTDE_PORT));

  EXPECT_THROW(rtde_interface::RTDEClient("localhost", notifier_, "", "resources/rtde_input_recipe.txt",
                                          g_RTDE_FREQUENCY, false, g_FAKE_RTDE_PORT),
               UrException);

  EXPECT_THROW(rtde_interface::RTDEClient("localhost", notifier_, "resources/empty.txt",
                                          "resources/rtde_input_recipe.txt", g_RTDE_FREQUENCY, false, g_FAKE_RTDE_PORT),
               UrException);

  EXPECT_THROW(rtde_interface::RTDEClient("localhost", notifier_, "resources/rtde_output_recipe.txt",
                                          "/i/do/not/exist/urclrtdetest.txt", g_RTDE_FREQUENCY, false,
                                          g_FAKE_RTDE_PORT),
               UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  setLogLevel(LogLevel::ERROR);

  return RUN_ALL_TESTS();
}
