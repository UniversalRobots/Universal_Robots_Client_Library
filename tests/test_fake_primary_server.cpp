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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "ur_client_library/comm/pipeline.h"
#include "ur_client_library/comm/producer.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/comm/tcp_socket.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"
#include "ur_client_library/primary/primary_package.h"
#include "ur_client_library/primary/primary_parser.h"
#include "ur_client_library/primary/robot_message/error_code_message.h"
#include "ur_client_library/primary/robot_message/key_message.h"
#include "ur_client_library/primary/robot_message/runtime_exception_message.h"
#include "ur_client_library/primary/robot_message/safety_mode_message.h"
#include "ur_client_library/primary/robot_message/text_message.h"
#include "ur_client_library/primary/robot_message/version_message.h"
#include "ur_client_library/primary/robot_state/robot_mode_data.h"

#include "fake_primary_server.h"

using namespace urcl;

namespace
{
// Picks a free, ephemeral port for each test so that they can run in parallel without colliding.
constexpr int ANY_PORT = 0;

/*!
 * \brief Simple consumer that collects parsed PrimaryPackage objects into a thread-safe queue,
 * so individual tests can wait for specific messages to arrive.
 */
class CollectingConsumer : public comm::IConsumer<primary_interface::PrimaryPackage>
{
public:
  bool consume(std::shared_ptr<primary_interface::PrimaryPackage> product) override
  {
    if (product == nullptr)
    {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(product);
    cv_.notify_all();
    return true;
  }

  template <typename T>
  std::shared_ptr<T> waitFor(const std::chrono::milliseconds timeout = std::chrono::seconds(1))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(mutex_);
    while (true)
    {
      while (!queue_.empty())
      {
        auto front = queue_.front();
        queue_.pop();
        if (auto typed = std::dynamic_pointer_cast<T>(front))
        {
          return typed;
        }
      }
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout)
      {
        return nullptr;
      }
    }
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::queue<std::shared_ptr<primary_interface::PrimaryPackage>> queue_;
};

/*!
 * \brief Helper that hooks a real PrimaryParser-based pipeline onto a TCP stream so we can verify
 * that data sent by FakePrimaryServer is actually a well-formed primary package and decodes back
 * to the expected fields.
 */
class TestClient
{
public:
  TestClient(const std::string& host, int port)
    : stream_(host, port)
    , producer_(stream_, parser_)
    , consumer_(std::make_shared<CollectingConsumer>())
    , pipeline_(producer_, consumer_.get(), "TestClient Pipeline", notifier_)
  {
    parser_.setStrictMode(false);
    pipeline_.init();
    pipeline_.run();
  }

  ~TestClient()
  {
    pipeline_.stop();
    stream_.close();
  }

  CollectingConsumer& consumer()
  {
    return *consumer_;
  }

  bool send(const std::string& text)
  {
    size_t written = 0;
    return stream_.write(reinterpret_cast<const uint8_t*>(text.data()), text.size(), written) && written == text.size();
  }

private:
  comm::INotifier notifier_;
  primary_interface::PrimaryParser parser_;
  comm::URStream<primary_interface::PrimaryPackage> stream_;
  comm::URProducer<primary_interface::PrimaryPackage> producer_;
  std::shared_ptr<CollectingConsumer> consumer_;
  comm::Pipeline<primary_interface::PrimaryPackage> pipeline_;
};

}  // namespace

class FakePrimaryServerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_ = std::make_unique<FakePrimaryServer>(ANY_PORT);
    client_ = std::make_unique<TestClient>("127.0.0.1", server_->getPort());
    ASSERT_TRUE(server_->waitForClient(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    client_.reset();
    server_.reset();
  }

  std::unique_ptr<FakePrimaryServer> server_;
  std::unique_ptr<TestClient> client_;
};

TEST_F(FakePrimaryServerTest, send_version_message_roundtrip)
{
  ASSERT_TRUE(server_->sendVersionMessage("URControl", 5, 24, 0, 4242, "04-10-2025, 00:25:33"));

  auto msg = client_->consumer().waitFor<primary_interface::VersionMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->project_name_, "URControl");
  EXPECT_EQ(msg->major_version_, 5);
  EXPECT_EQ(msg->minor_version_, 24);
  EXPECT_EQ(msg->svn_version_, 0);
  EXPECT_EQ(msg->build_number_, 4242);
  EXPECT_EQ(msg->build_date_, "04-10-2025, 00:25:33");
}

TEST_F(FakePrimaryServerTest, send_text_message_roundtrip)
{
  ASSERT_TRUE(server_->sendTextMessage("hello world"));
  auto msg = client_->consumer().waitFor<primary_interface::TextMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->text_, "hello world");
}

TEST_F(FakePrimaryServerTest, send_key_message_roundtrip)
{
  ASSERT_TRUE(server_->sendKeyMessage("PROGRAM_XXX_STARTED", "test_fun", 1, 2));
  auto msg = client_->consumer().waitFor<primary_interface::KeyMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->title_, "PROGRAM_XXX_STARTED");
  EXPECT_EQ(msg->text_, "test_fun");
  EXPECT_EQ(msg->message_code_, 1);
  EXPECT_EQ(msg->message_argument_, 2);
}

TEST_F(FakePrimaryServerTest, send_runtime_exception_message_roundtrip)
{
  ASSERT_TRUE(server_->sendRuntimeExceptionMessage(12, 5, "compile_error_name_not_found:foo:"));
  auto msg = client_->consumer().waitFor<primary_interface::RuntimeExceptionMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->line_number_, 12u);
  EXPECT_EQ(msg->column_number_, 5u);
  EXPECT_EQ(msg->text_, "compile_error_name_not_found:foo:");
}

TEST_F(FakePrimaryServerTest, send_safety_mode_message_roundtrip)
{
  ASSERT_TRUE(server_->sendSafetyModeMessage(SafetyMode::PROTECTIVE_STOP, 7, 8, 0, 1));
  auto msg = client_->consumer().waitFor<primary_interface::SafetyModeMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->safety_mode_type_, SafetyMode::PROTECTIVE_STOP);
  EXPECT_EQ(msg->message_code_, 7);
  EXPECT_EQ(msg->message_argument_, 8);
  EXPECT_EQ(msg->report_data_type_, 0u);
}

TEST_F(FakePrimaryServerTest, send_error_code_message_roundtrip)
{
  ASSERT_TRUE(server_->sendErrorCodeMessage(210, 0, primary_interface::ReportLevel::VIOLATION, "read-only PI"));
  auto msg = client_->consumer().waitFor<primary_interface::ErrorCodeMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->message_code_, 210);
  EXPECT_EQ(msg->report_level_, primary_interface::ReportLevel::VIOLATION);
  EXPECT_EQ(msg->text_, "read-only PI");
}

TEST_F(FakePrimaryServerTest, send_robot_mode_data_roundtrip)
{
  ASSERT_TRUE(server_->sendRobotModeData(RobotMode::RUNNING, true, true, true, false, false, true, false));
  auto msg = client_->consumer().waitFor<primary_interface::RobotModeData>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->robot_mode_, static_cast<int8_t>(RobotMode::RUNNING));
  EXPECT_TRUE(msg->is_real_robot_connected_);
  EXPECT_TRUE(msg->is_program_running_);
  EXPECT_FALSE(msg->is_protective_stopped_);
}

TEST_F(FakePrimaryServerTest, script_callback_is_invoked)
{
  std::mutex mutex;
  std::condition_variable cv;
  std::string captured;
  server_->setScriptCallback([&](const std::string& payload) {
    std::lock_guard<std::mutex> lock(mutex);
    captured = payload;
    cv.notify_all();
  });

  ASSERT_TRUE(client_->send("textmsg(\"hi\")\n"));

  std::unique_lock<std::mutex> lock(mutex);
  ASSERT_TRUE(cv.wait_for(lock, std::chrono::seconds(1), [&]() { return !captured.empty(); }));
  EXPECT_EQ(captured, "textmsg(\"hi\")\n");
  EXPECT_EQ(server_->getLastReceivedScript(), "textmsg(\"hi\")\n");
}

TEST_F(FakePrimaryServerTest, raw_send_forwards_unmodified_bytes)
{
  // Build a minimal VersionMessage by hand and use sendRaw. Total size: 4 (size) + 1 (type) + 8
  // (timestamp) + 1 (source) + 1 (message_type) + 1 (name_length) + 2 (name) + 1 (major) + 1
  // (minor) + 4 (svn) + 4 (build) = 28 bytes; build_date is empty (parsed from remainder).
  const std::vector<uint8_t> raw = {
    0x00, 0x00, 0x00, 0x1C,                          // size = 28 (big-endian)
    0x14,                                            // type = ROBOT_MESSAGE
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // timestamp
    0xfe,                                            // source
    0x03,                                            // robot_message_type = VERSION
    0x02, 0x55, 0x52,                                // project_name_length = 2, "UR"
    0x05, 0x18,                                      // major = 5, minor = 24
    0x00, 0x00, 0x00, 0x00,                          // svn_version = 0
    0x00, 0x00, 0x00, 0x2A                           // build_number = 42
  };

  ASSERT_TRUE(server_->sendRaw(raw.data(), raw.size()));

  auto msg = client_->consumer().waitFor<primary_interface::VersionMessage>();
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->project_name_, "UR");
  EXPECT_EQ(msg->major_version_, 5);
  EXPECT_EQ(msg->minor_version_, 24);
  EXPECT_EQ(msg->svn_version_, 0);
  EXPECT_EQ(msg->build_number_, 42);
  EXPECT_EQ(msg->build_date_, "");
}

TEST(FakePrimaryServerSingleTest, multiple_clients_receive_broadcasts)
{
  FakePrimaryServer server(ANY_PORT);
  TestClient a("127.0.0.1", server.getPort());
  TestClient b("127.0.0.1", server.getPort());

  // Wait until both have connected.
  auto start = std::chrono::steady_clock::now();
  while (server.getClientCount() < 2 && std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(server.getClientCount(), 2u);

  ASSERT_TRUE(server.sendTextMessage("broadcast"));

  auto msg_a = a.consumer().waitFor<primary_interface::TextMessage>();
  auto msg_b = b.consumer().waitFor<primary_interface::TextMessage>();
  ASSERT_NE(msg_a, nullptr);
  ASSERT_NE(msg_b, nullptr);
  EXPECT_EQ(msg_a->text_, "broadcast");
  EXPECT_EQ(msg_b->text_, "broadcast");
}

TEST(FakePrimaryServerSingleTest, send_without_client_returns_false)
{
  FakePrimaryServer server(ANY_PORT);
  EXPECT_FALSE(server.sendTextMessage("nobody listening"));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
