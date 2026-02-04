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

#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/ur/calibration_checker.h>
#include <chrono>
#include <memory>
#include <queue>
#include <stdexcept>
#include <thread>
#include "ur_client_library/exceptions.h"
#include "ur_client_library/helpers.h"

using namespace urcl;

std::string g_ROBOT_IP = "192.168.56.101";

class RobotMessageConsumer : public comm::IConsumer<primary_interface::PrimaryPackage>
{
public:
  RobotMessageConsumer() = default;
  virtual ~RobotMessageConsumer() = default;

  virtual bool consume(std::shared_ptr<primary_interface::PrimaryPackage> product)
  {
    auto message_ptr = std::dynamic_pointer_cast<primary_interface::RobotMessage>(product);

    if (message_ptr != nullptr)
    {
      std::lock_guard<std::mutex> lock(data_lock_);
      package_queue_.push(message_ptr);
      URCL_LOG_INFO("%s", message_ptr->toString().c_str());
      data_cv_.notify_one();
      return true;
    }
    return true;
  }

  std::shared_ptr<primary_interface::RobotMessage>
  getOrWaitForMessage(const std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    std::unique_lock<std::mutex> lock(data_lock_);
    if (!package_queue_.empty() || data_cv_.wait_for(lock, timeout) == std::cv_status::no_timeout)
    {
      auto data = package_queue_.front();
      package_queue_.pop();
      return data;
    }
    throw TimeoutException("Did not receive RobotMessage in time", timeout);
  }

private:
  std::queue<std::shared_ptr<primary_interface::RobotMessage>> package_queue_;
  std::condition_variable data_cv_;
  std::mutex data_lock_;
};

class PrimaryClientTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_ = std::make_unique<primary_interface::PrimaryClient>(g_ROBOT_IP, notifier_);
  }

  std::unique_ptr<primary_interface::PrimaryClient> client_;
  comm::INotifier notifier_;
};

TEST_F(PrimaryClientTest, start_communication_succeeds)
{
  EXPECT_NO_THROW(client_->start());
}

TEST_F(PrimaryClientTest, add_and_remove_consumer)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  auto calibration_consumer = std::make_shared<urcl::CalibrationChecker>("test");
#pragma GCC diagnostic pop

  client_->addPrimaryConsumer(calibration_consumer);

  EXPECT_NO_THROW(client_->start());

  auto start_time = std::chrono::system_clock::now();
  const auto timeout = std::chrono::seconds(5);
  while (!calibration_consumer->isChecked() && std::chrono::system_clock::now() - start_time < timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(calibration_consumer->isChecked());

  client_->removePrimaryConsumer(calibration_consumer);
}

TEST_F(PrimaryClientTest, test_power_cycle_commands)
{
  EXPECT_NO_THROW(client_->start());
  EXPECT_NO_THROW(client_->commandPowerOff());
  EXPECT_NO_THROW(client_->commandPowerOn());
  EXPECT_NO_THROW(client_->commandBrakeRelease());
  EXPECT_NO_THROW(client_->commandPowerOff());
  EXPECT_NO_THROW(client_->commandBrakeRelease());
  EXPECT_NO_THROW(client_->commandPowerOff());

  auto timeout = std::chrono::seconds(30);
  // provoke a timeout
  EXPECT_THROW(client_->commandBrakeRelease(true, std::chrono::milliseconds(1)), urcl::TimeoutException);
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::RUNNING; }, timeout));
  EXPECT_THROW(client_->commandPowerOff(true, std::chrono::milliseconds(1)), urcl::TimeoutException);
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::POWER_OFF; }, timeout));
  EXPECT_THROW(client_->commandPowerOn(true, std::chrono::milliseconds(1)), urcl::TimeoutException);
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::IDLE; }, timeout));

  // Without a verification the calls should succeed, the robot ending up in the desired state
  // eventually.
  EXPECT_NO_THROW(client_->commandPowerOff(false));
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::POWER_OFF; }, timeout));
  EXPECT_NO_THROW(client_->commandPowerOn(false));
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::IDLE; }, timeout));
  EXPECT_NO_THROW(client_->commandBrakeRelease(false));
  EXPECT_NO_THROW(waitFor([this]() { return client_->getRobotMode() == RobotMode::RUNNING; }, timeout));
}

TEST_F(PrimaryClientTest, test_unlock_protective_stop)
{
  EXPECT_NO_THROW(client_->start());
  EXPECT_NO_THROW(client_->commandBrakeRelease(true, std::chrono::milliseconds(20000)));
  client_->sendScript("protective_stop()");
  EXPECT_NO_THROW(waitFor([this]() { return client_->isRobotProtectiveStopped(); }, std::chrono::milliseconds(1000)));
  // This will not happen immediately
  EXPECT_THROW(client_->commandUnlockProtectiveStop(true, std::chrono::milliseconds(1)), TimeoutException);

  // It is not allowed to unlock the protective stop immediately
  std::this_thread::sleep_for(std::chrono::seconds(5));
  EXPECT_NO_THROW(client_->commandUnlockProtectiveStop());
}

TEST_F(PrimaryClientTest, test_uninitialized_primary_client)
{
  // The client is not started yet, so the robot mode should be UNKNOWN
  ASSERT_EQ(client_->getRobotMode(), RobotMode::UNKNOWN);
  ASSERT_THROW(client_->isRobotProtectiveStopped(), UrException);
  // The client is not started yet, so the robot type should be UNDEFINED
  ASSERT_EQ(client_->getRobotType(), RobotType::UNDEFINED);
}

TEST_F(PrimaryClientTest, test_stop_command)
{
  // Without started communication the latest robot mode data is a nullptr
  EXPECT_THROW(client_->commandStop(), UrException);

  EXPECT_NO_THROW(client_->start());
  EXPECT_NO_THROW(client_->commandPowerOff());
  EXPECT_NO_THROW(client_->commandBrakeRelease());

  const std::string script_code = "def test_fun():\n"
                                  "  while True:\n"
                                  "     textmsg(\"still running\")\n"
                                  "     sleep(1.0)\n"
                                  "     sync()\n"
                                  "  end\n"
                                  "end";

  EXPECT_TRUE(client_->sendScript(script_code));
  waitFor([this]() { return client_->getRobotModeData()->is_program_running_; }, std::chrono::seconds(5));

  EXPECT_NO_THROW(client_->commandStop());
  EXPECT_FALSE(client_->getRobotModeData()->is_program_running_);

  // Without a program running it should not throw an exception
  EXPECT_NO_THROW(client_->commandStop());

  EXPECT_TRUE(client_->sendScript(script_code));
  waitFor([this]() { return client_->getRobotModeData()->is_program_running_; }, std::chrono::seconds(5));
  EXPECT_THROW(client_->commandStop(true, std::chrono::milliseconds(1)), TimeoutException);
  EXPECT_NO_THROW(waitFor(
      [this]() {
        return !client_->getRobotModeData()->is_program_running_ && !client_->getRobotModeData()->is_program_paused_;
      },
      std::chrono::seconds(5)));

  // without validation
  EXPECT_TRUE(client_->sendScript(script_code));
  waitFor([this]() { return client_->getRobotModeData()->is_program_running_; }, std::chrono::seconds(5));
  EXPECT_NO_THROW(client_->commandStop(false));
  EXPECT_NO_THROW(waitFor(
      [this]() {
        return !client_->getRobotModeData()->is_program_running_ && !client_->getRobotModeData()->is_program_paused_;
      },
      std::chrono::seconds(5)));
}

TEST_F(PrimaryClientTest, test_configuration_data)
{
  EXPECT_NO_THROW(client_->start());

  // Once we connect to the primary client we should receive configuration data
  auto start_time = std::chrono::system_clock::now();
  const auto timeout = std::chrono::seconds(1);
  auto config_data = client_->getConfigurationData();
  while (config_data == nullptr && std::chrono::system_clock::now() - start_time < timeout)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    config_data = client_->getConfigurationData();
  }

  // We should have received a configuration data package
  EXPECT_NE(config_data, nullptr);

  // Robot type should no longer be undefined once we have received configuration data.
  EXPECT_NE(client_->getRobotType(), RobotType::UNDEFINED);
}

TEST_F(PrimaryClientTest, test_program_execution)
{
  auto consumer = std::make_shared<RobotMessageConsumer>();

  client_->addPrimaryConsumer(consumer);

  EXPECT_NO_THROW(client_->start());
  EXPECT_NO_THROW(client_->commandPowerOff());
  EXPECT_NO_THROW(client_->commandBrakeRelease());

  const std::string script_code = "def test_fun():\n"
                                  "  textmsg(\"still running\")\n"
                                  "  sleep(0.1)\n"
                                  "  sync()\n"
                                  "end";

  EXPECT_TRUE(client_->sendScript(script_code));

  {  // we get a key message that the program started
    bool answer_received = false;
    while (!answer_received)
    {
      auto message = consumer->getOrWaitForMessage();
      auto key_message = std::dynamic_pointer_cast<primary_interface::KeyMessage>(message);
      if (key_message)
      {
        answer_received = true;
        EXPECT_EQ(key_message->title_, "PROGRAM_XXX_STARTED");
        EXPECT_EQ(key_message->text_, "test_fun");
      }
    }
  }

  {  // we get the textmessage of the program
    bool answer_received = false;
    while (!answer_received)
    {
      auto message = consumer->getOrWaitForMessage();
      auto text_message = std::dynamic_pointer_cast<primary_interface::TextMessage>(message);
      if (text_message)
      {
        answer_received = true;
        EXPECT_EQ(text_message->text_, "still running");
      }
    }
  }

  {  // we get a key message that the program stopped
    bool answer_received = false;
    while (!answer_received)
    {
      auto message = consumer->getOrWaitForMessage();
      auto key_message = std::dynamic_pointer_cast<primary_interface::KeyMessage>(message);
      if (key_message)
      {
        answer_received = true;
        EXPECT_EQ(key_message->title_, "PROGRAM_XXX_STOPPED");
        EXPECT_EQ(key_message->text_, "test_fun");
      }
    }
  }
}

TEST_F(PrimaryClientTest, test_program_execution_reports_exception)
{
  auto consumer = std::make_shared<RobotMessageConsumer>();

  client_->addPrimaryConsumer(consumer);

  EXPECT_NO_THROW(client_->start());
  EXPECT_NO_THROW(client_->commandPowerOff());
  EXPECT_NO_THROW(client_->commandBrakeRelease());

  const std::string script_code = "def illegal_fun():\n"
                                  "  calldoesntexist()\n"
                                  "end";

  EXPECT_TRUE(client_->sendScript(script_code));

  {  // we get a RuntimeException message saying that out function doesn't exist
    bool answer_received = false;
    while (!answer_received)
    {
      auto message = consumer->getOrWaitForMessage();
      auto typed_msg = std::dynamic_pointer_cast<primary_interface::RuntimeExceptionMessage>(message);
      if (typed_msg)
      {
        answer_received = true;
        EXPECT_EQ(typed_msg->line_number_, 2);
        EXPECT_EQ(typed_msg->column_number_, 3);
        EXPECT_EQ(typed_msg->text_, "compile_error_name_not_found:calldoesntexist:");
      }
    }
  }
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
