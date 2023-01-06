// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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
#include <condition_variable>
#include <chrono>

#include <ur_client_library/comm/tcp_server.h>
#define private public
#include <ur_client_library/primary/primary_client.h>

using namespace urcl;

std::string ROBOT_IP = "127.0.0.1";
int PRIMARY_PORT = 30001;
int DASHBOARD_PORT = 29999;
int FAKE_PRIMARY_PORT = 60061;
int FAKE_DASHBOARD_PORT = 60059;

class PrimaryClientTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    in_remote_control_ = true;
  }

  void TearDown()
  {
    dashboard_server_.reset();
    primary_server_.reset();
    client_.reset();
  }

  void run()
  {
    unsigned char message[] = { 0x00, 0x00, 0x00, 0x50, 0x19, 0x00, 0x00, 0x00,
                                0x00, 0x56, 0x76, 0xd3, 0xa0, 0x00, 0x00, 0x00 };  // Empty GlobalVariablesSetupMessage
    size_t len = sizeof(message);
    size_t written;
    while (running_)
    {
      primary_server_->write(client_fd_, message, len, written);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void stopThread()
  {
    URCL_LOG_DEBUG("Shutting down thread");
    running_ = false;
    if (server_thread_.joinable())
    {
      server_thread_.join();
    }
  }

  void connectionCallback(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fd_ = filedescriptor;
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

  void connectionCallbackDB(const int filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_db_);
    client_fd_db_ = filedescriptor;

    unsigned char message[] = {
      0x43, 0x6f, 0x6e, 0x6e, 0x65, 0x63, 0x74, 0x65, 0x64, 0x3a, 0x20, 0x55, 0x6e, 0x69, 0x76, 0x65, 0x72, 0x73, 0x61,
      0x6c, 0x20, 0x52, 0x6f, 0x62, 0x6f, 0x74, 0x73, 0x20, 0x46, 0x61, 0x6b, 0x65, 0x20, 0x54, 0x65, 0x73, 0x74, 0x20,
      0x44, 0x61, 0x73, 0x68, 0x62, 0x6f, 0x61, 0x72, 0x64, 0x20, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x0a
    };  // "Connected: Universal Robots Fake Test Dashboard Server\n"
    size_t len = sizeof(message);
    size_t written;
    dashboard_server_->write(client_fd_db_, message, len, written);
    connect_cv_db_.notify_one();
    connection_callback_db_ = true;
  }

  void messageCallback(const int filedescriptor, char* buffer)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    message_ = std::string(buffer);
    message_cv_.notify_one();
    message_callback_ = true;
  }

  void messageCallbackDB(const int filedescriptor, char* buffer)
  {
    std::lock_guard<std::mutex> lk(message_mutex_db_);
    message_db_ = std::string(buffer);
    if (message_db_ == "is in remote control\n")
    {
      unsigned char message_true[] = { 0x74, 0x72, 0x75, 0x65, 0x0a };         // "true\n"
      unsigned char message_false[] = { 0x66, 0x61, 0x6c, 0x73, 0x65, 0x0a };  // "false\n"

      size_t len = in_remote_control_ ? sizeof(message_true) : sizeof(message_false);
      size_t written;
      dashboard_server_->write(client_fd_db_, in_remote_control_ ? message_true : message_false, len, written);
    }
    if (message_db_ == "PolyscopeVersion\n")
    {
      unsigned char message_pv[] = {
        0x55, 0x52, 0x53, 0x6f, 0x66, 0x74, 0x77, 0x61, 0x72, 0x65, 0x20, 0x35, 0x2e, 0x31,
        0x32, 0x2e, 0x32, 0x2e, 0x31, 0x31, 0x30, 0x31, 0x35, 0x33, 0x34, 0x20, 0x28, 0x4a,
        0x75, 0x6c, 0x20, 0x30, 0x36, 0x20, 0x32, 0x30, 0x32, 0x32, 0x29, 0x0a
      };  // URSoftware 5.12.2.1101534 (Jul 06 2022)
      size_t len = sizeof(message_pv);
      size_t written;
      dashboard_server_->write(client_fd_db_, message_pv, len, written);
    }

    message_cv_db_.notify_one();
    message_callback_db_ = true;
  }

  bool waitForConnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (connect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        connection_callback_ == true)
    {
      connection_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool waitForMessageCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(message_mutex_);
    if (message_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        message_callback_ == true)
    {
      message_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  bool waitForMessageCallbackDB(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(message_mutex_db_);
    if (message_cv_db_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        message_callback_db_ == true)
    {
      message_callback_db_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  std::string message_ = "";
  std::string message_db_ = "";
  int client_fd_ = -1;
  int client_fd_db_ = -1;

  std::atomic<bool> running_;
  std::thread server_thread_;
  std::unique_ptr<primary_interface::PrimaryClient> client_;
  std::unique_ptr<comm::TCPServer> primary_server_;
  std::unique_ptr<comm::TCPServer> dashboard_server_;

  std::condition_variable connect_cv_, connect_cv_db_;
  std::mutex connect_mutex_, connect_mutex_db_;

  std::condition_variable message_cv_, message_cv_db_;
  std::mutex message_mutex_, message_mutex_db_;

  bool connection_callback_ = false;
  bool connection_callback_db_ = false;
  bool message_callback_ = false;
  bool message_callback_db_ = false;

  bool in_remote_control_;
};

TEST_F(PrimaryClientTest, check_remote_control)
{
  dashboard_server_.reset(new comm::TCPServer(FAKE_DASHBOARD_PORT));
  primary_server_.reset(new comm::TCPServer(FAKE_PRIMARY_PORT));
  primary_server_->setMessageCallback(std::bind(&PrimaryClientTest_check_remote_control_Test::messageCallback, this,
                                                std::placeholders::_1, std::placeholders::_2));
  dashboard_server_->setMessageCallback(std::bind(&PrimaryClientTest_check_remote_control_Test::messageCallbackDB, this,
                                                  std::placeholders::_1, std::placeholders::_2));
  primary_server_->setConnectCallback(
      std::bind(&PrimaryClientTest_check_remote_control_Test::connectionCallback, this, std::placeholders::_1));
  dashboard_server_->setConnectCallback(
      std::bind(&PrimaryClientTest_check_remote_control_Test::connectionCallbackDB, this, std::placeholders::_1));
  dashboard_server_->start();
  primary_server_->start();
  server_thread_ = std::thread(&PrimaryClientTest_check_remote_control_Test::run, this);

  std::unique_ptr<primary_interface::PrimaryClient> temp_client;
  client_.reset(new primary_interface::PrimaryClient(ROBOT_IP, ""));
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Let connections set up

  // Disconnect from URSim servers and connect to fake servers
  client_->pipeline_->stop();
  client_->stream_->disconnect();
  client_->dashboard_client_->disconnect();
  client_->dashboard_client_->port_ = FAKE_DASHBOARD_PORT;
  client_->stream_->port_ = FAKE_PRIMARY_PORT;
  client_->stream_->connect();
  client_->dashboard_client_->connect();
  client_->pipeline_->run();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // let connections set up

  // When in_remote_control_ is true the primary client should be able to send script
  in_remote_control_ = true;
  client_->sendScript("true\n");
  EXPECT_TRUE(waitForMessageCallback(1000));

  // Make sure thread sets in_remote_control_ to false and primary client has time to reconnect
  in_remote_control_ = false;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // When in_remote_control_ is false the primary client NOT should be able to send script
  client_->sendScript("false\n");
  EXPECT_FALSE(waitForMessageCallback(1000));

  stopThread();
}

TEST_F(PrimaryClientTest, send_script)
{
  client_.reset(new primary_interface::PrimaryClient(ROBOT_IP, ""));
  client_->setSimulated(true);
  std::stringstream cmd;
  cmd.imbue(std::locale::classic());  // Make sure, decimal divider is actually '.'
  cmd << "sec setup():" << std::endl
      << " textmsg(\"Command through primary interface complete \")" << std::endl
      << "end";

  std::string script_code = cmd.str();
  auto program_with_newline = script_code + '\n';
  // Should always return true in pipeline as robot is simulated
  EXPECT_TRUE(client_->sendScript(program_with_newline));
}

TEST_F(PrimaryClientTest, get_data)
{
  client_.reset(new primary_interface::PrimaryClient(ROBOT_IP, ""));
  client_->setSimulated(true);
  EXPECT_EQ(client_->getVersionMessage()->build_number_, 0);
  vector6d_t zero_array = { 0 };
  EXPECT_EQ(client_->getCartesianInfo()->tcp_offset_coordinates_, zero_array);
  EXPECT_EQ(client_->getForceModeData()->wrench_, zero_array);
  // getGlobalVariablesSetupMessage() will throw an exception since a program on the robot has not been started while
  // this client has been connected
  EXPECT_THROW(client_->getGlobalVariablesSetupMessage()->variable_names_, UrException);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}