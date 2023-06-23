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
#include <iterator>
#include <numeric>

#include <ur_client_library/control/script_command_interface.h>
#include <ur_client_library/comm/tcp_socket.h>

using namespace urcl;

class ScriptCommandInterfaceTest : public ::testing::Test
{
protected:
  class Client : public comm::TCPSocket
  {
  public:
    Client(const int& port)
    {
      std::string host = "127.0.0.1";
      TCPSocket::setup(host, port);
      timeval tv;
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      TCPSocket::setReceiveTimeout(tv);
    }

    void send(const int32_t& result)
    {
      uint8_t buffer[sizeof(int32_t)];
      int32_t val = htobe32(result);
      std::memcpy(buffer, &val, sizeof(int32_t));
      size_t written = 0;
      TCPSocket::write(buffer, sizeof(buffer), written);
    }

    void readMessage(int32_t& command, std::vector<int32_t>& message)
    {
      // Max message length is 26
      uint8_t buf[sizeof(int32_t) * 26];
      uint8_t* b_pos = buf;
      size_t read = 0;
      size_t remainder = sizeof(int32_t) * 26;
      while (remainder > 0)
      {
        if (!TCPSocket::read(b_pos, remainder, read))
        {
          std::cout << "Failed to read from socket, this should not happen during a test!" << std::endl;
          break;
        }
        b_pos += read;
        remainder -= read;
      }

      // Reset buffer pos for parsing
      b_pos = buf;
      uint8_t* b_end = b_pos + read;

      // Decode command signal
      int32_t val;
      std::memcpy(&val, b_pos, sizeof(int32_t));
      command = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode remainder of message
      while (b_pos < b_end)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        message.push_back(be32toh(val));
        b_pos += sizeof(int32_t);
      }
    }

  protected:
    virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
    {
      return ::connect(socket_fd, address, address_len) == 0;
    }
  };

  void SetUp()
  {
    script_command_interface_.reset(new control::ScriptCommandInterface(50004));
    client_.reset(new Client(50004));
  }

  void TearDown()
  {
    if (script_command_interface_->clientConnected() == true)
    {
      client_->close();
      waitForClientConnection(false);
    }
  }

  bool waitForClientConnection(bool client_connected = true,
                               std::chrono::duration<double> timeout = std::chrono::milliseconds(1000))
  {
    const std::chrono::duration<double> wait_period = std::chrono::milliseconds(50);
    std::chrono::duration<double> time_done(0);
    while (time_done < timeout)
    {
      if (script_command_interface_->clientConnected() == client_connected)
      {
        return true;
      }
      std::this_thread::sleep_for(wait_period);
      time_done += wait_period;
    }
    return false;
  }

  bool waitToolContactResult(control::ToolContactResult result, int milliseconds = 1000)
  {
    std::unique_lock<std::mutex> lk(tool_contact_result_mutex_);
    if (tool_contact_result_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        received_result_ == result)
    {
      if (received_result_ == result)
      {
        return true;
      }
    }
    return false;
  }

  std::unique_ptr<control::ScriptCommandInterface> script_command_interface_;
  std::unique_ptr<Client> client_;
  control::ToolContactResult received_result_;

public:
  void handleToolContactResult(control::ToolContactResult result)
  {
    std::lock_guard<std::mutex> lk(tool_contact_result_mutex_);
    tool_contact_result_.notify_one();
    received_result_ = result;
  }

private:
  std::condition_variable tool_contact_result_;
  std::mutex tool_contact_result_mutex_;
};

TEST_F(ScriptCommandInterfaceTest, test_zero_ft_sensor)
{
  // Wait for the client to connect to the server
  waitForClientConnection();

  script_command_interface_->zeroFTSensor();
  int32_t command;
  std::vector<int32_t> message;
  client_->readMessage(command, message);

  // 0 is zero ft sensor
  int32_t expected_command = 0;
  EXPECT_EQ(command, expected_command);

  // The rest of the message should be zero
  int32_t message_sum = std::accumulate(std::begin(message), std::end(message), 0);
  int32_t expected_message_sum = 0;
  EXPECT_EQ(message_sum, expected_message_sum);
}

TEST_F(ScriptCommandInterfaceTest, test_set_payload)
{
  // Wait for the client to connect to the server
  waitForClientConnection();

  double mass = 1.0;
  vector3d_t cog = { 0.2, 0.3, 0.1 };
  script_command_interface_->setPayload(mass, &cog);
  int32_t command;
  std::vector<int32_t> message;
  client_->readMessage(command, message);

  // 1 is set payload
  int32_t expected_command = 1;
  EXPECT_EQ(command, expected_command);

  // Test mass
  double received_mass = (double)message[0] / script_command_interface_->MULT_JOINTSTATE;
  EXPECT_EQ(received_mass, mass);

  // Test cog
  vector3d_t received_cog;
  for (unsigned int i = 0; i < cog.size(); ++i)
  {
    received_cog[i] = (double)message[i + 1] / script_command_interface_->MULT_JOINTSTATE;
    EXPECT_EQ(received_cog[i], cog[i]);
  }

  // The rest of the message should be zero
  int32_t message_sum = std::accumulate(std::begin(message) + 4, std::end(message), 0);
  int32_t expected_message_sum = 0;
  EXPECT_EQ(message_sum, expected_message_sum);
}

TEST_F(ScriptCommandInterfaceTest, test_set_tool_voltage)
{
  // Wait for the client to connect to the server
  waitForClientConnection();

  ToolVoltage tool_voltage = ToolVoltage::_12V;
  script_command_interface_->setToolVoltage(tool_voltage);
  int32_t command;
  std::vector<int32_t> message;
  client_->readMessage(command, message);

  // 2 is set tool voltage
  int32_t expected_command = 2;
  EXPECT_EQ(command, expected_command);

  // Test tool voltage
  int received_tool_voltage = message[0] / script_command_interface_->MULT_JOINTSTATE;
  EXPECT_EQ(received_tool_voltage, toUnderlying(tool_voltage));

  // The rest of the message should be zero
  int32_t message_sum = std::accumulate(std::begin(message) + 1, std::end(message), 0);
  int32_t expected_message_sum = 0;
  EXPECT_EQ(message_sum, expected_message_sum);
}

TEST_F(ScriptCommandInterfaceTest, test_force_mode)
{
  // Wait for the client to connect to the server
  waitForClientConnection();

  urcl::vector6d_t task_frame = { 0.1, 0, 0, 0, 0, 0.785 };
  urcl::vector6uint32_t selection_vector = { 0, 0, 1, 0, 0, 0 };
  urcl::vector6d_t wrench = { 20, 0, 40, 0, 0, 0 };
  int32_t force_mode_type = 2;
  urcl::vector6d_t limits = { 0.1, 0.1, 0.1, 0.785, 0.785, 1.57 };
  script_command_interface_->startForceMode(&task_frame, &selection_vector, &wrench, force_mode_type, &limits);

  int32_t command;
  std::vector<int32_t> message;
  client_->readMessage(command, message);

  // 3 is start force mode
  int32_t expected_command = 3;
  EXPECT_EQ(command, expected_command);

  // Test task frame
  vector6d_t received_task_frame;
  for (unsigned int i = 0; i < task_frame.size(); ++i)
  {
    received_task_frame[i] = (double)message[i] / script_command_interface_->MULT_JOINTSTATE;
    EXPECT_EQ(received_task_frame[i], task_frame[i]);
  }

  // Test selection vector
  vector6uint32_t received_selection_vector;
  for (unsigned int i = 0; i < selection_vector.size(); ++i)
  {
    received_selection_vector[i] = message[i + 6] / script_command_interface_->MULT_JOINTSTATE;
    EXPECT_EQ(received_selection_vector[i], selection_vector[i]);
  }

  // Test wrench
  vector6d_t received_wrench;
  for (unsigned int i = 0; i < wrench.size(); ++i)
  {
    received_wrench[i] = (double)message[i + 12] / script_command_interface_->MULT_JOINTSTATE;
    EXPECT_EQ(received_wrench[i], wrench[i]);
  }

  // Test force mode type
  int32_t received_force_mode_type = message[18] / script_command_interface_->MULT_JOINTSTATE;
  EXPECT_EQ(received_force_mode_type, force_mode_type);

  // Test limits
  vector6d_t received_limits;
  for (unsigned int i = 0; i < wrench.size(); ++i)
  {
    received_limits[i] = (double)message[i + 19] / script_command_interface_->MULT_JOINTSTATE;
    EXPECT_EQ(received_limits[i], limits[i]);
  }

  // The rest of the message should be zero
  int32_t message_sum = std::accumulate(std::begin(message) + 25, std::end(message), 0);
  int32_t expected_message_sum = 0;
  EXPECT_EQ(message_sum, expected_message_sum);

  // End force mode
  script_command_interface_->endForceMode();
  message.clear();
  client_->readMessage(command, message);

  // 4 is end force mode
  expected_command = 4;
  EXPECT_EQ(command, expected_command);

  // The rest of the message should be zero
  message_sum = std::accumulate(std::begin(message), std::end(message), 0);
  EXPECT_EQ(message_sum, expected_message_sum);
}

TEST_F(ScriptCommandInterfaceTest, test_tool_contact)
{
  // Wait for the client to connect to the server
  waitForClientConnection();

  script_command_interface_->startToolContact();

  int32_t command;
  std::vector<int32_t> message;
  client_->readMessage(command, message);

  // 5 is start tool contact
  int32_t expected_command = 5;
  EXPECT_EQ(command, expected_command);

  // The rest of the message should be zero
  int32_t message_sum = std::accumulate(std::begin(message) + 25, std::end(message), 0);
  int32_t expected_message_sum = 0;
  EXPECT_EQ(message_sum, expected_message_sum);

  // End tool contact
  script_command_interface_->endToolContact();

  message.clear();
  client_->readMessage(command, message);
  // 6 is end tool contact
  expected_command = 6;
  EXPECT_EQ(command, expected_command);

  // The rest of the message should be zero
  message_sum = std::accumulate(std::begin(message) + 25, std::end(message), 0);
  EXPECT_EQ(message_sum, expected_message_sum);
}

TEST_F(ScriptCommandInterfaceTest, test_tool_contact_callback)
{
  // Wait for the client to connect to the server
  waitForClientConnection();
  script_command_interface_->setToolContactResultCallback(
      std::bind(&ScriptCommandInterfaceTest::handleToolContactResult, this, std::placeholders::_1));

  control::ToolContactResult send_result = control::ToolContactResult::UNTIL_TOOL_CONTACT_RESULT_CANCELED;
  client_->send(toUnderlying(send_result));
  waitToolContactResult(send_result);

  EXPECT_EQ(toUnderlying(received_result_), toUnderlying(send_result));

  send_result = control::ToolContactResult::UNTIL_TOOL_CONTACT_RESULT_SUCCESS;
  client_->send(toUnderlying(send_result));
  waitToolContactResult(send_result);

  EXPECT_EQ(toUnderlying(received_result_), toUnderlying(send_result));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
