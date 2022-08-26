// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include <gtest/gtest.h>
#include <ur_client_library/control/reverse_interface.h>
#include <ur_client_library/comm/tcp_socket.h>

using namespace urcl;

class ReverseIntefaceTest : public ::testing::Test
{
protected:
  class Client : public comm::TCPSocket  // maybe just use stream instead
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

    void readMessage(int32_t& keep_alive_signal, vector6int32_t& pos, int32_t& control_mode)
    {
      // Read message
      uint8_t buf[sizeof(int32_t) * 8];
      uint8_t* b_pos = buf;
      size_t read = 0;
      size_t remainder = sizeof(int32_t) * 8;
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

      // Decode keepalive signal
      int32_t val;
      b_pos = buf;
      std::memcpy(&val, b_pos, sizeof(int32_t));
      keep_alive_signal = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode positions
      for (unsigned int i = 0; i < pos.size(); ++i)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        pos[i] = be32toh(val);
        b_pos += sizeof(int32_t);
      }

      // Decode control mode
      std::memcpy(&val, b_pos, sizeof(int32_t));
      control_mode = be32toh(val);
    }

    // Helper functions to get different parts of the received message
    vector6int32_t getPositions()
    {
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return pos;
    }

    int32_t getKeepAliveCount()
    {
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return keep_alive_signal;
    }

    int32_t getControlMode()
    {
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return control_mode;
    }

    int32_t getTrajectoryControlMode()
    {
      // received_pos[0]=control::TrajectoryControlMessage, when writing a trajectory control message
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return pos[0];
    }

    int32_t getTrajectoryPointNumber()
    {
      // received_pos[1]=point_number, when writing a trajectory control message
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return pos[1];
    }

  protected:
    virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
    {
      return ::connect(socket_fd, address, address_len) == 0;
    }
  };

public:
  void handleProgramState(bool program_state)
  {
    std::lock_guard<std::mutex> lk(program_running_mutex_);
    program_running_.notify_one();
    program_state_ = program_state;
  }

  bool waitForProgramState(int milliseconds = 100, bool program_state = true)
  {
    std::unique_lock<std::mutex> lk(program_running_mutex_);
    if (program_running_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        program_state_ == program_state)
    {
      if (program_state_ == program_state)
      {
        return true;
      }
    }
    return false;
  }

private:
  std::atomic<bool> program_state_ = ATOMIC_VAR_INIT(false);
  std::condition_variable program_running_;
  std::mutex program_running_mutex_;
};

TEST_F(ReverseIntefaceTest, handle_program_state)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Test that handle program state is called when the client connects to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // Test that handle program state is called when the client disconnects from the server
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, write_positions)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  urcl::vector6d_t written_positions = { 1.2, -3.1, -2.2, -3.4, 1.1, 1.2 };
  reverse_interface.write(&written_positions);
  vector6int32_t received_positions = client.getPositions();

  EXPECT_EQ(written_positions[0], ((double)received_positions[0]) / reverse_interface.MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[1], ((double)received_positions[1]) / reverse_interface.MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[2], ((double)received_positions[2]) / reverse_interface.MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[3], ((double)received_positions[3]) / reverse_interface.MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[4], ((double)received_positions[4]) / reverse_interface.MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[5], ((double)received_positions[5]) / reverse_interface.MULT_JOINTSTATE);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, write_trajectory_control_message)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  control::TrajectoryControlMessage written_control_message = control::TrajectoryControlMessage::TRAJECTORY_CANCEL;
  reverse_interface.writeTrajectoryControlMessage(written_control_message, 1);
  int32_t received_control_message = client.getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);

  written_control_message = control::TrajectoryControlMessage::TRAJECTORY_NOOP;
  reverse_interface.writeTrajectoryControlMessage(written_control_message, 1);
  received_control_message = client.getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);

  written_control_message = control::TrajectoryControlMessage::TRAJECTORY_START;
  reverse_interface.writeTrajectoryControlMessage(written_control_message, 1);
  received_control_message = client.getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, write_trajectory_point_number)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  int32_t written_point_number = 2;
  reverse_interface.writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START,
                                                  written_point_number);
  int32_t received_point_number = client.getTrajectoryPointNumber();

  EXPECT_EQ(written_point_number, received_point_number);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, control_mode_is_forward)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // When writing trajectory control message, the control mode should always be mode forward
  comm::ControlMode expected_control_mode = comm::ControlMode::MODE_FORWARD;
  reverse_interface.writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  int32_t received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, remaining_message_points_are_zeros)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // When using trajectory control message, the received message is keep_alive_signal=keep_alive_signal,
  // received_pos[0]=control::TrajectoryControlMessage, received_pos[1]=point_number and received_pos[2]-received_pos[5]
  // should be zeros.
  reverse_interface.writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  vector6int32_t received_pos = client.getPositions();

  EXPECT_EQ(0, received_pos[2]);
  EXPECT_EQ(0, received_pos[3]);
  EXPECT_EQ(0, received_pos[4]);
  EXPECT_EQ(0, received_pos[5]);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, keep_alive_count)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  int expected_keep_alive_count = 5;
  reverse_interface.setKeepaliveCount(expected_keep_alive_count);

  urcl::vector6d_t pos = { 0, 0, 0, 0, 0, 0 };
  reverse_interface.write(&pos);
  int32_t received_keep_alive_count = client.getKeepAliveCount();

  EXPECT_EQ(expected_keep_alive_count, received_keep_alive_count);

  // Test that keep alive signal works with trajectory controll message as well
  reverse_interface.writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  received_keep_alive_count = client.getKeepAliveCount();

  EXPECT_EQ(expected_keep_alive_count, received_keep_alive_count);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, write_control_mode)
{
  control::ReverseInterface reverse_interface(
      50001, std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1));
  Client client(50001);

  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  vector6d_t pos = { 0, 0, 0, 0, 0, 0 };

  comm::ControlMode expected_control_mode = comm::ControlMode::MODE_FORWARD;
  reverse_interface.write(&pos, expected_control_mode);
  int32_t received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_IDLE;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_POSE;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SERVOJ;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SPEEDJ;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SPEEDL;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_STOPPED;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_UNINITIALIZED;
  reverse_interface.write(&pos, expected_control_mode);
  received_control_mode = client.getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  // Correctly shutdown the connection
  client.close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
