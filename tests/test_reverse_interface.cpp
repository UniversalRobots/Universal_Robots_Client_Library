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
#include <ur_client_library/control/reverse_interface.h>
#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/exceptions.h>
#include "ur_client_library/log.h"

using namespace urcl;
std::mutex g_connection_mutex;
std::condition_variable g_connection_condition;

class TestableReverseInterface : public control::ReverseInterface
{
public:
  TestableReverseInterface(const control::ReverseInterfaceConfig& config) : control::ReverseInterface(config)
  {
  }

  virtual void connectionCallback(const socket_t filedescriptor)
  {
    control::ReverseInterface::connectionCallback(filedescriptor);
    connected = true;
    std::lock_guard<std::mutex> lk(g_connection_mutex);
    g_connection_condition.notify_one();
  }

  virtual void disconnectionCallback(const socket_t filedescriptor)
  {
    URCL_LOG_DEBUG("There are %zu disconnection callbacks registered.", disconnect_callbacks_.size());
    control::ReverseInterface::disconnectionCallback(filedescriptor);
    connected = false;
    std::lock_guard<std::mutex> lk(g_connection_mutex);
    g_connection_condition.notify_one();
  }

  std::atomic<bool> connected = false;
};

class ReverseIntefaceTest : public ::testing::Test
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

    void readMessage(int32_t& read_timeout, vector6int32_t& pos, int32_t& control_mode)
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
      read_timeout = be32toh(val);
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

    int32_t getReadTimeout()
    {
      int32_t read_timeout;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(read_timeout, pos, control_mode);
      return read_timeout;
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

    int32_t getFreedriveControlMode()
    {
      // received_pos[0]=control::FreedriveControlMessage, when writing a trajectory control message
      int32_t keep_alive_signal;
      int32_t control_mode;
      vector6int32_t pos;
      readMessage(keep_alive_signal, pos, control_mode);
      return pos[0];
    }
  };

  void SetUp()
  {
    control::ReverseInterfaceConfig config;
    config.port = 50001;
    config.handle_program_state = std::bind(&ReverseIntefaceTest::handleProgramState, this, std::placeholders::_1);
    reverse_interface_.reset(new TestableReverseInterface(config));
    client_.reset(new Client(50001));
    std::unique_lock<std::mutex> lk(g_connection_mutex);
    g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                    [&]() { return reverse_interface_->connected.load(); });
  }

  void TearDown()
  {
    if (client_->getState() == comm::SocketState::Connected)
    {
      client_->close();
      waitForProgramState(1000, false);
    }
  }

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

  std::unique_ptr<TestableReverseInterface> reverse_interface_;
  std::unique_ptr<Client> client_;

private:
  std::atomic<bool> program_state_ = ATOMIC_VAR_INIT(false);
  std::condition_variable program_running_;
  std::mutex program_running_mutex_;
};

TEST_F(ReverseIntefaceTest, handle_program_state)
{
  // Test that handle program state is called when the client connects to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // Test that handle program state is called when the client disconnects from the server
  client_->close();
  EXPECT_TRUE(waitForProgramState(1000, false));
}

TEST_F(ReverseIntefaceTest, write_positions)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  urcl::vector6d_t written_positions = { 1.2, -3.1, -2.2, -3.4, 1.1, 1.2 };
  reverse_interface_->write(&written_positions);
  vector6int32_t received_positions = client_->getPositions();

  EXPECT_EQ(written_positions[0], ((double)received_positions[0]) / reverse_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[1], ((double)received_positions[1]) / reverse_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[2], ((double)received_positions[2]) / reverse_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[3], ((double)received_positions[3]) / reverse_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[4], ((double)received_positions[4]) / reverse_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(written_positions[5], ((double)received_positions[5]) / reverse_interface_->MULT_JOINTSTATE);
}

TEST_F(ReverseIntefaceTest, write_trajectory_control_message)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  control::TrajectoryControlMessage written_control_message = control::TrajectoryControlMessage::TRAJECTORY_CANCEL;
  reverse_interface_->writeTrajectoryControlMessage(written_control_message, 1);
  int32_t received_control_message = client_->getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);

  written_control_message = control::TrajectoryControlMessage::TRAJECTORY_NOOP;
  reverse_interface_->writeTrajectoryControlMessage(written_control_message, 1);
  received_control_message = client_->getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);

  written_control_message = control::TrajectoryControlMessage::TRAJECTORY_START;
  reverse_interface_->writeTrajectoryControlMessage(written_control_message, 1);
  received_control_message = client_->getTrajectoryControlMode();

  EXPECT_EQ(toUnderlying(written_control_message), received_control_message);
}

TEST_F(ReverseIntefaceTest, write_trajectory_point_number)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  int32_t written_point_number = 2;
  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START,
                                                    written_point_number);
  int32_t received_point_number = client_->getTrajectoryPointNumber();

  EXPECT_EQ(written_point_number, received_point_number);
}

TEST_F(ReverseIntefaceTest, control_mode_is_forward)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // When writing trajectory control message, the control mode should always be mode forward
  comm::ControlMode expected_control_mode = comm::ControlMode::MODE_FORWARD;
  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  int32_t received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);
}

TEST_F(ReverseIntefaceTest, remaining_message_points_are_zeros)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // When using trajectory control message, the received message is keep_alive_signal=keep_alive_signal,
  // received_pos[0]=control::TrajectoryControlMessage, received_pos[1]=point_number and received_pos[2]-received_pos[5]
  // should be zeros.
  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  vector6int32_t received_pos = client_->getPositions();

  EXPECT_EQ(0, received_pos[2]);
  EXPECT_EQ(0, received_pos[3]);
  EXPECT_EQ(0, received_pos[4]);
  EXPECT_EQ(0, received_pos[5]);
}

TEST_F(ReverseIntefaceTest, read_timeout)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  int32_t expected_read_timeout = 500;

  urcl::vector6d_t pos = { 0, 0, 0, 0, 0, 0 };
  reverse_interface_->write(&pos, comm::ControlMode::MODE_FORWARD,
                            RobotReceiveTimeout::millisec(expected_read_timeout));
  int32_t received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  // Test that read timeout works with trajectory control message as well
  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1,
                                                    RobotReceiveTimeout::millisec(expected_read_timeout));
  received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  // Test that read timeout works with free drive message as well
  reverse_interface_->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP,
                                                   RobotReceiveTimeout::millisec(expected_read_timeout));
  received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);
}

TEST_F(ReverseIntefaceTest, default_read_timeout)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  int32_t expected_read_timeout = 20;

  urcl::vector6d_t pos = { 0, 0, 0, 0, 0, 0 };
  reverse_interface_->write(&pos, comm::ControlMode::MODE_FORWARD);
  int32_t received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  // Test that read timeout works with trajectory control message as well
  expected_read_timeout = 200;
  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  // Test that read timeout works with free drive message as well
  reverse_interface_->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP);
  received_read_timeout = client_->getReadTimeout();

  EXPECT_EQ(expected_read_timeout, received_read_timeout);
}

TEST_F(ReverseIntefaceTest, write_control_mode)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  vector6d_t pos = { 0, 0, 0, 0, 0, 0 };

  comm::ControlMode expected_control_mode = comm::ControlMode::MODE_FORWARD;
  reverse_interface_->write(&pos, expected_control_mode);
  int32_t received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_IDLE;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_POSE;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SERVOJ;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SPEEDJ;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_SPEEDL;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_STOPPED;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);

  expected_control_mode = comm::ControlMode::MODE_UNINITIALIZED;
  EXPECT_THROW(reverse_interface_->write(&pos, expected_control_mode), UrException);

  expected_control_mode = comm::ControlMode::MODE_TORQUE;
  reverse_interface_->write(&pos, expected_control_mode);
  received_control_mode = client_->getControlMode();

  EXPECT_EQ(toUnderlying(expected_control_mode), received_control_mode);
}

TEST_F(ReverseIntefaceTest, write_freedrive_control_message)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  control::FreedriveControlMessage written_freedrive_message = control::FreedriveControlMessage::FREEDRIVE_STOP;
  reverse_interface_->writeFreedriveControlMessage(written_freedrive_message);
  int32_t received_freedrive_message = client_->getFreedriveControlMode();

  EXPECT_EQ(toUnderlying(written_freedrive_message), received_freedrive_message);

  written_freedrive_message = control::FreedriveControlMessage::FREEDRIVE_NOOP;
  reverse_interface_->writeFreedriveControlMessage(written_freedrive_message);
  received_freedrive_message = client_->getFreedriveControlMode();

  EXPECT_EQ(toUnderlying(written_freedrive_message), received_freedrive_message);

  written_freedrive_message = control::FreedriveControlMessage::FREEDRIVE_START;
  reverse_interface_->writeFreedriveControlMessage(written_freedrive_message);
  received_freedrive_message = client_->getFreedriveControlMode();

  EXPECT_EQ(toUnderlying(written_freedrive_message), received_freedrive_message);
}

TEST_F(ReverseIntefaceTest, deprecated_set_keep_alive_count)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  // Test that it works to set the keepalive count using the deprecated function
  int keep_alive_count = 10;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  reverse_interface_->setKeepaliveCount(keep_alive_count);
#pragma GCC diagnostic pop
  int32_t expected_read_timeout = 20 * keep_alive_count;

  urcl::vector6d_t pos = { 0, 0, 0, 0, 0, 0 };
  reverse_interface_->write(&pos, comm::ControlMode::MODE_FORWARD);
  int32_t received_read_timeout = client_->getReadTimeout();
  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  reverse_interface_->writeTrajectoryControlMessage(control::TrajectoryControlMessage::TRAJECTORY_START, 1);
  received_read_timeout = client_->getReadTimeout();
  EXPECT_EQ(expected_read_timeout, received_read_timeout);

  reverse_interface_->writeFreedriveControlMessage(control::FreedriveControlMessage::FREEDRIVE_STOP);
  received_read_timeout = client_->getReadTimeout();
  EXPECT_EQ(expected_read_timeout, received_read_timeout);
}

TEST_F(ReverseIntefaceTest, disconnected_callbacks_are_called)
{
  // Wait for the client to connect to the server
  EXPECT_TRUE(waitForProgramState(1000, true));

  std::atomic<bool> disconnect_called_1 = false;
  std::atomic<bool> disconnect_called_2 = false;

  // Register disconnection callbacks
  int disconnection_callback_id_1 =
      reverse_interface_->registerDisconnectionCallback([&disconnect_called_1](const int fd) {
        std::cout << "Disconnection 1 callback called with fd: " << fd << std::endl;
        disconnect_called_1 = true;
      });
  int disconnection_callback_id_2 =
      reverse_interface_->registerDisconnectionCallback([&disconnect_called_2](const int fd) {
        std::cout << "Disconnection 2 callback called with fd: " << fd << std::endl;
        disconnect_called_2 = true;
      });

  // Close the client connection
  client_->close();
  EXPECT_TRUE(waitForProgramState(1000, false));
  std::unique_lock<std::mutex> lk(g_connection_mutex);
  g_connection_condition.wait_for(lk, std::chrono::seconds(1), [&]() { return !reverse_interface_->connected.load(); });
  EXPECT_TRUE(disconnect_called_1);
  EXPECT_TRUE(disconnect_called_2);

  // Unregister 1. 2 should still be called
  disconnect_called_1 = false;
  disconnect_called_2 = false;
  client_.reset(new Client(50001));
  EXPECT_TRUE(waitForProgramState(1000, true));
  reverse_interface_->unregisterDisconnectionCallback(disconnection_callback_id_1);
  client_->close();
  g_connection_condition.wait_for(lk, std::chrono::seconds(1), [&]() { return !reverse_interface_->connected.load(); });
  EXPECT_TRUE(waitForProgramState(1000, false));
  EXPECT_FALSE(disconnect_called_1);
  EXPECT_TRUE(disconnect_called_2);

  // Unregister both. None should be called
  disconnect_called_1 = false;
  disconnect_called_2 = false;
  client_.reset(new Client(50001));
  EXPECT_TRUE(waitForProgramState(1000, true));
  reverse_interface_->unregisterDisconnectionCallback(disconnection_callback_id_2);
  client_->close();
  g_connection_condition.wait_for(lk, std::chrono::seconds(1), [&]() { return !reverse_interface_->connected.load(); });
  EXPECT_TRUE(waitForProgramState(1000, false));
  EXPECT_FALSE(disconnect_called_1);
  EXPECT_FALSE(disconnect_called_2);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  urcl::setLogLevel(LogLevel::INFO);

  return RUN_ALL_TESTS();
}
