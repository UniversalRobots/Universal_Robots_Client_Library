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
#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/comm/tcp_socket.h>

using namespace urcl;

class TrajectoryPointInterfaceTest : public ::testing::Test
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

    void readMessage(vector6int32_t& pos, int32_t& goal_time, int32_t& blend_radius, int32_t& cartesian)
    {
      // Read message
      uint8_t buf[sizeof(int32_t) * 9];
      uint8_t* b_pos = buf;
      size_t read = 0;
      size_t remainder = sizeof(int32_t) * 9;
      while (remainder > 0)
      {
        TCPSocket::setOptions(getSocketFD());
        if (!TCPSocket::read(b_pos, remainder, read))
        {
          std::cout << "Failed to read from socket, this should not happen during a test!" << std::endl;
          break;
        }
        b_pos += read;
        remainder -= read;
      }

      // Decode positions
      int32_t val;
      b_pos = buf;
      for (unsigned int i = 0; i < pos.size(); ++i)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        pos[i] = be32toh(val);
        b_pos += sizeof(int32_t);
      }

      // Decode goal time
      std::memcpy(&val, b_pos, sizeof(int32_t));
      goal_time = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode blend radius
      std::memcpy(&val, b_pos, sizeof(int32_t));
      blend_radius = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode cartesian
      std::memcpy(&val, b_pos, sizeof(int32_t));
      cartesian = be32toh(val);
    }

    vector6int32_t getPosition()
    {
      int32_t goal_time, blend_radius, cartesian;
      vector6int32_t pos;
      readMessage(pos, goal_time, blend_radius, cartesian);
      return pos;
    }

    int32_t getGoalTime()
    {
      int32_t goal_time, blend_radius, cartesian;
      vector6int32_t pos;
      readMessage(pos, goal_time, blend_radius, cartesian);
      return goal_time;
    }

    int32_t getBlendRadius()
    {
      int32_t goal_time, blend_radius, cartesian;
      vector6int32_t pos;
      readMessage(pos, goal_time, blend_radius, cartesian);
      return blend_radius;
    }

    int32_t getCartesian()
    {
      int32_t goal_time, blend_radius, cartesian;
      vector6int32_t pos;
      readMessage(pos, goal_time, blend_radius, cartesian);
      return cartesian;
    }

  protected:
    virtual bool open(int socket_fd, struct sockaddr* address, size_t address_len)
    {
      return ::connect(socket_fd, address, address_len) == 0;
    }
  };

public:
  void handleTrajectoryEnd(control::TrajectoryResult result)
  {
    std::lock_guard<std::mutex> lk(trajectory_end_mutex_);
    trajectory_end_.notify_one();
    result_ = result;
  }

  bool waitTrajectoryEnd(int milliseconds = 100,
                         control::TrajectoryResult result = control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED)
  {
    std::unique_lock<std::mutex> lk(trajectory_end_mutex_);
    if (trajectory_end_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        result_ == result)
    {
      if (result_ == result)
      {
        return true;
      }
    }
    return false;
  }

private:
  std::condition_variable trajectory_end_;
  std::mutex trajectory_end_mutex_;
  control::TrajectoryResult result_;
};

TEST_F(TrajectoryPointInterfaceTest, write_postions)
{
  control::TrajectoryPointInterface traj_point_interface(50002);
  Client client(50002);

  // Need to be sure that the client has connected to the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  urcl::vector6d_t send_positions = { 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  traj_point_interface.writeTrajectoryPoint(&send_positions, 0, 0, 0);
  vector6int32_t received_positions = client.getPosition();

  EXPECT_EQ(send_positions[0], ((double)received_positions[0]) / traj_point_interface.MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[1], ((double)received_positions[1]) / traj_point_interface.MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[2], ((double)received_positions[2]) / traj_point_interface.MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[3], ((double)received_positions[3]) / traj_point_interface.MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[4], ((double)received_positions[4]) / traj_point_interface.MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[5], ((double)received_positions[5]) / traj_point_interface.MULT_JOINTSTATE);
}

TEST_F(TrajectoryPointInterfaceTest, write_goal_time)
{
  control::TrajectoryPointInterface traj_point_interface(50002);
  Client client(50002);

  // Need to be sure that the client has connected to the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_goal_time = 0.5;
  traj_point_interface.writeTrajectoryPoint(&send_positions, send_goal_time, 0, 0);
  int32_t received_goal_time = client.getGoalTime();

  EXPECT_EQ(send_goal_time, ((float)received_goal_time) / traj_point_interface.MULT_TIME);
}

TEST_F(TrajectoryPointInterfaceTest, write_blend_radius)
{
  control::TrajectoryPointInterface traj_point_interface(50002);
  Client client(50002);

  // Need to be sure that the client has connected to the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_blend_radius = 0.5;
  traj_point_interface.writeTrajectoryPoint(&send_positions, 0, send_blend_radius, 0);
  int32_t received_blend_radius = client.getBlendRadius();

  EXPECT_EQ(send_blend_radius, ((float)received_blend_radius) / traj_point_interface.MULT_TIME);
}

TEST_F(TrajectoryPointInterfaceTest, write_cartesian)
{
  control::TrajectoryPointInterface traj_point_interface(50002);
  Client client(50002);

  // Need to be sure that the client has connected to the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Write cartesian point
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  bool send_cartesian = true;
  traj_point_interface.writeTrajectoryPoint(&send_positions, 0, 0, send_cartesian);
  bool received_cartesian = bool(client.getCartesian());

  EXPECT_EQ(send_cartesian, received_cartesian);

  // Write joint point
  send_cartesian = false;
  traj_point_interface.writeTrajectoryPoint(&send_positions, 0, 0, send_cartesian);
  received_cartesian = bool(client.getCartesian());

  EXPECT_EQ(send_cartesian, received_cartesian);
}

TEST_F(TrajectoryPointInterfaceTest, trajectory_result)
{
  control::TrajectoryPointInterface traj_point_interface(50002);
  traj_point_interface.setTrajectoryEndCallback(
      std::bind(&TrajectoryPointInterfaceTest::handleTrajectoryEnd, this, std::placeholders::_1));
  Client client(50002);

  // Need to make sure that the client has connected to the server
  std::this_thread::sleep_for(std::chrono::seconds(1));

  client.send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED));

  client.send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE));

  client.send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
