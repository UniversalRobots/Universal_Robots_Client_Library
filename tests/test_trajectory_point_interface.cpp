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

    void readMessage(vector6int32_t& pos, vector6int32_t& vel, vector6int32_t& acc, int32_t& goal_time,
                     int32_t& blend_radius_or_spline_type, int32_t& motion_type)
    {
      // Read message
      uint8_t buf[sizeof(int32_t) * 21];
      uint8_t* b_pos = buf;
      size_t read = 0;
      size_t remainder = sizeof(int32_t) * 21;
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

      // Decode positions
      int32_t val;
      b_pos = buf;
      for (unsigned int i = 0; i < pos.size(); ++i)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        pos[i] = be32toh(val);
        b_pos += sizeof(int32_t);
      }

      // Read velocity
      for (unsigned int i = 0; i < pos.size(); ++i)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        vel[i] = be32toh(val);
        b_pos += sizeof(int32_t);
      }

      // Read acceleration
      for (unsigned int i = 0; i < pos.size(); ++i)
      {
        std::memcpy(&val, b_pos, sizeof(int32_t));
        acc[i] = be32toh(val);
        b_pos += sizeof(int32_t);
      }

      // Decode goal time
      std::memcpy(&val, b_pos, sizeof(int32_t));
      goal_time = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode blend radius or spline type
      std::memcpy(&val, b_pos, sizeof(int32_t));
      blend_radius_or_spline_type = be32toh(val);
      b_pos += sizeof(int32_t);

      // Decode motion type
      std::memcpy(&val, b_pos, sizeof(int32_t));
      motion_type = be32toh(val);
    }

    vector6int32_t getPosition()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return pos;
    }

    vector6int32_t getVelocity()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return vel;
    }

    int32_t getGoalTime()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return goal_time;
    }

    int32_t getBlendRadius()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return blend_radius_or_spline_type;
    }

    int32_t getMotionType()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return motion_type;
    }

    struct TrajData
    {
      vector6int32_t pos, vel, acc;
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
    };

    TrajData getData()
    {
      TrajData spl;
      readMessage(spl.pos, spl.vel, spl.acc, spl.goal_time, spl.blend_radius_or_spline_type, spl.motion_type);
      return spl;
    }
  };

  void SetUp()
  {
    traj_point_interface_.reset(new control::TrajectoryPointInterface(50003));
    client_.reset(new Client(50003));
    // Need to be sure that the client has connected to the server
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  void TearDown()
  {
    if (client_->getState() == comm::SocketState::Connected)
    {
      client_->close();
    }
  }

  std::unique_ptr<control::TrajectoryPointInterface> traj_point_interface_;
  std::unique_ptr<Client> client_;

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
  urcl::vector6d_t send_positions = { 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, 0, 0);
  vector6int32_t received_positions = client_->getPosition();

  EXPECT_EQ(send_positions[0], ((double)received_positions[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[1], ((double)received_positions[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[2], ((double)received_positions[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[3], ((double)received_positions[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[4], ((double)received_positions[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_positions[5], ((double)received_positions[5]) / traj_point_interface_->MULT_JOINTSTATE);
}

TEST_F(TrajectoryPointInterfaceTest, write_quintic_joint_spline)
{
  urcl::vector6d_t send_pos = { 1.2, 3.1, 2.2, -1.4, -2.1, -3.2 };
  urcl::vector6d_t send_vel = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t send_acc = { 3.2, 1.1, 1.2, -3.4, -1.1, -1.2 };
  float send_goal_time = 0.5;
  traj_point_interface_->writeTrajectorySplinePoint(&send_pos, &send_vel, &send_acc, send_goal_time);
  Client::TrajData received_data = client_->getData();

  // Position
  EXPECT_EQ(send_pos[0], ((double)received_data.pos[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[1], ((double)received_data.pos[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[2], ((double)received_data.pos[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[3], ((double)received_data.pos[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[4], ((double)received_data.pos[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[5], ((double)received_data.pos[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Velocities
  EXPECT_EQ(send_vel[0], ((double)received_data.vel[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[1], ((double)received_data.vel[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[2], ((double)received_data.vel[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[3], ((double)received_data.vel[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[4], ((double)received_data.vel[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[5], ((double)received_data.vel[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Velocities
  EXPECT_EQ(send_acc[0], ((double)received_data.acc[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[1], ((double)received_data.acc[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[2], ((double)received_data.acc[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[3], ((double)received_data.acc[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[4], ((double)received_data.acc[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[5], ((double)received_data.acc[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Goal time
  EXPECT_EQ(send_goal_time, ((double)received_data.goal_time / traj_point_interface_->MULT_TIME));

  // Spline type
  EXPECT_EQ(static_cast<int32_t>(control::TrajectorySplineType::SPLINE_QUINTIC),
            received_data.blend_radius_or_spline_type);

  // Motion type
  EXPECT_EQ(static_cast<int32_t>(control::TrajectoryMotionType::JOINT_POINT_SPLINE), received_data.motion_type);
}

TEST_F(TrajectoryPointInterfaceTest, write_cubic_joint_spline)
{
  urcl::vector6d_t send_pos = { 1.2, 3.1, 2.2, -1.4, -2.1, -3.2 };
  urcl::vector6d_t send_vel = { 2.2, 2.1, 3.2, -2.4, -3.1, -2.3 };
  urcl::vector6d_t send_acc = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  float send_goal_time = 1.5;
  traj_point_interface_->writeTrajectorySplinePoint(&send_pos, &send_vel, nullptr, send_goal_time);
  Client::TrajData received_data = client_->getData();

  // Position
  EXPECT_EQ(send_pos[0], ((double)received_data.pos[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[1], ((double)received_data.pos[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[2], ((double)received_data.pos[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[3], ((double)received_data.pos[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[4], ((double)received_data.pos[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_pos[5], ((double)received_data.pos[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Velocities
  EXPECT_EQ(send_vel[0], ((double)received_data.vel[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[1], ((double)received_data.vel[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[2], ((double)received_data.vel[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[3], ((double)received_data.vel[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[4], ((double)received_data.vel[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[5], ((double)received_data.vel[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Velocities
  EXPECT_EQ(send_acc[0], ((double)received_data.acc[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[1], ((double)received_data.acc[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[2], ((double)received_data.acc[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[3], ((double)received_data.acc[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[4], ((double)received_data.acc[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_acc[5], ((double)received_data.acc[5]) / traj_point_interface_->MULT_JOINTSTATE);

  // Goal time
  EXPECT_EQ(send_goal_time, ((double)received_data.goal_time) / traj_point_interface_->MULT_TIME);

  // Spline type
  EXPECT_EQ(static_cast<int32_t>(control::TrajectorySplineType::SPLINE_CUBIC),
            received_data.blend_radius_or_spline_type);

  // Motion type
  EXPECT_EQ(static_cast<int32_t>(control::TrajectoryMotionType::JOINT_POINT_SPLINE), received_data.motion_type);
}

TEST_F(TrajectoryPointInterfaceTest, write_splines_velocities)
{
  urcl::vector6d_t send_pos = { 1.2, 3.1, 2.2, -1.4, -2.1, -3.2 };
  urcl::vector6d_t send_vel = { 2.2, 2.1, 3.2, -2.4, -3.1, -2.2 };
  urcl::vector6d_t send_acc = { 3.2, 1.1, 1.2, -3.4, -1.1, -1.2 };
  float send_goal_time = 0.5;
  traj_point_interface_->writeTrajectorySplinePoint(&send_pos, &send_vel, &send_acc, send_goal_time);
  vector6int32_t received_velocities = client_->getVelocity();

  EXPECT_EQ(send_vel[0], ((double)received_velocities[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[1], ((double)received_velocities[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[2], ((double)received_velocities[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[3], ((double)received_velocities[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[4], ((double)received_velocities[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[5], ((double)received_velocities[5]) / traj_point_interface_->MULT_JOINTSTATE);
}

TEST_F(TrajectoryPointInterfaceTest, write_splines_accelerations)
{
  urcl::vector6d_t send_pos = { 1.2, 3.1, 2.2, -1.4, -2.1, -3.2 };
  urcl::vector6d_t send_vel = { 2.2, 2.1, 3.2, -2.4, -3.1, -2.2 };
  urcl::vector6d_t send_acc = { 3.2, 1.1, 1.2, -3.4, -1.1, -1.2 };
  float send_goal_time = 0.5;
  traj_point_interface_->writeTrajectorySplinePoint(&send_pos, &send_vel, &send_acc, send_goal_time);
  vector6int32_t received_velocities = client_->getVelocity();

  EXPECT_EQ(send_vel[0], ((double)received_velocities[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[1], ((double)received_velocities[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[2], ((double)received_velocities[2]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[3], ((double)received_velocities[3]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[4], ((double)received_velocities[4]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_vel[5], ((double)received_velocities[5]) / traj_point_interface_->MULT_JOINTSTATE);
}

TEST_F(TrajectoryPointInterfaceTest, write_goal_time)
{
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_goal_time = 0.5;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, send_goal_time, 0, 0);
  int32_t received_goal_time = client_->getGoalTime();

  EXPECT_EQ(send_goal_time, ((float)received_goal_time) / traj_point_interface_->MULT_TIME);
}

TEST_F(TrajectoryPointInterfaceTest, write_blend_radius)
{
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_blend_radius = 0.5;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, send_blend_radius, 0);
  int32_t received_blend_radius = client_->getBlendRadius();

  EXPECT_EQ(send_blend_radius, ((float)received_blend_radius) / traj_point_interface_->MULT_TIME);
}

TEST_F(TrajectoryPointInterfaceTest, write_cartesian)
{
  // Write cartesian point
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  bool send_cartesian = true;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, 0, send_cartesian);
  bool received_cartesian = bool(client_->getMotionType());

  EXPECT_EQ(send_cartesian, received_cartesian);

  // Write joint point
  send_cartesian = false;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, 0, send_cartesian);
  received_cartesian = bool(client_->getMotionType());

  EXPECT_EQ(send_cartesian, received_cartesian);
}

TEST_F(TrajectoryPointInterfaceTest, trajectory_result)
{
  traj_point_interface_->setTrajectoryEndCallback(
      std::bind(&TrajectoryPointInterfaceTest::handleTrajectoryEnd, this, std::placeholders::_1));

  client_->send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_CANCELED));

  client_->send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_FAILURE));

  client_->send(toUnderlying(control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS));
  EXPECT_TRUE(waitTrajectoryEnd(1000, control::TrajectoryResult::TRAJECTORY_RESULT_SUCCESS));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
