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
#include <urcl_3rdparty/portable_endian.h>
#include <ur_client_library/control/trajectory_point_interface.h>
#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/control/motion_primitives.h>
#include <cmath>
#include <limits>
#include <variant>
#include "ur_client_library/exceptions.h"

using namespace urcl;

std::mutex g_connection_mutex;
std::condition_variable g_connection_condition;

class TestableTrajectoryPointInterface : public control::TrajectoryPointInterface
{
public:
  TestableTrajectoryPointInterface(uint32_t port) : control::TrajectoryPointInterface(port)
  {
  }

  virtual void connectionCallback(const socket_t filedescriptor) override
  {
    control::TrajectoryPointInterface::connectionCallback(filedescriptor);
    connected = true;
    std::lock_guard<std::mutex> lk(g_connection_mutex);
    g_connection_condition.notify_one();
  }

  virtual void disconnectionCallback(const socket_t filedescriptor) override
  {
    URCL_LOG_DEBUG("There are %zu disconnection callbacks registered.", disconnect_callbacks_.size());
    control::TrajectoryPointInterface::disconnectionCallback(filedescriptor);
    connected = false;
    std::lock_guard<std::mutex> lk(g_connection_mutex);
    g_connection_condition.notify_one();
  }

  std::atomic<bool> connected = false;  //!< True, if the interface is connected to the robot.
};

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
      uint8_t buf[sizeof(int32_t) * urcl::control::TrajectoryPointInterface::MESSAGE_LENGTH];
      uint8_t* b_pos = buf;
      size_t read = 0;
      size_t remainder = sizeof(int32_t) * urcl::control::TrajectoryPointInterface::MESSAGE_LENGTH;
      while (remainder > 0)
      {
        if (!TCPSocket::read(b_pos, remainder, read))
        {
          throw(std::runtime_error("Failed to read from socket, this should not happen during a test!"));
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

    vector6int32_t getAcceleration()
    {
      int32_t goal_time, blend_radius_or_spline_type, motion_type;
      vector6int32_t pos, vel, acc;
      readMessage(pos, vel, acc, goal_time, blend_radius_or_spline_type, motion_type);
      return acc;
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

    static double toDouble(int32_t raw)
    {
      return static_cast<double>(raw) / control::TrajectoryPointInterface::MULT_JOINTSTATE;
    }

    static vector6d_t toVector(const vector6int32_t& raw)
    {
      return vector6d_t{ toDouble(raw[0]), toDouble(raw[1]), toDouble(raw[2]),
                         toDouble(raw[3]), toDouble(raw[4]), toDouble(raw[5]) };
    }

    static urcl::Pose toPose(const vector6int32_t& raw)
    {
      return urcl::Pose{ toDouble(raw[0]), toDouble(raw[1]), toDouble(raw[2]),
                         toDouble(raw[3]), toDouble(raw[4]), toDouble(raw[5]) };
    }

    /*! \brief Decode Cartesian pose from block 0; if block 2 index 2 is non-zero, attach ``q_near`` from block 1. */
    static urcl::Pose poseWithOptionalQNear(const vector6int32_t& pose_raw, const vector6int32_t& second_raw,
                                            const vector6int32_t& third_raw)
    {
      urcl::Pose pose = toPose(pose_raw);
      if (toDouble(third_raw[2]) != 0.0)
      {
        pose.q_near = urcl::Q{ toDouble(second_raw[0]), toDouble(second_raw[1]), toDouble(second_raw[2]),
                               toDouble(second_raw[3]), toDouble(second_raw[4]), toDouble(second_raw[5]) };
      }
      return pose;
    }

    std::shared_ptr<control::MotionPrimitive> getMotionPrimitive()
    {
      TrajData spl = getData();
      const double blend_radius = toDouble(spl.blend_radius_or_spline_type);
      // Third block (decoded into ``spl.acc``) is [velocity, acceleration, …] for non-spline moves.
      const double velocity = toDouble(spl.acc[0]);
      const double acceleration = toDouble(spl.acc[1]);
      const auto duration = std::chrono::microseconds(spl.goal_time);

      if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEJ))
      {
        return std::make_shared<control::MoveJPrimitive>(toVector(spl.pos), blend_radius, duration, acceleration,
                                                         velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEJ_POSE))
      {
        return std::make_shared<control::MoveJPrimitive>(
            urcl::MotionTarget{ poseWithOptionalQNear(spl.pos, spl.vel, spl.acc) }, blend_radius, duration,
            acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEL))
      {
        return std::make_shared<control::MoveLPrimitive>(poseWithOptionalQNear(spl.pos, spl.vel, spl.acc), blend_radius,
                                                         duration, acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEL_JOINT))
      {
        const auto values = toVector(spl.pos);
        return std::make_shared<control::MoveLPrimitive>(
            urcl::MotionTarget{ urcl::Q{ values[0], values[1], values[2], values[3], values[4], values[5] } },
            blend_radius, duration, acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEP))
      {
        return std::make_shared<control::MovePPrimitive>(poseWithOptionalQNear(spl.pos, spl.vel, spl.acc), blend_radius,
                                                         acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEP_JOINT))
      {
        const auto values = toVector(spl.pos);
        return std::make_shared<control::MovePPrimitive>(
            urcl::MotionTarget{ urcl::Q{ values[0], values[1], values[2], values[3], values[4], values[5] } },
            blend_radius, acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC) ||
               spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC_JOINT) ||
               spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC_POSE_JOINT) ||
               spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC_JOINT_POSE))
      {
        // For movec the third block holds [velocity, acceleration, mode, 0, 0, 0]
        const double movec_velocity = toDouble(spl.acc[0]);
        const double movec_acceleration = toDouble(spl.acc[1]);
        const int32_t mode = static_cast<int32_t>(
            round(static_cast<double>(spl.acc[2]) / control::TrajectoryPointInterface::MULT_JOINTSTATE));

        auto make_target = [&](bool is_pose, const vector6int32_t& raw) -> urcl::MotionTarget {
          if (is_pose)
          {
            return urcl::MotionTarget{ toPose(raw) };
          }
          const auto values = toVector(raw);
          return urcl::MotionTarget{ urcl::Q{ values[0], values[1], values[2], values[3], values[4], values[5] } };
        };

        bool target_is_pose = spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC) ||
                              spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC_POSE_JOINT);
        bool via_is_pose = spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC) ||
                           spl.motion_type == static_cast<int32_t>(control::MotionType::MOVEC_JOINT_POSE);

        return std::make_shared<control::MoveCPrimitive>(make_target(via_is_pose, spl.vel),
                                                         make_target(target_is_pose, spl.pos), blend_radius,
                                                         movec_acceleration, movec_velocity, mode);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::OPTIMOVEJ))
      {
        return std::make_shared<control::OptimoveJPrimitive>(toVector(spl.pos), blend_radius, acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::OPTIMOVEJ_POSE))
      {
        return std::make_shared<control::OptimoveJPrimitive>(
            urcl::MotionTarget{ poseWithOptionalQNear(spl.pos, spl.vel, spl.acc) }, blend_radius, acceleration,
            velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::OPTIMOVEL))
      {
        return std::make_shared<control::OptimoveLPrimitive>(poseWithOptionalQNear(spl.pos, spl.vel, spl.acc),
                                                             blend_radius, acceleration, velocity);
      }
      else if (spl.motion_type == static_cast<int32_t>(control::MotionType::OPTIMOVEL_JOINT))
      {
        const auto values = toVector(spl.pos);
        return std::make_shared<control::OptimoveLPrimitive>(
            urcl::MotionTarget{ urcl::Q{ values[0], values[1], values[2], values[3], values[4], values[5] } },
            blend_radius, acceleration, velocity);
      }
      else
      {
        throw std::runtime_error("Unknown motion type");
      }
    }
  };

  void SetUp()
  {
    traj_point_interface_.reset(new TestableTrajectoryPointInterface(50003));
    client_.reset(new Client(50003));
    // Need to be sure that the client has connected to the server
    std::unique_lock<std::mutex> lk(g_connection_mutex);
    g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                    [&]() { return traj_point_interface_->connected.load(); });
  }

  void TearDown()
  {
    if (client_->getState() == comm::SocketState::Connected)
    {
      client_->close();
    }
  }

  std::unique_ptr<TestableTrajectoryPointInterface> traj_point_interface_;
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
  control::TrajectoryResult result_ = control::TrajectoryResult::TRAJECTORY_RESULT_UNKNOWN;
};

TEST_F(TrajectoryPointInterfaceTest, write_postions)
{
  urcl::vector6d_t send_positions = { 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, 0, false);
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

  // Goal time (segment duration, ``MULT_TIME`` on the wire)
  EXPECT_EQ(send_goal_time, ((double)received_data.goal_time / traj_point_interface_->MULT_TIME));

  // Spline type
  EXPECT_EQ(static_cast<int32_t>(control::TrajectorySplineType::SPLINE_QUINTIC),
            received_data.blend_radius_or_spline_type);

  // Motion type
  EXPECT_EQ(static_cast<int32_t>(control::MotionType::SPLINE), received_data.motion_type);
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
  EXPECT_EQ(static_cast<int32_t>(control::MotionType::SPLINE), received_data.motion_type);
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
  traj_point_interface_->writeTrajectoryPoint(&send_positions, send_goal_time, 0, false);
  int32_t received_goal_time = client_->getGoalTime();

  EXPECT_EQ(send_goal_time, ((float)received_goal_time) / traj_point_interface_->MULT_TIME);
}

// Wire format: int32 microseconds (MULT_TIME). Duration must reach writeMotionPrimitive as seconds
// in double form so encoding uses round(seconds * MULT_TIME), not trunc(int(ms)).
TEST_F(TrajectoryPointInterfaceTest, write_goal_time_preserves_submillisecond_precision)
{
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  const float send_goal_time = 0.5006f;
  const int32_t expected_encoded = static_cast<int32_t>(
      std::round(static_cast<double>(send_goal_time) * static_cast<double>(traj_point_interface_->MULT_TIME)));

  traj_point_interface_->writeTrajectoryPoint(&send_positions, send_goal_time, 0, false);
  EXPECT_EQ(expected_encoded, client_->getGoalTime());

  traj_point_interface_->writeTrajectoryPoint(&send_positions, 1.4f, 1.05f, send_goal_time, 0, false);
  EXPECT_EQ(expected_encoded, client_->getGoalTime());

  // High-precision duration: must match round(goal_time * MULT_TIME), not trunc(goal_time * MULT_TIME).
  const float precise_goal_time = 1.234567f;
  const int32_t expected_precise = static_cast<int32_t>(
      std::round(static_cast<double>(precise_goal_time) * static_cast<double>(traj_point_interface_->MULT_TIME)));
  traj_point_interface_->writeTrajectoryPoint(&send_positions, precise_goal_time, 0, false);
  EXPECT_EQ(expected_precise, client_->getGoalTime());
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 1.4f, 1.05f, precise_goal_time, 0, false);
  EXPECT_EQ(expected_precise, client_->getGoalTime());

  urcl::vector6d_t send_vel = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t send_acc = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  traj_point_interface_->writeTrajectorySplinePoint(&send_positions, &send_vel, &send_acc, send_goal_time);
  EXPECT_EQ(expected_encoded, client_->getGoalTime());
  traj_point_interface_->writeTrajectorySplinePoint(&send_positions, &send_vel, &send_acc, precise_goal_time);
  EXPECT_EQ(expected_precise, client_->getGoalTime());
}

// Segment duration is capped by int32 microseconds on the wire (~35.79 minutes).
TEST_F(TrajectoryPointInterfaceTest, write_rejects_goal_time_above_max_encodable_duration)
{
  const double max_goal_time_seconds = static_cast<double>(std::numeric_limits<int32_t>::max()) /
                                       static_cast<double>(urcl::control::TrajectoryPointInterface::MULT_TIME);
  const float too_long_goal_time = static_cast<float>(max_goal_time_seconds + 1.0);
  const float almost_too_long_goal_time = static_cast<float>(max_goal_time_seconds - 1.0);

  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  EXPECT_FALSE(traj_point_interface_->writeTrajectoryPoint(&send_positions, too_long_goal_time, 0, false));
  EXPECT_FALSE(traj_point_interface_->writeTrajectoryPoint(&send_positions, 1.4f, 1.05f, too_long_goal_time, 0, false));
  EXPECT_TRUE(traj_point_interface_->writeTrajectoryPoint(&send_positions, almost_too_long_goal_time, 0, false));
  EXPECT_TRUE(
      traj_point_interface_->writeTrajectoryPoint(&send_positions, 1.4f, 1.05f, almost_too_long_goal_time, 0, false));

  urcl::vector6d_t send_vel = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  urcl::vector6d_t send_acc = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  EXPECT_FALSE(
      traj_point_interface_->writeTrajectorySplinePoint(&send_positions, &send_vel, &send_acc, too_long_goal_time));
  EXPECT_TRUE(traj_point_interface_->writeTrajectorySplinePoint(&send_positions, &send_vel, &send_acc,
                                                                almost_too_long_goal_time));
}

TEST_F(TrajectoryPointInterfaceTest, write_acceleration_velocity)
{
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_move_acceleration = 0.123f;
  float send_move_velocity = 0.456f;
  float send_goal_time = 0.5;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, send_move_acceleration, send_move_velocity,
                                              send_goal_time, 0, 0);
  Client::TrajData d = client_->getData();

  EXPECT_EQ(send_move_velocity, ((float)d.acc[0]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_move_acceleration, ((float)d.acc[1]) / traj_point_interface_->MULT_JOINTSTATE);
  EXPECT_EQ(send_goal_time, ((float)d.goal_time) / traj_point_interface_->MULT_TIME);
}

TEST_F(TrajectoryPointInterfaceTest, write_blend_radius)
{
  urcl::vector6d_t send_positions = { 0, 0, 0, 0, 0, 0 };
  float send_blend_radius = 0.5;
  traj_point_interface_->writeTrajectoryPoint(&send_positions, 0, send_blend_radius, false);
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

TEST_F(TrajectoryPointInterfaceTest, send_movej)
{
  urcl::vector6d_t send_positions = { 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(434);
  auto primitive =
      std::make_shared<control::MoveJPrimitive>(send_positions, blend_radius, duration, acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEJ);
  EXPECT_EQ(std::get<urcl::Q>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            send_positions);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->duration, duration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movel)
{
  urcl::Pose send_positions(1.2, 3.1, 2.2, -3.4, -1.1, -1.2);
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(434);
  auto primitive =
      std::make_shared<control::MoveLPrimitive>(send_positions, blend_radius, duration, acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEL);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            send_positions);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->duration, duration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movep)
{
  urcl::Pose send_positions(1.2, 3.1, 2.2, -3.4, -1.1, -1.2);
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto primitive = std::make_shared<control::MovePPrimitive>(send_positions, blend_radius, acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEP);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            send_positions);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movec)
{
  urcl::Pose send_target(1.2, 3.1, 2.2, -3.4, -1.1, -1.2);
  urcl::Pose send_via(0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
  double blend_radius = 0.5;
  double acceleration = 0.4;
  double velocity = 0.7;
  int32_t mode = 1;
  auto primitive =
      std::make_shared<control::MoveCPrimitive>(send_via, send_target, blend_radius, acceleration, velocity, mode);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEC);
  auto received_movec = std::static_pointer_cast<control::MoveCPrimitive>(received_primitive);
  EXPECT_EQ(std::get<urcl::Pose>(received_movec->getTarget().value()), send_target);
  EXPECT_EQ(std::get<urcl::Pose>(received_movec->getVia()), send_via);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_movec->mode, mode);
}

TEST_F(TrajectoryPointInterfaceTest, send_movej_pose)
{
  const urcl::Pose expected_pose(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);
  double blend_radius = 0.25;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(500);
  auto primitive = std::make_shared<control::MoveJPrimitive>(
      urcl::MotionTarget{ std::in_place_type<urcl::Pose>, 0.1, -0.2, 0.3, 0.4, -0.5, 0.6 }, blend_radius, duration,
      acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEJ_POSE);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            expected_pose);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->duration, duration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movej_pose_with_q_near_roundtrip)
{
  urcl::Pose expected_pose(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);
  expected_pose.q_near = urcl::Q{ -1.0, -2.0, 1.0, 0.5, 0.25, 0.0 };
  double blend_radius = 0.25;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(500);
  urcl::MotionTarget target{ std::in_place_type<urcl::Pose>, 0.1, -0.2, 0.3, 0.4, -0.5, 0.6 };
  std::get<urcl::Pose>(target).q_near = urcl::Q{ -1.0, -2.0, 1.0, 0.5, 0.25, 0.0 };
  auto primitive =
      std::make_shared<control::MoveJPrimitive>(std::move(target), blend_radius, duration, acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEJ_POSE);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            expected_pose);
}

TEST_F(TrajectoryPointInterfaceTest, send_movel_with_q_near_roundtrip)
{
  urcl::Pose send_positions(1.2, 3.1, 2.2, -3.4, -1.1, -1.2);
  send_positions.q_near = urcl::Q{ 0.11, -0.22, 0.33, 0.44, -0.55, 0.66 };
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(434);
  auto primitive =
      std::make_shared<control::MoveLPrimitive>(send_positions, blend_radius, duration, acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEL);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            send_positions);
}

TEST_F(TrajectoryPointInterfaceTest, send_optimovej_pose_with_q_near_roundtrip)
{
  const urcl::Pose expected_pose(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);
  double blend_radius = 0.1;
  double acceleration_fraction = 0.4;
  double velocity_fraction = 0.6;
  urcl::MotionTarget target{ std::in_place_type<urcl::Pose>, 0.1, -0.2, 0.3, 0.4, -0.5, 0.6 };
  std::get<urcl::Pose>(target).q_near = urcl::Q{ 0.01, 0.02, -0.03, 0.04, -0.05, 0.06 };
  auto primitive = std::make_shared<control::OptimoveJPrimitive>(std::move(target), blend_radius, acceleration_fraction,
                                                                 velocity_fraction);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::OPTIMOVEJ_POSE);
  urcl::Pose expected_with_q = expected_pose;
  expected_with_q.q_near = urcl::Q{ 0.01, 0.02, -0.03, 0.04, -0.05, 0.06 };
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            expected_with_q);
}

TEST_F(TrajectoryPointInterfaceTest, send_movel_joint)
{
  urcl::Q send_joints{ 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto duration = std::chrono::milliseconds(434);
  auto primitive = std::make_shared<control::MoveLPrimitive>(urcl::MotionTarget{ send_joints }, blend_radius, duration,
                                                             acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEL_JOINT);
  EXPECT_EQ(std::get<urcl::Q>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value())
                .values,
            send_joints.values);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->duration, duration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movep_joint)
{
  urcl::Q send_joints{ 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.5;
  double velocity = 0.6;
  double acceleration = 0.7;
  auto primitive = std::make_shared<control::MovePPrimitive>(urcl::MotionTarget{ send_joints }, blend_radius,
                                                             acceleration, velocity);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEP_JOINT);
  EXPECT_EQ(std::get<urcl::Q>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value())
                .values,
            send_joints.values);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
}

TEST_F(TrajectoryPointInterfaceTest, send_movec_joint_joint)
{
  urcl::Q send_via{ 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 };
  urcl::Q send_target{ 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.5;
  double acceleration = 0.7;
  double velocity = 0.7;
  int32_t mode = 1;
  auto primitive = std::make_shared<control::MoveCPrimitive>(
      urcl::MotionTarget{ send_via }, urcl::MotionTarget{ send_target }, blend_radius, acceleration, velocity, mode);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEC_JOINT);
  auto movec = std::static_pointer_cast<control::MoveCPrimitive>(received_primitive);
  EXPECT_EQ(std::get<urcl::Q>(movec->getTarget().value()).values, send_target.values);
  EXPECT_EQ(std::get<urcl::Q>(movec->getVia()).values, send_via.values);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(movec->mode, mode);
}

TEST_F(TrajectoryPointInterfaceTest, send_movec_pose_joint)
{
  // via is a Pose, target is a Q
  urcl::Pose send_via(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);
  urcl::Q send_target{ 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.25;
  double acceleration = 0.7;
  double velocity = 0.5;
  int32_t mode = 0;
  auto primitive = std::make_shared<control::MoveCPrimitive>(
      urcl::MotionTarget{ send_via }, urcl::MotionTarget{ send_target }, blend_radius, acceleration, velocity, mode);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  // The via is a pose, the target is a joint configuration -> MOVEC_JOINT_POSE
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEC_JOINT_POSE);
  auto movec = std::static_pointer_cast<control::MoveCPrimitive>(received_primitive);
  EXPECT_EQ(std::get<urcl::Pose>(movec->getVia()), send_via);
  EXPECT_EQ(std::get<urcl::Q>(movec->getTarget().value()).values, send_target.values);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(movec->mode, mode);
}

TEST_F(TrajectoryPointInterfaceTest, send_movec_joint_pose)
{
  // via is a Q, target is a Pose
  urcl::Q send_via{ 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 };
  urcl::Pose send_target(1.2, 3.1, 2.2, -3.4, -1.1, -1.2);
  double blend_radius = 0.25;
  double acceleration = 0.7;
  double velocity = 0.5;
  int32_t mode = 0;
  auto primitive = std::make_shared<control::MoveCPrimitive>(
      urcl::MotionTarget{ send_via }, urcl::MotionTarget{ send_target }, blend_radius, acceleration, velocity, mode);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  // via is a joint configuration, target is a pose -> MOVEC_POSE_JOINT
  EXPECT_EQ(received_primitive->type, control::MotionType::MOVEC_POSE_JOINT);
  auto movec = std::static_pointer_cast<control::MoveCPrimitive>(received_primitive);
  EXPECT_EQ(std::get<urcl::Q>(movec->getVia()).values, send_via.values);
  EXPECT_EQ(std::get<urcl::Pose>(movec->getTarget().value()), send_target);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->acceleration, acceleration);
  EXPECT_EQ(received_primitive->velocity, velocity);
  EXPECT_EQ(movec->mode, mode);
}

TEST_F(TrajectoryPointInterfaceTest, send_optimovej_pose)
{
  const urcl::Pose expected_pose(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);
  double blend_radius = 0.1;
  double acceleration_fraction = 0.4;
  double velocity_fraction = 0.6;
  auto primitive = std::make_shared<control::OptimoveJPrimitive>(
      urcl::MotionTarget{ std::in_place_type<urcl::Pose>, 0.1, -0.2, 0.3, 0.4, -0.5, 0.6 }, blend_radius,
      acceleration_fraction, velocity_fraction);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::OPTIMOVEJ_POSE);
  EXPECT_EQ(std::get<urcl::Pose>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value()),
            expected_pose);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity_fraction);
  EXPECT_EQ(received_primitive->acceleration, acceleration_fraction);
}

TEST_F(TrajectoryPointInterfaceTest, send_optimovel_joint)
{
  urcl::Q send_joints{ 1.2, 3.1, 2.2, -3.4, -1.1, -1.2 };
  double blend_radius = 0.1;
  double acceleration_fraction = 0.4;
  double velocity_fraction = 0.6;
  auto primitive = std::make_shared<control::OptimoveLPrimitive>(urcl::MotionTarget{ send_joints }, blend_radius,
                                                                 acceleration_fraction, velocity_fraction);

  traj_point_interface_->writeMotionPrimitive(primitive);
  auto received_primitive = client_->getMotionPrimitive();
  EXPECT_EQ(received_primitive->type, control::MotionType::OPTIMOVEL_JOINT);
  EXPECT_EQ(std::get<urcl::Q>(
                std::static_pointer_cast<control::MotionPrimitiveWithTarget>(received_primitive)->getTarget().value())
                .values,
            send_joints.values);
  EXPECT_EQ(received_primitive->blend_radius, blend_radius);
  EXPECT_EQ(received_primitive->velocity, velocity_fraction);
  EXPECT_EQ(received_primitive->acceleration, acceleration_fraction);
}

TEST_F(TrajectoryPointInterfaceTest, unsupported_motion_type_throws)
{
  auto primitive = std::make_shared<control::MotionPrimitive>();
  EXPECT_THROW(traj_point_interface_->writeMotionPrimitive(primitive), urcl::UnsupportedMotionType);
}

TEST_F(TrajectoryPointInterfaceTest, disconnected_callbacks_are_called_correctly)
{
  std::atomic<bool> disconnect_called_1 = false;
  std::atomic<bool> disconnect_called_2 = false;

  // Register disconnection callbacks
  int disconnection_callback_id_1 =
      traj_point_interface_->registerDisconnectionCallback([&disconnect_called_1](const socket_t fd) {
        std::cout << "Disconnection 1 callback called with fd: " << fd << std::endl;
        disconnect_called_1 = true;
      });
  int disconnection_callback_id_2 =
      traj_point_interface_->registerDisconnectionCallback([&disconnect_called_2](const socket_t fd) {
        std::cout << "Disconnection 2 callback called with fd: " << fd << std::endl;
        disconnect_called_2 = true;
      });

  // Close the client connection
  client_->close();
  std::unique_lock<std::mutex> lk(g_connection_mutex);
  g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                  [&]() { return !traj_point_interface_->connected.load(); });
  EXPECT_TRUE(disconnect_called_1);
  EXPECT_TRUE(disconnect_called_2);

  // Unregister 1. 2 should still be called
  disconnect_called_1 = false;
  disconnect_called_2 = false;
  client_.reset(new Client(50003));
  g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                  [&]() { return traj_point_interface_->connected.load(); });

  traj_point_interface_->unregisterDisconnectionCallback(disconnection_callback_id_1);
  client_->close();
  g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                  [&]() { return !traj_point_interface_->connected.load(); });
  EXPECT_FALSE(disconnect_called_1);
  EXPECT_TRUE(disconnect_called_2);

  // Unregister both. None should be called
  disconnect_called_1 = false;
  disconnect_called_2 = false;
  client_.reset(new Client(50003));
  g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                  [&]() { return traj_point_interface_->connected.load(); });
  traj_point_interface_->unregisterDisconnectionCallback(disconnection_callback_id_2);
  client_->close();
  g_connection_condition.wait_for(lk, std::chrono::seconds(1),
                                  [&]() { return !traj_point_interface_->connected.load(); });
  EXPECT_FALSE(disconnect_called_1);
  EXPECT_FALSE(disconnect_called_2);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
