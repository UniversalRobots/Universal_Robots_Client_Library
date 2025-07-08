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

#ifndef UR_CLIENT_LIBRARY_MOTION_PRIMITIVES_H_INCLUDED
#define UR_CLIENT_LIBRARY_MOTION_PRIMITIVES_H_INCLUDED

#include <chrono>
#include <optional>
#include <ur_client_library/types.h>

namespace urcl
{
namespace control
{

enum class MotionType : uint8_t
{
  MOVEJ = 0,
  MOVEL = 1,
  MOVEP = 2,
  MOVEC = 3,
  OPTIMOVEJ = 4,
  OPTIMOVEL = 5,
  SPLINE = 51,
  UNKNOWN = 255
};

/*!
 * Spline types
 */
enum class TrajectorySplineType : int32_t
{
  SPLINE_CUBIC = 1,
  SPLINE_QUINTIC = 2
};

struct MotionPrimitive
{
  virtual ~MotionPrimitive() = default;
  MotionType type = MotionType::UNKNOWN;
  std::chrono::duration<double> duration;
  double acceleration;
  double velocity;
  double blend_radius = 0.0;

  virtual bool validate() const;
};

struct MoveJPrimitive : public MotionPrimitive
{
  MoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04)
  {
    type = MotionType::MOVEJ;
    target_joint_configuration = target;
    this->duration = duration;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
  }

  urcl::vector6d_t target_joint_configuration;
};

struct MoveLPrimitive : public MotionPrimitive
{
  MoveLPrimitive(const urcl::Pose& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04)
  {
    type = MotionType::MOVEL;
    target_pose = target;
    this->duration = duration;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
  }

  urcl::Pose target_pose;
};

struct MovePPrimitive : public MotionPrimitive
{
  MovePPrimitive(const urcl::Pose& target, const double blend_radius = 0, const double acceleration = 1.4,
                 const double velocity = 1.04)
  {
    type = MotionType::MOVEP;
    target_pose = target;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
  }

  urcl::Pose target_pose;
};

struct MoveCPrimitive : public MotionPrimitive
{
  MoveCPrimitive(const urcl::Pose& via_point, const urcl::Pose& target, const double blend_radius = 0,
                 const double acceleration = 1.4, const double velocity = 1.04, const int32_t mode = 0)
  {
    type = MotionType::MOVEC;
    via_point_pose = via_point;
    target_pose = target;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
    this->mode = mode;
  }

  urcl::Pose via_point_pose;
  urcl::Pose target_pose;
  int32_t mode = 0;
};

struct SplinePrimitive : public MotionPrimitive
{
  SplinePrimitive(const urcl::vector6d_t& target_positions, const vector6d_t& target_velocities,
                  const std::optional<vector6d_t>& target_accelerations,
                  const std::chrono::duration<double> duration = std::chrono::milliseconds(0))
  {
    type = MotionType::SPLINE;
    this->target_positions = target_positions;
    this->target_velocities = target_velocities;
    this->target_accelerations = target_accelerations;
    this->duration = duration;
  }

  control::TrajectorySplineType getSplineType() const
  {
    if (target_accelerations.has_value())
    {
      return control::TrajectorySplineType::SPLINE_QUINTIC;
    }
    else
    {
      return control::TrajectorySplineType::SPLINE_CUBIC;
    }
  }

  bool validate() const override;

  vector6d_t target_positions;
  vector6d_t target_velocities;
  std::optional<vector6d_t> target_accelerations;
};

struct OptimoveJPrimitive : public MotionPrimitive
{
  OptimoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3)
  {
    type = MotionType::OPTIMOVEJ;
    target_joint_configuration = target;
    this->blend_radius = blend_radius;
    this->acceleration = acceleration_fraction;
    this->velocity = velocity_fraction;
  }

  bool validate() const override;

  urcl::vector6d_t target_joint_configuration;
};

struct OptimoveLPrimitive : public MotionPrimitive
{
  OptimoveLPrimitive(const urcl::Pose& target, const double blend_radius = 0, const double acceleration_fraction = 0.3,
                     const double velocity_fraction = 0.3)
  {
    type = MotionType::OPTIMOVEL;
    target_pose = target;
    this->blend_radius = blend_radius;
    this->acceleration = acceleration_fraction;
    this->velocity = velocity_fraction;
  }

  bool validate() const override;

  urcl::Pose target_pose;
};
}  // namespace control
}  // namespace urcl
#endif  // UR_CLIENT_LIBRARY_MOTION_PRIMITIVES_H_INCLUDED