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
#include <variant>
#include <ur_client_library/types.h>

namespace urcl
{
namespace control
{

/*!
 * \brief Discriminator for the motion primitive type sent over the trajectory interface.
 *
 * The base values (``MOVEJ``, ``MOVEL``, ``MOVEP``, ``MOVEC``, ``OPTIMOVEJ``, ``OPTIMOVEL``) use
 * their "natural" target type for the URScript command (joint configuration for ``movej`` /
 * ``optimovej``, Cartesian pose for ``movel`` / ``movep`` / ``movec`` / ``optimovel``). The
 * additional ``*_POSE`` and ``*_JOINT`` entries indicate that the *other* target kind was
 * requested, e.g. ``MOVEJ_POSE`` performs a ``movej`` towards a Cartesian pose, and
 * ``MOVEC_POSE_JOINT`` performs a ``movec`` whose via point is a Cartesian pose and whose target
 * is a joint configuration.
 *
 * These values must stay in sync with the ``MOTION_TYPE_*`` constants in
 * ``resources/external_control.urscript``.
 */
enum class MotionType : uint8_t
{
  MOVEJ = 0,              //!< ``movej`` towards a joint configuration.
  MOVEL = 1,              //!< ``movel`` towards a Cartesian pose.
  MOVEP = 2,              //!< ``movep`` towards a Cartesian pose.
  MOVEC = 3,              //!< ``movec`` with via and target as Cartesian poses.
  OPTIMOVEJ = 4,          //!< ``optimovej`` towards a joint configuration.
  OPTIMOVEL = 5,          //!< ``optimovel`` towards a Cartesian pose.
  MOVEJ_POSE = 6,         //!< ``movej`` towards a Cartesian pose.
  MOVEL_JOINT = 7,        //!< ``movel`` towards a joint configuration.
  MOVEP_JOINT = 8,        //!< ``movep`` towards a joint configuration.
  MOVEC_JOINT = 9,        //!< ``movec`` with via and target both as joint configurations.
  MOVEC_JOINT_POSE = 10,  //!< ``movec`` with a Cartesian via and a joint target.
  MOVEC_POSE_JOINT = 11,  //!< ``movec`` with a joint via and a Cartesian target.
  OPTIMOVEJ_POSE = 12,    //!< ``optimovej`` towards a Cartesian pose.
  OPTIMOVEL_JOINT = 13,   //!< ``optimovel`` towards a joint configuration.
  SPLINE = 51,
  UNKNOWN = 255
};

std::string motionTypeToString(const MotionType type);

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
  /*!
   * \brief Construct a MoveJ primitive from a \ref urcl::MotionTarget.
   *
   * If ``target`` holds a \ref urcl::Q, ``type`` is set to \ref MotionType::MOVEJ and
   * ``target_joint_configuration`` is populated. If ``target`` holds a \ref urcl::Pose, ``type``
   * is set to \ref MotionType::MOVEJ_POSE and ``target_pose`` is populated instead; the robot
   * will internally solve inverse kinematics to reach the pose with a ``movej``.
   */
  MoveJPrimitive(const urcl::MotionTarget& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04)
  {
    this->duration = duration;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;

    std::visit(
        [&](const auto& target_variant) {
          using T = std::decay_t<decltype(target_variant)>;
          if constexpr (std::is_same_v<T, urcl::Pose>)
          {
            type = MotionType::MOVEJ_POSE;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<T, Q>)
          {
            type = MotionType::MOVEJ;
            target_joint_configuration = target_variant.values;
          }
        },
        target);
  }

  urcl::vector6d_t target_joint_configuration{};
  urcl::Pose target_pose{};
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
  /*!
   * \brief Construct a MoveL primitive from a \ref urcl::MotionTarget.
   *
   * If ``target`` holds a \ref urcl::Pose, ``type`` is set to \ref MotionType::MOVEL. If it
   * holds a \ref urcl::Q, ``type`` is set to \ref MotionType::MOVEL_JOINT and the configuration
   * is stored in ``target_joint_configuration``. The robot will still execute a tool-space
   * linear motion, resolving the joint configuration to its forward kinematics pose on the
   * controller.
   */
  MoveLPrimitive(const urcl::MotionTarget& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04)
  {
    this->duration = duration;
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
    std::visit(
        [&](const auto& target_variant) {
          using T = std::decay_t<decltype(target_variant)>;
          if constexpr (std::is_same_v<T, urcl::Pose>)
          {
            type = MotionType::MOVEL;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<T, Q>)
          {
            type = MotionType::MOVEL_JOINT;
            target_joint_configuration = target_variant.values;
          }
        },
        target);
  }

  urcl::Pose target_pose{};
  urcl::vector6d_t target_joint_configuration{};
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
  /*!
   * \brief Construct a MoveP primitive from a \ref urcl::MotionTarget.
   *
   * Analogous to \ref MoveJPrimitive / \ref MoveLPrimitive: a \ref urcl::Pose selects
   * \ref MotionType::MOVEP, a \ref urcl::Q selects \ref MotionType::MOVEP_JOINT.
   */
  MovePPrimitive(const MotionTarget& target, const double blend_radius = 0, const double acceleration = 1.4,
                 const double velocity = 1.04)
  {
    std::visit(
        [&](const auto& target_variant) {
          using T = std::decay_t<decltype(target_variant)>;
          if constexpr (std::is_same_v<T, urcl::Pose>)
          {
            type = MotionType::MOVEP;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<T, Q>)
          {
            type = MotionType::MOVEP_JOINT;
            target_joint_configuration = target_variant.values;
          }
        },
        target);
    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
  }

  urcl::Pose target_pose{};
  urcl::vector6d_t target_joint_configuration{};
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

  /*!
   * \brief Construct a MoveC primitive from two \ref urcl::MotionTarget values.
   *
   * Every combination of \ref urcl::Pose and \ref urcl::Q for the via point and the target is
   * supported and mapped to the corresponding \ref MotionType (``MOVEC``, ``MOVEC_JOINT``,
   * ``MOVEC_POSE_JOINT``, or ``MOVEC_JOINT_POSE``). The naming convention is
   * ``MOVEC_<target>_<via>``, i.e. ``MOVEC_POSE_JOINT`` denotes a movec whose target is a pose
   * and whose via point is a joint configuration.
   *
   * Unhandled variant alternatives are caught at compile time via ``static_assert``.
   */
  MoveCPrimitive(const MotionTarget& via_point, const MotionTarget& target, const double blend_radius = 0,
                 const double acceleration = 1.4, const double velocity = 1.04, const int32_t mode = 0)
  {
    std::visit(
        [&](const auto& via_variant, const auto& target_variant) {
          using ViaT = std::decay_t<decltype(via_variant)>;
          using TargetT = std::decay_t<decltype(target_variant)>;
          static_assert(std::is_same_v<ViaT, Q> || std::is_same_v<ViaT, urcl::Pose>, "Unhandled MotionTarget "
                                                                                     "alternative for via_point");
          static_assert(std::is_same_v<TargetT, Q> || std::is_same_v<TargetT, urcl::Pose>, "Unhandled MotionTarget "
                                                                                           "alternative for target");

          if constexpr (std::is_same_v<ViaT, urcl::Pose> && std::is_same_v<TargetT, urcl::Pose>)
          {
            type = MotionType::MOVEC;
            via_point_pose = via_variant;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<ViaT, Q> && std::is_same_v<TargetT, Q>)
          {
            type = MotionType::MOVEC_JOINT;
            via_point_joint_configuration = via_variant.values;
            target_joint_configuration = target_variant.values;
          }
          else if constexpr (std::is_same_v<ViaT, Q> && std::is_same_v<TargetT, urcl::Pose>)
          {
            type = MotionType::MOVEC_POSE_JOINT;
            via_point_joint_configuration = via_variant.values;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<ViaT, urcl::Pose> && std::is_same_v<TargetT, Q>)
          {
            type = MotionType::MOVEC_JOINT_POSE;
            via_point_pose = via_variant;
            target_joint_configuration = target_variant.values;
          }
        },
        via_point, target);

    this->acceleration = acceleration;
    this->velocity = velocity;
    this->blend_radius = blend_radius;
    this->mode = mode;
  }

  urcl::Pose via_point_pose{};
  urcl::Pose target_pose{};
  urcl::vector6d_t via_point_joint_configuration{};
  urcl::vector6d_t target_joint_configuration{};
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

  /*!
   * \brief Construct an OptimoveJ primitive from a \ref urcl::MotionTarget.
   *
   * A \ref urcl::Q selects \ref MotionType::OPTIMOVEJ, a \ref urcl::Pose selects
   * \ref MotionType::OPTIMOVEJ_POSE.
   */
  OptimoveJPrimitive(const MotionTarget& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3)
  {
    std::visit(
        [&](const auto& target_variant) {
          using T = std::decay_t<decltype(target_variant)>;
          if constexpr (std::is_same_v<T, urcl::Pose>)
          {
            type = MotionType::OPTIMOVEJ_POSE;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<T, Q>)
          {
            type = MotionType::OPTIMOVEJ;
            target_joint_configuration = target_variant.values;
          }
        },
        target);
    this->blend_radius = blend_radius;
    this->acceleration = acceleration_fraction;
    this->velocity = velocity_fraction;
  }

  bool validate() const override;

  urcl::vector6d_t target_joint_configuration{};
  Pose target_pose{};
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
  /*!
   * \brief Construct an OptimoveL primitive from a \ref urcl::MotionTarget.
   *
   * A \ref urcl::Pose selects \ref MotionType::OPTIMOVEL, a \ref urcl::Q selects
   * \ref MotionType::OPTIMOVEL_JOINT.
   */
  OptimoveLPrimitive(const MotionTarget& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3)
  {
    std::visit(
        [&](const auto& target_variant) {
          using T = std::decay_t<decltype(target_variant)>;
          if constexpr (std::is_same_v<T, urcl::Pose>)
          {
            type = MotionType::OPTIMOVEL;
            target_pose = target_variant;
          }
          else if constexpr (std::is_same_v<T, Q>)
          {
            type = MotionType::OPTIMOVEL_JOINT;
            target_joint_configuration = target_variant.values;
          }
        },
        target);
    this->blend_radius = blend_radius;
    this->acceleration = acceleration_fraction;
    this->velocity = velocity_fraction;
  }

  bool validate() const override;

  urcl::Pose target_pose{};
  vector6d_t target_joint_configuration{};
};
}  // namespace control
}  // namespace urcl
#endif  // UR_CLIENT_LIBRARY_MOTION_PRIMITIVES_H_INCLUDED
