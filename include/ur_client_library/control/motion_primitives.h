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

class MotionPrimitive
{
public:
  MotionPrimitive(const double blend_radius = 0,
                  const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                  const double acceleration = 1.4, const double velocity = 1.04)
    : duration(duration), acceleration(acceleration), velocity(velocity), blend_radius(blend_radius)
  {
  }
  virtual ~MotionPrimitive() = default;
  MotionType type = MotionType::UNKNOWN;
  std::chrono::duration<double> duration;
  double acceleration;
  double velocity;
  double blend_radius = 0.0;

  virtual bool validate() const;
};

class MotionPrimitiveWithTarget : public MotionPrimitive
{
public:
  MotionPrimitiveWithTarget(const double blend_radius = 0,
                            const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                            const double acceleration = 1.4, const double velocity = 1.04)
    : MotionPrimitive(blend_radius, duration, acceleration, velocity)
  {
  }

  virtual void setTarget(const MotionTarget& target) = 0;

  [[nodiscard]]
  std::optional<MotionTarget> getTarget() const
  {
    if (target_ != nullptr)
    {
      return *target_;
    }
    return std::nullopt;
  }

protected:
  std::unique_ptr<MotionTarget> target_;
};

class MoveJPrimitive : public MotionPrimitiveWithTarget
{
public:
  MoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04);

  /*!
   * \brief Construct a MoveJ primitive from a \ref urcl::MotionTarget.
   *
   * If ``target`` holds a \ref urcl::Q, ``type`` is set to \ref MotionType::MOVEJ. If ``target``
   * holds a \ref urcl::Pose, ``type`` is set to \ref MotionType::MOVEJ_POSE; the robot will
   * internally solve inverse kinematics to reach the pose with a ``movej``. The stored target is
   * accessible via ``getTarget()``.
   */
  MoveJPrimitive(const urcl::MotionTarget& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04);

  void setTarget(const MotionTarget& target) override;

  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::vector6d_t target_joint_configuration{};
};

class MoveLPrimitive : public MotionPrimitiveWithTarget
{
public:
  MoveLPrimitive(const urcl::Pose& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04);

  /*!
   * \brief Construct a MoveL primitive from a \ref urcl::MotionTarget.
   *
   * If ``target`` holds a \ref urcl::Pose, ``type`` is set to \ref MotionType::MOVEL. If it
   * holds a \ref urcl::Q, ``type`` is set to \ref MotionType::MOVEL_JOINT. The robot will still
   * execute a tool-space linear motion, resolving the joint configuration to its forward
   * kinematics pose on the controller. The stored target is accessible via ``getTarget()``.
   */
  MoveLPrimitive(const urcl::MotionTarget& target, const double blend_radius = 0,
                 const std::chrono::duration<double> duration = std::chrono::milliseconds(0),
                 const double acceleration = 1.4, const double velocity = 1.04);

  ~MoveLPrimitive() override;

  void setTarget(const MotionTarget& target) override;

  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::Pose target_pose{};
};

class MovePPrimitive : public MotionPrimitiveWithTarget
{
public:
  MovePPrimitive(const urcl::Pose& target, const double blend_radius = 0, const double acceleration = 1.4,
                 const double velocity = 1.04);

  /*!
   * \brief Construct a MoveP primitive from a \ref urcl::MotionTarget.
   *
   * Analogous to \ref MoveJPrimitive / \ref MoveLPrimitive: a \ref urcl::Pose selects
   * \ref MotionType::MOVEP, a \ref urcl::Q selects \ref MotionType::MOVEP_JOINT.
   */
  MovePPrimitive(const MotionTarget& target, const double blend_radius = 0, const double acceleration = 1.4,
                 const double velocity = 1.04);

  ~MovePPrimitive() override;

  void setTarget(const MotionTarget& target) override;

  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::Pose target_pose{};
};

class MoveCPrimitive : public MotionPrimitiveWithTarget
{
public:
  MoveCPrimitive(const urcl::Pose& via_point, const urcl::Pose& target, const double blend_radius = 0,
                 const double acceleration = 1.4, const double velocity = 1.04, const int32_t mode = 0);

  /*!
   * \brief Construct a MoveC primitive from two \ref urcl::MotionTarget values.
   *
   * Every combination of \ref urcl::Pose and \ref urcl::Q for the via point and the target is
   * supported and mapped to the corresponding \ref MotionType (``MOVEC``, ``MOVEC_JOINT``,
   * ``MOVEC_POSE_JOINT``, or ``MOVEC_JOINT_POSE``). The naming convention is
   * ``MOVEC_<target>_<via>``, i.e. ``MOVEC_POSE_JOINT`` denotes a movec whose target is a pose
   * and whose via point is a joint configuration.
   */
  MoveCPrimitive(const MotionTarget& via_point, const MotionTarget& target, const double blend_radius = 0,
                 const double acceleration = 1.4, const double velocity = 1.04, const int32_t mode = 0);

  ~MoveCPrimitive() override;

  void setTarget(const MotionTarget& target) override;

  void setVia(const MotionTarget& via_point);

  MotionTarget getVia() const
  {
    return via_point_;
  }

  [[deprecated("Use getVia() and setVia() instead.")]]
  urcl::Pose via_point_pose{};
  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::Pose target_pose{};
  int32_t mode = 0;

protected:
  urcl::MotionTarget via_point_;

private:
  void recomputeType();

  void refreshLegacyFields();
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

class OptimoveJPrimitive : public MotionPrimitiveWithTarget
{
public:
  OptimoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3);

  /*!
   * \brief Construct an OptimoveJ primitive from a \ref urcl::MotionTarget.
   *
   * A \ref urcl::Q selects \ref MotionType::OPTIMOVEJ, a \ref urcl::Pose selects
   * \ref MotionType::OPTIMOVEJ_POSE.
   */
  OptimoveJPrimitive(const MotionTarget& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3);

  void setTarget(const MotionTarget& target) override;

  bool validate() const override;

  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::vector6d_t target_joint_configuration{};
};

class OptimoveLPrimitive : public MotionPrimitiveWithTarget
{
public:
  OptimoveLPrimitive(const urcl::Pose& target, const double blend_radius = 0, const double acceleration_fraction = 0.3,
                     const double velocity_fraction = 0.3);
  /*!
   * \brief Construct an OptimoveL primitive from a \ref urcl::MotionTarget.
   *
   * A \ref urcl::Pose selects \ref MotionType::OPTIMOVEL, a \ref urcl::Q selects
   * \ref MotionType::OPTIMOVEL_JOINT.
   */
  OptimoveLPrimitive(const MotionTarget& target, const double blend_radius = 0,
                     const double acceleration_fraction = 0.3, const double velocity_fraction = 0.3);

  ~OptimoveLPrimitive() override;

  void setTarget(const MotionTarget& target) override;

  bool validate() const override;

  [[deprecated("Use getTarget() and setTarget() instead.")]]
  urcl::Pose target_pose{};
};
}  // namespace control
}  // namespace urcl
#endif  // UR_CLIENT_LIBRARY_MOTION_PRIMITIVES_H_INCLUDED
