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

#include <ur_client_library/control/motion_primitives.h>
#include <ur_client_library/log.h>

namespace urcl::control
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

bool MotionPrimitive::validate() const
{
  if (blend_radius < 0)
  {
    URCL_LOG_ERROR("Negative blend radius passed to motion primitive. This is not allowed.");
    return false;
  }
  if (acceleration < 0)
  {
    URCL_LOG_ERROR("Negative acceleration passed to motion primitive. This is not allowed.");
    return false;
  }
  if (velocity < 0)
  {
    URCL_LOG_ERROR("Negative velocity passed to motion primitive. This is not allowed.");
    return false;
  }
  return true;
}

bool SplinePrimitive::validate() const
{
  // Spline primitives don't have the same restriction as others do. Whether the primitives are valid or not
  // is checked in the URScript program.
  return true;
}
bool OptimoveJPrimitive::validate() const
{
  if (!MotionPrimitive::validate())
  {
    return false;
  }
  if (acceleration <= 0 || acceleration > 1.0)
  {
    URCL_LOG_ERROR("Acceleration fraction must be in range (0, 1].");
    return false;
  }
  if (velocity <= 0 || velocity > 1.0)
  {
    URCL_LOG_ERROR("Velocity fraction must be in range (0, 1].");
    return false;
  }
  return true;
}

bool OptimoveLPrimitive::validate() const
{
  if (!MotionPrimitive::validate())
  {
    return false;
  }
  if (acceleration <= 0 || acceleration > 1.0)
  {
    URCL_LOG_ERROR("Acceleration fraction must be in range (0, 1].");
    return false;
  }
  if (velocity <= 0 || velocity > 1.0)
  {
    URCL_LOG_ERROR("Velocity fraction must be in range (0, 1].");
    return false;
  }
  return true;
}

std::string motionTypeToString(const MotionType type)
{
  switch (type)
  {
    case MotionType::MOVEJ:
      return "MOVEJ";
    case MotionType::MOVEL:
      return "MOVEL";
    case MotionType::MOVEP:
      return "MOVEP";
    case MotionType::MOVEC:
      return "MOVEC";
    case MotionType::OPTIMOVEJ:
      return "OPTIMOVEJ";
    case MotionType::OPTIMOVEL:
      return "OPTIMOVEL";
    case MotionType::MOVEJ_POSE:
      return "MOVEJ_POSE";
    case MotionType::MOVEL_JOINT:
      return "MOVEL_JOINT";
    case MotionType::MOVEP_JOINT:
      return "MOVEP_JOINT";
    case MotionType::MOVEC_JOINT:
      return "MOVEC_JOINT";
    case MotionType::MOVEC_JOINT_POSE:
      return "MOVEC_JOINT_POSE";
    case MotionType::MOVEC_POSE_JOINT:
      return "MOVEC_POSE_JOINT";
    case MotionType::OPTIMOVEJ_POSE:
      return "OPTIMOVEJ_POSE";
    case MotionType::OPTIMOVEL_JOINT:
      return "OPTIMOVEL_JOINT";
    case MotionType::SPLINE:
      return "SPLINE";
    default:
      return "UNKNOWN";
  }
}

MoveJPrimitive::MoveJPrimitive(const MotionTarget& target, const double blend_radius,
                               const std::chrono::duration<double> duration, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(target, blend_radius, duration, acceleration, velocity)
{
}

MoveJPrimitive::MoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius,
                               const std::chrono::duration<double> duration, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(Q(target), blend_radius, duration, acceleration, velocity)
{
}

void MoveJPrimitive::setTarget(const MotionTarget& target)
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        this->target_ = std::make_unique<MotionTarget>(target_variant);
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          type = MotionType::MOVEJ_POSE;

          target_joint_configuration.fill(0);
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          type = MotionType::MOVEJ;
          std::copy(target_variant.values.begin(), target_variant.values.end(), target_joint_configuration.begin());
        }
      },
      target);
}

MoveLPrimitive::MoveLPrimitive(const urcl::Pose& target, const double blend_radius,
                               const std::chrono::duration<double> duration, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(target, blend_radius, duration, acceleration, velocity)
{
}

MoveLPrimitive::MoveLPrimitive(const MotionTarget& target, const double blend_radius,
                               const std::chrono::duration<double> duration, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(target, blend_radius, duration, acceleration, velocity)
{
}
void MoveLPrimitive::setTarget(const MotionTarget& target)
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        target_ = std::make_unique<MotionTarget>(target_variant);
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          type = MotionType::MOVEL;
          target_pose = target_variant;
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          type = MotionType::MOVEL_JOINT;
          target_pose = urcl::Pose{};
        }
      },
      target);
}

MovePPrimitive::MovePPrimitive(const urcl::Pose& target, const double blend_radius, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration, velocity)
{
}

MovePPrimitive::MovePPrimitive(const MotionTarget& target, const double blend_radius, const double acceleration,
                               const double velocity)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration, velocity)
{
}

void MovePPrimitive::setTarget(const MotionTarget& target)
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        target_ = std::make_unique<MotionTarget>(target_variant);
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          type = MotionType::MOVEP;
          target_pose = target_variant;
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          type = MotionType::MOVEP_JOINT;
          target_pose = urcl::Pose{};
        }
      },
      target);
}

MoveCPrimitive::MoveCPrimitive(const urcl::Pose& via_point, const urcl::Pose& target, const double blend_radius,
                               const double acceleration, const double velocity, const int32_t mode)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration, velocity)
  , mode(mode)
  , via_point_(via_point)
{
  setVia(via_point);
  setTarget(target);
}

MoveCPrimitive::MoveCPrimitive(const MotionTarget& via_point, const MotionTarget& target, const double blend_radius,
                               const double acceleration, const double velocity, const int32_t mode)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration, velocity)
  , mode(mode)
  , via_point_(via_point)
{
  setVia(via_point);
  setTarget(target);
}

void MoveCPrimitive::setTarget(const MotionTarget& target)
{
  target_ = std::make_unique<MotionTarget>(target);
  recomputeType();
  refreshLegacyFields();
}

void MoveCPrimitive::setVia(const MotionTarget& via_point)
{
  via_point_ = via_point;
  recomputeType();
  refreshLegacyFields();
}

void MoveCPrimitive::recomputeType()
{
  const bool target_is_pose = std::holds_alternative<urcl::Pose>(*target_);
  const bool via_is_pose = std::holds_alternative<urcl::Pose>(via_point_);
  if (target_is_pose && via_is_pose)
  {
    type = MotionType::MOVEC;
  }
  else if (!target_is_pose && !via_is_pose)
  {
    type = MotionType::MOVEC_JOINT;
  }
  else if (target_is_pose && !via_is_pose)
  {
    type = MotionType::MOVEC_POSE_JOINT;
  }
  else
  {
    type = MotionType::MOVEC_JOINT_POSE;
  }
}

void MoveCPrimitive::refreshLegacyFields()
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          target_pose = target_variant;
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          target_pose = urcl::Pose{};
        }
      },
      *target_);
  std::visit(
      [&](const auto& via_variant) {
        using T = std::decay_t<decltype(via_variant)>;
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          via_point_pose = via_variant;
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          via_point_pose = urcl::Pose{};
        }
      },
      via_point_);
}

OptimoveJPrimitive::OptimoveJPrimitive(const urcl::vector6d_t& target, const double blend_radius,
                                       const double acceleration_fraction, const double velocity_fraction)
  : MotionPrimitiveWithTarget(Q(target), blend_radius, std::chrono::milliseconds(0), acceleration_fraction,
                              velocity_fraction)
{
}

OptimoveJPrimitive::OptimoveJPrimitive(const MotionTarget& target, const double blend_radius,
                                       const double acceleration_fraction, const double velocity_fraction)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration_fraction,
                              velocity_fraction)
{
}

void OptimoveJPrimitive::setTarget(const MotionTarget& target)
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        this->target_ = std::make_unique<MotionTarget>(target_variant);
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          type = MotionType::OPTIMOVEJ_POSE;
          target_joint_configuration.fill(0);
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          type = MotionType::OPTIMOVEJ;
          std::copy(target_variant.values.begin(), target_variant.values.end(), target_joint_configuration.begin());
        }
      },
      target);
}

OptimoveLPrimitive::OptimoveLPrimitive(const urcl::Pose& target, const double blend_radius,
                                       const double acceleration_fraction, const double velocity_fraction)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration_fraction,
                              velocity_fraction)
{
}

OptimoveLPrimitive::OptimoveLPrimitive(const MotionTarget& target, const double blend_radius,
                                       const double acceleration_fraction, const double velocity_fraction)
  : MotionPrimitiveWithTarget(target, blend_radius, std::chrono::milliseconds(0), acceleration_fraction,
                              velocity_fraction)
{
}

void OptimoveLPrimitive::setTarget(const MotionTarget& target)
{
  std::visit(
      [&](const auto& target_variant) {
        using T = std::decay_t<decltype(target_variant)>;
        this->target_ = std::make_unique<MotionTarget>(target_variant);
        if constexpr (std::is_same_v<T, urcl::Pose>)
        {
          type = MotionType::OPTIMOVEL;
          target_pose = target_variant;
        }
        else if constexpr (std::is_same_v<T, Q>)
        {
          type = MotionType::OPTIMOVEL_JOINT;
          target_pose = urcl::Pose{};
        }
      },
      target);
}
#pragma GCC diagnostic pop

}  // namespace urcl::control
