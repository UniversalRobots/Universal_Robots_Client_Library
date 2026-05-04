// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

#include <cstdint>
#include <variant>

#include <ur_client_library/control/motion_primitives.h>

using urcl::MotionTarget;
using urcl::Pose;
using urcl::Q;
using urcl::vector6d_t;
using urcl::control::MotionPrimitive;
using urcl::control::MotionType;
using urcl::control::motionTypeToString;
using urcl::control::MoveCPrimitive;
using urcl::control::MoveJPrimitive;
using urcl::control::MoveLPrimitive;
using urcl::control::MovePPrimitive;
using urcl::control::OptimoveJPrimitive;
using urcl::control::OptimoveLPrimitive;
using urcl::control::SplinePrimitive;
using urcl::control::TrajectorySplineType;

namespace
{
vector6d_t zeroJoints()
{
  return { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
}

Pose samplePose(const double x = 1.0)
{
  return Pose(x, 2.0, 3.0, 0.1, 0.2, 0.3);
}
}  // namespace

TEST(MotionTypeToStringTest, returns_expected_string_for_each_known_type)
{
  EXPECT_EQ(motionTypeToString(MotionType::MOVEJ), "MOVEJ");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEL), "MOVEL");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEP), "MOVEP");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEC), "MOVEC");
  EXPECT_EQ(motionTypeToString(MotionType::OPTIMOVEJ), "OPTIMOVEJ");
  EXPECT_EQ(motionTypeToString(MotionType::OPTIMOVEL), "OPTIMOVEL");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEJ_POSE), "MOVEJ_POSE");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEL_JOINT), "MOVEL_JOINT");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEP_JOINT), "MOVEP_JOINT");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEC_JOINT), "MOVEC_JOINT");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEC_JOINT_POSE), "MOVEC_JOINT_POSE");
  EXPECT_EQ(motionTypeToString(MotionType::MOVEC_POSE_JOINT), "MOVEC_POSE_JOINT");
  EXPECT_EQ(motionTypeToString(MotionType::OPTIMOVEJ_POSE), "OPTIMOVEJ_POSE");
  EXPECT_EQ(motionTypeToString(MotionType::OPTIMOVEL_JOINT), "OPTIMOVEL_JOINT");
  EXPECT_EQ(motionTypeToString(MotionType::SPLINE), "SPLINE");
}

TEST(MotionTypeToStringTest, returns_unknown_for_explicit_unknown_value)
{
  EXPECT_EQ(motionTypeToString(MotionType::UNKNOWN), "UNKNOWN");
}

TEST(MotionTypeToStringTest, returns_unknown_for_out_of_range_value)
{
  const auto out_of_range = static_cast<MotionType>(static_cast<uint8_t>(200));
  EXPECT_EQ(motionTypeToString(out_of_range), "UNKNOWN");
}

TEST(MotionPrimitiveTest, validate_accepts_defaults_and_non_negative_parameters)
{
  MotionPrimitive mp;
  EXPECT_TRUE(mp.validate());
  EXPECT_DOUBLE_EQ(mp.blend_radius, 0.0);
  EXPECT_DOUBLE_EQ(mp.acceleration, 1.4);
  EXPECT_DOUBLE_EQ(mp.velocity, 1.04);
}

TEST(MotionPrimitiveTest, validate_rejects_negative_blend_radius_acceleration_or_velocity)
{
  MotionPrimitive bad_blend(-1.0);
  EXPECT_FALSE(bad_blend.validate());

  MotionPrimitive bad_acc(0, std::chrono::milliseconds(0), -0.1);
  EXPECT_FALSE(bad_acc.validate());

  MotionPrimitive bad_vel(0, std::chrono::milliseconds(0), 1.4, -1.0);
  EXPECT_FALSE(bad_vel.validate());
}

TEST(MoveJPrimitiveTest, joint_vector_constructor_sets_movej_and_target)
{
  const vector6d_t q{ { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 } };
  MoveJPrimitive mj(q);
  EXPECT_EQ(mj.type, MotionType::MOVEJ);
  ASSERT_TRUE(mj.getTarget().has_value());
  ASSERT_TRUE(std::holds_alternative<Q>(*mj.getTarget()));
  EXPECT_EQ(std::get<Q>(*mj.getTarget()), Q(q));
}

TEST(MoveJPrimitiveTest, motion_target_q_vs_pose_sets_motion_type)
{
  MoveJPrimitive from_q(MotionTarget(Q(0, 0, 0, 0, 0, 0)));
  EXPECT_EQ(from_q.type, MotionType::MOVEJ);

  const MotionTarget pose_target{ samplePose() };
  MoveJPrimitive movej_from_pose(pose_target);
  EXPECT_EQ(movej_from_pose.type, MotionType::MOVEJ_POSE);
  ASSERT_TRUE(movej_from_pose.getTarget().has_value());
  ASSERT_TRUE(std::holds_alternative<Pose>(*movej_from_pose.getTarget()));
}

TEST(MoveLPrimitiveTest, pose_and_motion_target_variants_set_motion_type)
{
  MoveLPrimitive ml(samplePose());
  EXPECT_EQ(ml.type, MotionType::MOVEL);
  ASSERT_TRUE(ml.getTarget().has_value());
  ASSERT_TRUE(std::holds_alternative<Pose>(*ml.getTarget()));

  MoveLPrimitive mlj(MotionTarget(Q(1, 2, 3, 4, 5, 6)));
  EXPECT_EQ(mlj.type, MotionType::MOVEL_JOINT);
  ASSERT_TRUE(std::holds_alternative<Q>(*mlj.getTarget()));
}

TEST(MovePPrimitiveTest, pose_and_motion_target_variants_set_motion_type)
{
  MovePPrimitive mp(samplePose());
  EXPECT_EQ(mp.type, MotionType::MOVEP);

  MovePPrimitive mpj(MotionTarget(Q(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)));
  EXPECT_EQ(mpj.type, MotionType::MOVEP_JOINT);
}

TEST(MoveCPrimitiveTest, recomputes_motion_type_for_via_target_combinations)
{
  const Pose via_p = samplePose(0.0);
  const Pose target_p = samplePose(10.0);
  const Q via_q(0, 0, 0, 0, 0, 0);
  const Q target_q(1, 1, 1, 1, 1, 1);

  MoveCPrimitive pp(via_p, target_p);
  EXPECT_EQ(pp.type, MotionType::MOVEC);

  MoveCPrimitive qq(via_q, target_q);
  EXPECT_EQ(qq.type, MotionType::MOVEC_JOINT);

  MoveCPrimitive qp(via_q, target_p);
  EXPECT_EQ(qp.type, MotionType::MOVEC_POSE_JOINT);

  MoveCPrimitive pq(via_p, target_q);
  EXPECT_EQ(pq.type, MotionType::MOVEC_JOINT_POSE);
}

TEST(MoveCPrimitiveTest, set_via_and_set_target_update_type_and_get_via)
{
  MoveCPrimitive mc(samplePose(0.0), samplePose(1.0));
  EXPECT_EQ(mc.type, MotionType::MOVEC);

  mc.setVia(MotionTarget(Q(0, 0, 0, 0, 0, 0)));
  mc.setTarget(MotionTarget(Q(1, 1, 1, 1, 1, 1)));
  EXPECT_EQ(mc.type, MotionType::MOVEC_JOINT);

  ASSERT_TRUE(std::holds_alternative<Q>(mc.getVia()));
  EXPECT_EQ(std::get<Q>(mc.getVia()), Q(0, 0, 0, 0, 0, 0));

  EXPECT_EQ(mc.mode, 0);
}

TEST(SplinePrimitiveTest, type_spline_and_getSplineType_cubic_vs_quintic)
{
  const vector6d_t pos = zeroJoints();
  const vector6d_t vel = zeroJoints();

  SplinePrimitive cubic(pos, vel, std::nullopt);
  EXPECT_EQ(cubic.type, MotionType::SPLINE);
  EXPECT_EQ(cubic.getSplineType(), TrajectorySplineType::SPLINE_CUBIC);
  EXPECT_TRUE(cubic.validate());

  const vector6d_t acc = zeroJoints();
  SplinePrimitive quintic(pos, vel, acc);
  EXPECT_EQ(quintic.getSplineType(), TrajectorySplineType::SPLINE_QUINTIC);
  EXPECT_TRUE(quintic.validate());
}

TEST(OptimoveJPrimitiveTest, validate_fraction_range_and_motion_types)
{
  OptimoveJPrimitive ok(zeroJoints(), 0.0, 0.3, 0.3);
  EXPECT_TRUE(ok.validate());
  EXPECT_EQ(ok.type, MotionType::OPTIMOVEJ);

  const MotionTarget pose_target{ samplePose() };
  OptimoveJPrimitive optj_from_pose(pose_target);
  EXPECT_EQ(optj_from_pose.type, MotionType::OPTIMOVEJ_POSE);

  OptimoveJPrimitive bad_acc(zeroJoints(), 0.0, 0.0, 0.5);
  EXPECT_FALSE(bad_acc.validate());

  OptimoveJPrimitive bad_acc_high(zeroJoints(), 0.0, 1.01, 0.5);
  EXPECT_FALSE(bad_acc_high.validate());

  OptimoveJPrimitive bad_vel(zeroJoints(), 0.0, 0.5, 0.0);
  EXPECT_FALSE(bad_vel.validate());

  OptimoveJPrimitive bad_vel_high(zeroJoints(), 0.0, 0.5, 1.01);
  EXPECT_FALSE(bad_vel_high.validate());
}

TEST(OptimoveLPrimitiveTest, validate_fraction_range_and_motion_types)
{
  OptimoveLPrimitive ok(samplePose(), 0.0, 0.3, 0.3);
  EXPECT_TRUE(ok.validate());
  EXPECT_EQ(ok.type, MotionType::OPTIMOVEL);

  OptimoveLPrimitive from_q(MotionTarget(Q(0, 0, 0, 0, 0, 0)));
  EXPECT_EQ(from_q.type, MotionType::OPTIMOVEL_JOINT);

  OptimoveLPrimitive bad(samplePose(), 0.0, 0.0, 0.5);
  EXPECT_FALSE(bad.validate());
}
