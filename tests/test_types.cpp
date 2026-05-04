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

#include <sstream>
#include <variant>

#include <ur_client_library/types.h>

using namespace urcl;

TEST(TestTypes, Q_constructors_and_equality)
{
  const Q from_doubles{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };
  const vector6d_t arr{ { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 } };
  const Q from_array(arr);

  EXPECT_EQ(from_doubles, from_array);
  EXPECT_EQ(from_doubles.values.size(), 6u);
  EXPECT_EQ(from_array.values.size(), 6u);

  const Q different{ 1.0, 0.2, 0.3, 0.4, 0.5, 0.6 };
  EXPECT_FALSE(from_doubles == different);
}

TEST(TestTypes, Q_equals_vector6d)
{
  const Q q{ 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
  const vector6d_t match{ { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 } };
  const vector6d_t mismatch{ { 1.0, 2.0, 3.0, 4.0, 5.0, 7.0 } };

  EXPECT_TRUE(q == match);
  EXPECT_FALSE(q == mismatch);
}

TEST(TestTypes, Pose_default_and_constructors)
{
  Pose default_pose;
  EXPECT_DOUBLE_EQ(default_pose.x, 0.0);
  EXPECT_DOUBLE_EQ(default_pose.y, 0.0);
  EXPECT_DOUBLE_EQ(default_pose.z, 0.0);
  EXPECT_DOUBLE_EQ(default_pose.rx, 0.0);
  EXPECT_DOUBLE_EQ(default_pose.ry, 0.0);
  EXPECT_DOUBLE_EQ(default_pose.rz, 0.0);
  EXPECT_FALSE(default_pose.q_near.has_value());

  const Pose cartesian(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  EXPECT_DOUBLE_EQ(cartesian.x, 1.0);
  EXPECT_DOUBLE_EQ(cartesian.y, 2.0);
  EXPECT_DOUBLE_EQ(cartesian.z, 3.0);
  EXPECT_DOUBLE_EQ(cartesian.rx, 0.1);
  EXPECT_DOUBLE_EQ(cartesian.ry, 0.2);
  EXPECT_DOUBLE_EQ(cartesian.rz, 0.3);
  EXPECT_FALSE(cartesian.q_near.has_value());

  const Q hint{ 0.0, -0.5, 1.0, 0.0, 0.5, 0.0 };
  const Pose with_hint(10.0, 20.0, 30.0, 1.0, 2.0, 3.0, hint);
  ASSERT_TRUE(with_hint.q_near.has_value());
  EXPECT_EQ(*with_hint.q_near, hint);
}

TEST(TestTypes, Pose_equality)
{
  const Pose a(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  const Pose b(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
  const Pose c(1.1, 2.0, 3.0, 0.1, 0.2, 0.3);

  const Q q{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const Pose with_q(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, q);
  const Pose without_q_near(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
  EXPECT_FALSE(with_q == without_q_near);

  const Pose with_same_q(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, q);
  EXPECT_TRUE(with_q == with_same_q);

  const Q q2{ 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  const Pose different_hint(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, q2);
  EXPECT_FALSE(with_q == different_hint);
}

TEST(TestTypes, MotionTarget_variant)
{
  const MotionTarget joint_side = Q{ 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 };
  ASSERT_TRUE(std::holds_alternative<Q>(joint_side));
  EXPECT_EQ(std::get<Q>(joint_side), Q(0.0, 0.1, 0.2, 0.3, 0.4, 0.5));

  const MotionTarget cart_side = Pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
  ASSERT_TRUE(std::holds_alternative<Pose>(cart_side));
  const Pose& p = std::get<Pose>(cart_side);
  EXPECT_DOUBLE_EQ(p.x, 1.0);
  EXPECT_DOUBLE_EQ(p.z, 3.0);
}

TEST(TestTypes, array_stream_operator)
{
  const vector3d_t v3{ { 1.0, 2.5, -3.0 } };
  const vector6int32_t v6i{ { -1, 0, 42, 3, 4, 5 } };

  std::ostringstream out3;
  out3 << v3;
  EXPECT_EQ(out3.str(), "[1, 2.5, -3]");

  std::ostringstream out6;
  out6 << v6i;
  EXPECT_EQ(out6.str(), "[-1, 0, 42, 3, 4, 5]");
}

TEST(TestTypes, toUnderlying)
{
  enum class Small : uint8_t
  {
    X = 7,
    Y = 200
  };
  static_assert(toUnderlying(Small::X) == 7, "");
  EXPECT_EQ(toUnderlying(Small::X), 7u);
  EXPECT_EQ(toUnderlying(Small::Y), 200u);
}

TEST(TestTypes, HandlerFunction_equality_uses_id_only)
{
  int calls_a = 0;
  int calls_b = 0;
  HandlerFunction<void()> ha(1, [&calls_a]() { ++calls_a; });
  HandlerFunction<void()> hb(1, [&calls_b]() { ++calls_b; });
  HandlerFunction<void()> hc(2, [&calls_a]() { ++calls_a; });

  EXPECT_TRUE(ha == hb);
  EXPECT_FALSE(ha == hc);

  ha.function();
  EXPECT_EQ(calls_a, 1);
  EXPECT_EQ(calls_b, 0);
}
