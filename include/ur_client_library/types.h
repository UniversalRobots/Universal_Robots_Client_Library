/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <inttypes.h>
#include <array>
#include <functional>
#include <iostream>
#include <variant>
#include "ur_client_library/log.h"

namespace urcl
{
using vector3d_t = std::array<double, 3>;
using vector6d_t = std::array<double, 6>;
using vector6int32_t = std::array<int32_t, 6>;
using vector6uint32_t = std::array<uint32_t, 6>;

/*!
 * \brief A joint configuration (6 joint positions in radians).
 *
 * This is a strong type around a \ref vector6d_t meant to unambiguously express "this 6-tuple
 * represents joint values", as opposed to a Cartesian pose. It is primarily used together with
 * \ref MotionTarget to select between joint-space and Cartesian-space targets when calling
 * motion functions that can accept either.
 *
 * Unlike raw initializer lists (``{...}``) which may bind to either \ref vector6d_t or
 * \ref Pose, wrapping values in ``urcl::Q{...}`` always forces a joint-target interpretation.
 */
struct Q
{
  constexpr Q(double q1, double q2, double q3, double q4, double q5, double q6) : values{ q1, q2, q3, q4, q5, q6 }
  {
  }
  explicit constexpr Q(const vector6d_t& values) : values(values)
  {
  }

  vector6d_t values;
};

struct Pose
{
  Pose() : x(0.0), y(0.0), z(0.0), rx(0.0), ry(0.0), rz(0.0)
  {
  }
  Pose(const double x, const double y, const double z, const double rx, const double ry, const double rz)
    : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz)
  {
  }
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;

  bool operator==(const Pose& other) const
  {
    return x == other.x && y == other.y && z == other.z && rx == other.rx && ry == other.ry && rz == other.rz;
  }
};

/*!
 * \brief A tagged union representing either a joint target (\ref Q) or a Cartesian target
 * (\ref Pose).
 *
 * ``MotionTarget`` is used throughout the motion API (e.g.
 * \ref InstructionExecutor::moveJ "InstructionExecutor::moveJ" and the ``Move*Primitive``
 * constructors) to let callers choose at call site whether a motion should be parametrized in
 * joint space or in Cartesian space without needing separate overloads for every combination.
 *
 * The overloads that take a ``MotionTarget`` are provided alongside explicit \ref vector6d_t and
 * \ref Pose overloads so that plain braced-initializer calls keep binding to the previous
 * behaviour; only explicitly constructed \ref Q or \ref Pose values (or an already-built
 * ``MotionTarget``) select the variant-based path.
 */
using MotionTarget = std::variant<Q, Pose>;

template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& out, const std::array<T, N>& item)
{
  out << "[";
  for (size_t i = 0; i < item.size(); ++i)
  {
    out << item[i];
    if (i != item.size() - 1)
    {
      out << ", ";
    }
  }
  out << "]";
  return out;
}

/*!
 * \brief Converts an enum type to its underlying type
 *
 * \param e Enum value that should be converted
 *
 * \returns Enum value converted to underlying type
 */
template <typename E>
constexpr typename std::underlying_type<E>::type toUnderlying(const E e) noexcept
{
  return static_cast<typename std::underlying_type<E>::type>(e);
}

template <typename FunctionT>
struct HandlerFunction
{
  uint32_t id;
  std::function<FunctionT> function;

  HandlerFunction(uint32_t id, std::function<FunctionT> function) : id(id), function(function)
  {
  }

  bool operator==(const HandlerFunction& other) const
  {
    return id == other.id;
  }
};
}  // namespace urcl
