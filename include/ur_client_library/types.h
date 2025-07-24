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
#include "ur_client_library/log.h"

namespace urcl
{
using vector3d_t = std::array<double, 3>;
using vector6d_t = std::array<double, 6>;
using vector6int32_t = std::array<int32_t, 6>;
using vector6uint32_t = std::array<uint32_t, 6>;

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
