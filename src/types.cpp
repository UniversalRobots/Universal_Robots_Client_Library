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

#include <ur_client_library/types.h>

#include <algorithm>

namespace urcl
{
Q::Q(const double q1, const double q2, const double q3, const double q4, const double q5, const double q6)
{
  values_ = { q1, q2, q3, q4, q5, q6 };
}

Q::Q(const vector6d_t& values)
{
  values_.resize(6);
  std::copy(values.begin(), values.end(), values_.begin());
}

const std::vector<double>& Q::getValues() const
{
  return values_;
}

void Q::setValues(const vector6d_t& values)
{
  values_.resize(6);
  std::copy(values.begin(), values.end(), values_.begin());
}

void Q::setValues(const std::vector<double>& values)
{
  if (values.size() != 6)
  {
    throw std::invalid_argument("Q must have exactly 6 values");
  }
  values_ = values;
}

bool operator==(const Q& lhs, const Q& rhs)
{
  return lhs.getValues().size() == rhs.getValues().size() &&
         std::equal(lhs.getValues().begin(), lhs.getValues().end(), rhs.getValues().begin());
}

Pose::Pose() : x(0.0), y(0.0), z(0.0), rx(0.0), ry(0.0), rz(0.0), q_near_(std::nullopt)
{
}

Pose::Pose(const double x, const double y, const double z, const double rx, const double ry, const double rz)
  : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz), q_near_(std::nullopt)
{
}

Pose::Pose(const double x, const double y, const double z, const double rx, const double ry, const double rz,
           const Q& q_near)
  : x(x), y(y), z(z), rx(rx), ry(ry), rz(rz), q_near_(q_near)
{
}

bool Pose::operator==(const Pose& other) const
{
  if (x != other.x || y != other.y || z != other.z || rx != other.rx || ry != other.ry || rz != other.rz)
  {
    return false;
  }
  if (q_near_.has_value() != other.q_near_.has_value())
  {
    return false;
  }
  if (!q_near_.has_value())
  {
    return true;
  }
  return *q_near_ == *other.q_near_;
}

const std::optional<Q>& Pose::getQNear() const
{
  return q_near_;
}

void Pose::setQNear(const Q& q_near)
{
  q_near_ = q_near;
}

bool operator==(const Q& lhs, const vector6d_t& rhs)
{
  return lhs.getValues().size() == rhs.size() &&
         std::equal(lhs.getValues().begin(), lhs.getValues().end(), rhs.begin());
}

void Pose::setPose(const double x, const double y, const double z, const double rx, const double ry, const double rz)
{
  this->x = x;
  this->y = y;
  this->z = z;
  this->rx = rx;
  this->ry = ry;
  this->rz = rz;
}

}  // namespace urcl
