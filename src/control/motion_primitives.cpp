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
}  // namespace urcl::control