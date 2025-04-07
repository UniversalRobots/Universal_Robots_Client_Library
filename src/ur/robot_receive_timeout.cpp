// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2023 Universal Robots A/S
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

#include "ur_client_library/ur/robot_receive_timeout.h"
#include "ur_client_library/log.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/types.h"

#include <sstream>

namespace urcl
{
RobotReceiveTimeout::RobotReceiveTimeout(std::chrono::milliseconds timeout) : timeout_(timeout)
{
}

RobotReceiveTimeout RobotReceiveTimeout::millisec(const unsigned int milliseconds)
{
  std::chrono::milliseconds timeout = std::chrono::milliseconds(milliseconds);
  RobotReceiveTimeout robot_receive_timeout(timeout);
  return robot_receive_timeout;
}

RobotReceiveTimeout RobotReceiveTimeout::sec(const float seconds)
{
  std::chrono::milliseconds timeout = std::chrono::milliseconds(static_cast<int>(seconds * 1000));
  RobotReceiveTimeout robot_receive_timeout(timeout);
  return robot_receive_timeout;
}

RobotReceiveTimeout RobotReceiveTimeout::off()
{
  RobotReceiveTimeout robot_receive_timeout(std::chrono::milliseconds(0));
  return robot_receive_timeout;
}

int RobotReceiveTimeout::verifyRobotReceiveTimeout(const comm::ControlMode control_mode,
                                                   const std::chrono::milliseconds step_time) const
{
  if (comm::ControlModeTypes::isControlModeNonRealtime(control_mode))
  {
    if (timeout_ < step_time && timeout_ > std::chrono::milliseconds(0))
    {
      std::stringstream ss;
      ss << "Robot receive timeout " << timeout_.count() << "ms is below the step time " << step_time.count()
         << "ms. It will be reset to the step time.";
      URCL_LOG_ERROR(ss.str().c_str());
      return step_time.count();
    }
    else
    {
      return timeout_.count();
    }
  }
  else if (comm::ControlModeTypes::isControlModeRealtime(control_mode))
  {
    if (timeout_ < step_time)
    {
      std::stringstream ss;
      ss << "Realtime read timeout " << timeout_.count() << "ms is below the step time " << step_time.count()
         << ". It will be reset to the step time.";
      URCL_LOG_ERROR(ss.str().c_str());
      return step_time.count();
    }
    else if (timeout_ > MAX_RT_RECEIVE_TIMEOUT_MS)
    {
      std::stringstream ss;
      ss << "Robot receive timeout " << timeout_.count()
         << "ms is above the maximum allowed timeout for realtime commands " << MAX_RT_RECEIVE_TIMEOUT_MS.count()
         << ". It will be reset to the maximum allowed timeout.";
      URCL_LOG_ERROR(ss.str().c_str());
      return MAX_RT_RECEIVE_TIMEOUT_MS.count();
    }
    else
    {
      return timeout_.count();
    }
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown control mode " << toUnderlying(control_mode) << " for verifying the robot receive timeout";
    throw UrException(ss.str().c_str());
  }
}

}  // namespace urcl