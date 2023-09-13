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

#include "ur_client_library/ur/watchdog.h"
#include "ur_client_library/log.h"
#include "ur_client_library/exceptions.h"
#include "ur_client_library/types.h"
#include <sstream>

namespace urcl
{
Watchdog::Watchdog()
{
  timeout_ = std::chrono::milliseconds(20);
}

Watchdog Watchdog::millisec(const std::chrono::milliseconds& milliseconds)
{
  Watchdog watch_dog;
  watch_dog.timeout_ = milliseconds;

  return watch_dog;
}

Watchdog Watchdog::off()
{
  Watchdog watch_dog;
  watch_dog.timeout_ = std::chrono::milliseconds(0);
  return watch_dog;
}

std::chrono::milliseconds Watchdog::verifyWatchdogTimeout(const comm::ControlMode& control_mode,
                                                          const std::chrono::milliseconds& step_time) const
{
  // Convert timeout to float
  if (comm::ControlModeTypes::is_control_mode_non_realtime(control_mode))
  {
    if (timeout_ < step_time && timeout_ > std::chrono::milliseconds(0))
    {
      std::stringstream ss;
      ss << "Watchdog timeout " << timeout_.count() << " is below the step time " << step_time.count()
         << ". It will be reset to the step time.";
      URCL_LOG_ERROR(ss.str().c_str());
      return step_time;
    }
    else
    {
      return timeout_;
    }
  }
  else if (comm::ControlModeTypes::is_control_mode_realtime(control_mode))
  {
    const std::chrono::milliseconds max_realtime_timeout = std::chrono::milliseconds(1000);
    if (timeout_ < step_time)
    {
      std::stringstream ss;
      ss << "Realtime read timeout " << timeout_.count() << " is below the step time " << step_time.count()
         << ". It will be reset to the step time.";
      URCL_LOG_ERROR(ss.str().c_str());
      return step_time;
    }
    else if (timeout_ > max_realtime_timeout)
    {
      std::stringstream ss;
      ss << "Watchdog timeout " << timeout_.count() << " is above the maximum allowed timeout for realtime commands "
         << max_realtime_timeout.count() << ". It will be reset to the maximum allowed timeout.";
      URCL_LOG_ERROR(ss.str().c_str());
      return max_realtime_timeout;
    }
    else
    {
      return timeout_;
    }
  }
  else
  {
    std::stringstream ss;
    ss << "Unknown control mode " << toUnderlying(control_mode) << " for verifying the watchdog";
    throw UrException(ss.str().c_str());
  }
}

}  // namespace urcl