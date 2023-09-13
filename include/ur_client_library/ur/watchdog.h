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

#pragma once

#include "ur_client_library/comm/control_mode.h"
#include <chrono>

namespace urcl
{
/*!
 * \brief Watchdog class containing a timeout configuration
 *
 * This watchdog is used to configure the read timeout for the reverse socket running in the external control script.
 * The read timeout is the number of milliseconds until the read action times out. A timeout of 0 or negative number
 * indicates that the function should not return until a read is completed, this will make the read function blocking on
 * the robot. This can be set using the function off().
 *
 */
class Watchdog
{
public:
  Watchdog();
  ~Watchdog() = default;

  /*!
   * \brief Create a watchdog object with a specific timeout
   *
   * \param milliseconds watchdog timeout
   *
   * \returns Watchdog object
   */
  static Watchdog millisec(const std::chrono::milliseconds& milliseconds = std::chrono::milliseconds(20));

  /*!
   * \brief Creates a watchdog object with no timeout
   *
   * \returns watchdog object
   */
  static Watchdog off();

  /*!
   * \brief Helper function to verify that the watchdog timeout is configured appropriately given the current control
   * mode
   *
   * \param control_mode current control mode
   * \param step_time The robots step time
   *
   * \returns watchdog timeout
   */
  std::chrono::milliseconds verifyWatchdogTimeout(const comm::ControlMode& control_mode,
                                                  const std::chrono::milliseconds& step_time) const;

  std::chrono::milliseconds timeout_;
};

}  // namespace urcl