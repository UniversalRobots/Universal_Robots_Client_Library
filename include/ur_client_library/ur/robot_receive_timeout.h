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
 * \brief RobotReceiveTimeout class containing a timeout configuration
 *
 * This robot receive timeout is used to configure the read timeout for the reverse socket running in the external
 * control script. The read timeout is the number of milliseconds until the read action times out. A timeout of 0 or
 * negative number indicates that the function should not return until a read is completed, this will make the read
 * function on the robot blocking. This can be set using the function off().
 *
 */
class RobotReceiveTimeout
{
public:
  static constexpr std::chrono::milliseconds MAX_RT_RECEIVE_TIMEOUT_MS = std::chrono::milliseconds(200);
  RobotReceiveTimeout() = delete;
  ~RobotReceiveTimeout() = default;

  /*!
   * \brief Create a RobotReceiveTimeout object with a specific timeout given in milliseconds
   *
   * \param milliseconds robot receive timeout
   *
   * \returns RobotReceiveTimeout object
   */
  static RobotReceiveTimeout millisec(const unsigned int milliseconds = 20);

  /*!
   * \brief Create a RobotReceiveTimeout object with a specific timeout given in seconds
   *
   * \param seconds robot receive timeout
   *
   * \returns RobotReceiveTimeout object
   */
  static RobotReceiveTimeout sec(const float seconds = 0.02);

  /*!
   * \brief Creates a RobotReceiveTimeout object with no timeout, this will make the read function on the robot blocking
   *
   * \returns RobotReceiveTimeout object
   */
  static RobotReceiveTimeout off();

  /*!
   * \brief Helper function to verify that the robot receive timeout is configured appropriately given the current
   * control mode
   *
   * \param control_mode current control mode
   * \param step_time The robots step time
   *
   * \returns receive timeout in milliseconds
   */
  int verifyRobotReceiveTimeout(const comm::ControlMode control_mode, const std::chrono::milliseconds step_time) const;

  std::chrono::milliseconds getAsMilliseconds() const
  {
    return timeout_;
  }

private:
  std::chrono::milliseconds timeout_;
  RobotReceiveTimeout(std::chrono::milliseconds timeout);
};

}  // namespace urcl
