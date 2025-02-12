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

#ifndef UR_CLIENT_LIBRARY_ROBOT_MODE_DATA_H_INCLUDED
#define UR_CLIENT_LIBRARY_ROBOT_MODE_DATA_H_INCLUDED

#include <stdexcept>
#include "ur_client_library/types.h"
#include "ur_client_library/primary/robot_state.h"
namespace urcl
{
namespace primary_interface
{

/*!
 * \brief This messages contains data about the mode of the robot.
 */
class RobotModeData : public RobotState
{
public:
  RobotModeData() = delete;
  /*!
   * \brief Creates a new RobotModeData object.
   *
   * \param type The type of RobotState message received
   */
  RobotModeData(const RobotStateType type) : RobotState(type)
  {
  }

  /*!
   * \brief Creates a copy of a RobotModeData object.
   *
   * \param pkg The RobotModeData object to be copied
   */
  RobotModeData(const RobotModeData& pkg);

  virtual ~RobotModeData() = default;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);

  /*!
   * \brief Consume this specific package with a specific consumer.
   *
   * \param consumer Placeholder for the consumer calling this
   *
   * \returns true on success
   */
  virtual bool consumeWith(AbstractPrimaryConsumer& consumer);

  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  uint64_t timestamp_;
  bool is_real_robot_connected_;
  bool is_real_robot_enabled_;
  bool is_robot_power_on_;
  bool is_emergency_stopped_;
  bool is_protective_stopped_;
  bool is_program_running_;
  bool is_program_paused_;
  int8_t robot_mode_;
  uint8_t control_mode_;
  double target_speed_fraction_;
  double speed_scaling_;
  double target_speed_fraction_limit_;
  std::string reserved_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_ROBOT_MODE_DATA_H_INCLUDED
