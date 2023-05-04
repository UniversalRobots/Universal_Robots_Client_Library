// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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

#ifndef UR_CLIENT_LIBRARY_JOINT_DATA_H_INCLUDED
#define UR_CLIENT_LIBRARY_JOINT_DATA_H_INCLUDED

#include "ur_client_library/types.h"
#include "ur_client_library/primary/robot_state.h"
namespace urcl
{
namespace primary_interface
{
/*!
 * \brief This messages contains data about the joints of the robot and the motors of the joints.
 */
class JointData : public RobotState
{
public:
  JointData() = delete;
  /*!
   * \brief Creates a new JointData object.
   *
   * \param type The type of RobotState message received
   */
  JointData(const RobotStateType type) : RobotState(type)
  {
  }

  /*!
   * \brief Creates a copy of a JointData object.
   *
   * \param pkg The JointData object to be copied
   */
  JointData(const JointData& pkg) : RobotState(RobotStateType::JOINT_DATA)
  {
    q_actual_ = pkg.q_actual_;
    q_target_ = pkg.q_target_;
    qd_actual_ = pkg.qd_actual_;
    i_actual_ = pkg.i_actual_;
    v_actual_ = pkg.v_actual_;
    t_motor_ = pkg.t_motor_;
    t_micro_ = pkg.t_micro_;
    joint_mode_ = pkg.joint_mode_;
  }
  virtual ~JointData() = default;

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

  vector6d_t q_actual_;
  vector6d_t q_target_;
  vector6d_t qd_actual_;
  vector6f_t i_actual_;
  vector6f_t v_actual_;
  vector6f_t t_motor_;
  vector6f_t t_micro_;  // Deprecated
  vector6uint8_t joint_mode_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_JOINT_DATA_H_INCLUDED
