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

#ifndef UR_CLIENT_LIBRARY_CARTESIAN_INFO_H_INCLUDED
#define UR_CLIENT_LIBRARY_CARTESIAN_INFO_H_INCLUDED

#include "ur_client_library/types.h"
#include "ur_client_library/primary/robot_state.h"
namespace urcl
{
namespace primary_interface
{
/*!
 * \brief This messages contains cartesian information on the robot. The flange_coordinates vector is the flange
 * coordinates of the robot and the tcp_offset_coordinates is the coordinates of the tcp offset. Both vectors are x, y,
 * z, rx, ry, rz.
 */
class CartesianInfo : public RobotState
{
public:
  CartesianInfo() = delete;
  /*!
   * \brief Creates a new CartesianInfo object.
   *
   * \param type The type of RobotState message received
   */
  CartesianInfo(const RobotStateType type) : RobotState(type)
  {
  }

  /*!
   * \brief Creates a copy of a CartesianInfo object.
   *
   * \param pkg The CartesianInfo object to be copied
   */
  CartesianInfo(const CartesianInfo& pkg) : RobotState(RobotStateType::CARTESIAN_INFO)
  {
    flange_coordinates_ = pkg.flange_coordinates_;
    tcp_offset_coordinates_ = pkg.tcp_offset_coordinates_;
  }

  virtual ~CartesianInfo() = default;

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

  /*!
   * \brief Calculates a hash value of the parameters to allow for identification of a calibration.
   *
   * \returns A hash value of the parameters
   */
  //   std::string toHash() const;

  vector6d_t flange_coordinates_;
  vector6d_t tcp_offset_coordinates_;
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_CARTESIAN_INFO_H_INCLUDED
