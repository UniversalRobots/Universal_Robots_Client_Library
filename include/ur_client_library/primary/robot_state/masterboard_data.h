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

#ifndef UR_CLIENT_LIBRARY_MASTERBOARD_DATA_H_INCLUDED
#define UR_CLIENT_LIBRARY_MASTERBOARD_DATA_H_INCLUDED

#include <bitset>

#include "ur_client_library/types.h"
#include "ur_client_library/primary/robot_state.h"

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief The MasterboardData class handles the masterboard data sub-package sent as part of the
 * Robot State message on the primary UR interface.
 *
 * This package contains information about the control box IOs, analog inputs/outputs, voltages,
 * currents, temperatures, the safety mode, as well as information about the optional IMMI
 * interface. See the Universal Robots primary/secondary guide for details:
 * https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html
 */
class MasterboardData : public RobotState
{
public:
  MasterboardData() = delete;

  /*!
   * \brief Creates a new MasterboardData object.
   *
   * \param type The type of RobotState message received
   */
  MasterboardData(const RobotStateType type) : RobotState(type)
  {
  }

  /*!
   * \brief Creates a copy of a MasterboardData object.
   *
   * \param pkg The MasterboardData object to be copied
   */
  MasterboardData(const MasterboardData& pkg);

  virtual ~MasterboardData() = default;

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

  std::bitset<32> digital_input_bits_;
  std::bitset<32> digital_output_bits_;
  uint8_t analog_input_range_0_;
  uint8_t analog_input_range_1_;
  double analog_input_0_;
  double analog_input_1_;
  char analog_output_domain_0_;
  char analog_output_domain_1_;
  double analog_output_0_;
  double analog_output_1_;
  float master_board_temperature_;
  float robot_voltage_48v_;
  float robot_current_;
  float master_io_current_;
  uint8_t safety_mode_;
  bool in_reduced_mode_;
  bool immi_interface_installed_{ false };

  // The following four fields are only valid if immi_interface_installed_ is true.
  std::bitset<32> immi_input_bits_;
  std::bitset<32> immi_output_bits_;
  float immi_voltage_24v_{ 0.0f };
  float immi_current_{ 0.0f };

  uint32_t reserved_1_{ 0 };
  uint8_t operational_mode_selector_input_{ 0 };
  bool three_position_enabling_device_input_{ false };
  uint8_t reserved_2_{ 0 };
};

}  // namespace primary_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_MASTERBOARD_DATA_H_INCLUDED
