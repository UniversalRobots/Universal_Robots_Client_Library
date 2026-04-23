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

#include "ur_client_library/primary/robot_state/masterboard_data.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

#include <iomanip>
#include <sstream>

namespace urcl
{
namespace primary_interface
{
MasterboardData::MasterboardData(const MasterboardData& pkg) : RobotState(RobotStateType::MASTERBOARD_DATA)
{
  digital_input_bits_ = pkg.digital_input_bits_;
  digital_output_bits_ = pkg.digital_output_bits_;
  analog_input_range_0_ = pkg.analog_input_range_0_;
  analog_input_range_1_ = pkg.analog_input_range_1_;
  analog_input_0_ = pkg.analog_input_0_;
  analog_input_1_ = pkg.analog_input_1_;
  analog_output_domain_0_ = pkg.analog_output_domain_0_;
  analog_output_domain_1_ = pkg.analog_output_domain_1_;
  analog_output_0_ = pkg.analog_output_0_;
  analog_output_1_ = pkg.analog_output_1_;
  master_board_temperature_ = pkg.master_board_temperature_;
  robot_voltage_48v_ = pkg.robot_voltage_48v_;
  robot_current_ = pkg.robot_current_;
  master_io_current_ = pkg.master_io_current_;
  safety_mode_ = pkg.safety_mode_;
  in_reduced_mode_ = pkg.in_reduced_mode_;
  immi_interface_installed_ = pkg.immi_interface_installed_;
  immi_input_bits_ = pkg.immi_input_bits_;
  immi_output_bits_ = pkg.immi_output_bits_;
  immi_voltage_24v_ = pkg.immi_voltage_24v_;
  immi_current_ = pkg.immi_current_;
  reserved_1_ = pkg.reserved_1_;
  operational_mode_selector_input_ = pkg.operational_mode_selector_input_;
  three_position_enabling_device_input_ = pkg.three_position_enabling_device_input_;
  reserved_2_ = pkg.reserved_2_;
}

bool MasterboardData::parseWith(comm::BinParser& bp)
{
  bp.parse<uint32_t>(digital_input_bits_);
  bp.parse<uint32_t>(digital_output_bits_);
  bp.parse(analog_input_range_0_);
  bp.parse(analog_input_range_1_);
  bp.parse(analog_input_0_);
  bp.parse(analog_input_1_);
  bp.parse(analog_output_domain_0_);
  bp.parse(analog_output_domain_1_);
  bp.parse(analog_output_0_);
  bp.parse(analog_output_1_);
  bp.parse(master_board_temperature_);
  bp.parse(robot_voltage_48v_);
  bp.parse(robot_current_);
  bp.parse(master_io_current_);
  bp.parse(safety_mode_);
  bp.parse(in_reduced_mode_);
  bp.parse(immi_interface_installed_);

  if (immi_interface_installed_)
  {
    bp.parse<uint32_t>(immi_input_bits_);
    bp.parse<uint32_t>(immi_output_bits_);
    bp.parse(immi_voltage_24v_);
    bp.parse(immi_current_);
  }

  bp.parse(reserved_1_);
  bp.parse(operational_mode_selector_input_);
  bp.parse(three_position_enabling_device_input_);
  bp.parse(reserved_2_);

  return true;
}

bool MasterboardData::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string MasterboardData::toString() const
{
  std::stringstream os;
  os << "MasterboardData:" << std::endl;
  os << "Digital input bits: 0b" << digital_input_bits_ << std::endl;
  os << "Digital output bits: 0b" << digital_output_bits_ << std::endl;
  os << "Analog input range 0: " << unsigned(analog_input_range_0_) << std::endl;
  os << "Analog input range 1: " << unsigned(analog_input_range_1_) << std::endl;
  os << "Analog input 0: " << analog_input_0_ << std::endl;
  os << "Analog input 1: " << analog_input_1_ << std::endl;
  os << "Analog output domain 0: " << static_cast<int>(analog_output_domain_0_) << std::endl;
  os << "Analog output domain 1: " << static_cast<int>(analog_output_domain_1_) << std::endl;
  os << "Analog output 0: " << analog_output_0_ << std::endl;
  os << "Analog output 1: " << analog_output_1_ << std::endl;
  os << "Masterboard temperature: " << master_board_temperature_ << std::endl;
  os << "Robot voltage 48V: " << robot_voltage_48v_ << std::endl;
  os << "Robot current: " << robot_current_ << std::endl;
  os << "Master I/O current: " << master_io_current_ << std::endl;
  os << "Safety mode: " << unsigned(safety_mode_) << std::endl;
  os << "In reduced mode: " << in_reduced_mode_ << std::endl;
  os << "IMMI interface installed: " << immi_interface_installed_ << std::endl;
  if (immi_interface_installed_)
  {
    os << "IMMI input bits: 0b" << immi_input_bits_ << std::endl;
    os << "IMMI output bits: 0b" << immi_output_bits_ << std::endl;
    os << "IMMI voltage 24V: " << immi_voltage_24v_ << std::endl;
    os << "IMMI current: " << immi_current_ << std::endl;
  }
  os << "Operational mode selector input: " << unsigned(operational_mode_selector_input_) << std::endl;
  os << "Three position enabling device input: " << three_position_enabling_device_input_ << std::endl;

  return os.str();
}

}  // namespace primary_interface
}  // namespace urcl
