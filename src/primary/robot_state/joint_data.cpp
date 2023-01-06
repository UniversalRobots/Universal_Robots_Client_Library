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

#include "ur_client_library/log.h"
#include "ur_client_library/primary/robot_state/joint_data.h"
#include "ur_client_library/primary/abstract_primary_consumer.h"

#include <iomanip>

namespace urcl
{
namespace primary_interface
{
bool JointData::parseWith(comm::BinParser& bp)
{
  for (size_t i = 0; i < q_actual_.size(); i++)
  {
    bp.parse(q_actual_.at(i));
    bp.parse(q_target_.at(i));
    bp.parse(qd_actual_.at(i));
    bp.parse(i_actual_.at(i));
    bp.parse(v_actual_.at(i));
    bp.parse(t_motor_.at(i));
    bp.parse(t_micro_.at(i));  // Deprecated
    bp.parse(joint_mode_.at(i));
  }

  return true;
}

bool JointData::consumeWith(AbstractPrimaryConsumer& consumer)
{
  return consumer.consume(*this);
}

std::string JointData::toString() const
{
  std::stringstream os;
  os << "q_actual: " << q_actual_ << std::endl;

  os << "q_target: " << q_target_ << std::endl;

  os << "qd_actual: " << qd_actual_ << std::endl;

  os << "I_actual: " << i_actual_ << std::endl;

  os << "V_actual: " << v_actual_ << std::endl;

  os << "T_motor: " << t_motor_ << std::endl;

  os << "joint_mode: [";
  for (size_t i = 0; i < joint_mode_.size(); ++i)
  {
    os << unsigned(joint_mode_[i]) << " ";
  }
  os << "]" << std::endl;

  return os.str();
}

}  // namespace primary_interface
}  // namespace urcl
