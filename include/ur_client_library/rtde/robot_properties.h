// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

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
//    * Neither the name of the copyright holder nor the names of its
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Universal Robots A/S
 * \date    2026-09-25
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_RTDE_ROBOT_PROPERTIES_H_INCLUDED
#define UR_CLIENT_LIBRARY_RTDE_ROBOT_PROPERTIES_H_INCLUDED

#include <mutex>
#include <optional>

#include "ur_client_library/comm/producer.h"
#include "ur_client_library/comm/stream.h"
#include "ur_client_library/rtde/read_properties.h"
#include "ur_client_library/rtde/rtde_package.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief The robot properties read while setting up RTDE communication.
 *
 * RTDEClient calls fetch() during its handshake. Applications read a snapshot through
 * RTDEClient::getRobotProperties().
 */
class RobotProperties
{
public:
  /*!
   * \brief Reads v1.software.version and then every catalog property that version supports.
   *
   * Only for the setup phase, before any data is streamed: nothing else may read from \p producer
   * meanwhile. The read only succeeds if the controller sends a value for every name, the software
   * version included. A failed read forgets any properties fetched earlier.
   *
   * \returns False if the controller kept sending other packages without answering. The answer
   * may then still be in the stream, so the connection has to be set up again. A read that fails
   * in any other way returns true, since the properties are optional.
   */
  bool fetch(comm::URStream<RTDEPackage>& stream, comm::URProducer<RTDEPackage>& producer);

  /*!
   * \brief Forgets any properties fetched earlier.
   */
  void clear();

  /*!
   * \brief Returns an independent snapshot of the fetched properties.
   *
   * \returns An empty optional if no properties were fetched.
   */
  std::optional<ReadProperties> get() const;

private:
  mutable std::mutex mutex_;
  std::optional<ReadProperties> properties_;
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_RTDE_ROBOT_PROPERTIES_H_INCLUDED
