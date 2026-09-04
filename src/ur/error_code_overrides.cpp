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
 * Dynamic error-code text overrides.
 *
 * This file is intentionally NOT generated or overwritten by
 * scripts/generate_error_codes.py.  Add cases here for error codes
 * whose human-readable text must be computed at runtime (e.g. by
 * calling a C++ helper function rather than returning a static string).
 *
 * For purely static additions or text corrections, prefer adding entries
 * to scripts/error_code_overrides.json instead, which will be merged
 * into the generated lookup table by the code-generation script.
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/ur/error_code_overrides.h"
#include "ur_client_library/ur/datatypes.h"

namespace urcl
{
namespace primary_interface
{
std::optional<std::string> getErrorCodeTextOverride(int32_t code, int32_t arg)
{
  switch (code)
  {
    // C100: Robot changed mode.
    // The argument encodes the new RobotMode; use robotModeString() to generate the human-readable
    // text.
    case 100:
    {
      if (arg < static_cast<int32_t>(RobotMode::UNKNOWN) ||
          arg > static_cast<int32_t>(RobotMode::UPDATING_FIRMWARE))
      {
        return "Robot mode changed to: UNKNOWN (mode=" + std::to_string(arg) + ")";
      }
      try
      {
        return "Robot mode changed to: " + robotModeString(static_cast<RobotMode>(arg));
      }
      catch (const std::invalid_argument&)
      {
        return "Robot mode changed to: UNKNOWN (mode=" + std::to_string(arg) + ")";
      }
    }

    default:
      return std::nullopt;
  }
}

}  // namespace primary_interface
}  // namespace urcl
