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
 * Declaration of the dynamic error-code text override hook.
 *
 * Edit src/primary/robot_message/error_code_overrides.cpp to add
 * runtime-computed descriptions for specific error codes.  That file
 * is intentionally NOT touched by the code-generation script.
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PRIMARY_ERROR_CODE_OVERRIDES_H_INCLUDED
#define UR_CLIENT_LIBRARY_PRIMARY_ERROR_CODE_OVERRIDES_H_INCLUDED

#include <cstdint>
#include <optional>
#include <string>

namespace urcl
{
namespace primary_interface
{
/*!
 * \brief Optional dynamic override for an error code text.
 *
 * Called by ErrorCodeMessage::toString() before the static lookup table.
 * Return a non-empty optional to supply a custom description; return
 * std::nullopt to fall through to the generated table.
 *
 * Implement additional cases in
 * src/primary/robot_message/error_code_overrides.cpp.
 *
 * \param code    The error code (message_code_)
 * \param arg     The error argument (message_argument_)
 * \returns       A human-readable string, or std::nullopt
 */
std::optional<std::string> getErrorCodeTextOverride(int32_t code, int32_t arg);

}  // namespace primary_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_PRIMARY_ERROR_CODE_OVERRIDES_H_INCLUDED
