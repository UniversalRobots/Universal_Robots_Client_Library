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

#ifndef UR_CLIENT_LIBRARY_RTDE_DATA_TYPE_H_INCLUDED
#define UR_CLIENT_LIBRARY_RTDE_DATA_TYPE_H_INCLUDED

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "ur_client_library/comm/bin_parser.h"
#include "ur_client_library/types.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief The data types an RTDE field can have.
 *
 * This is the complete set the protocol defines. Which one a given field has is decided by the
 * robot when it acknowledges a recipe, so this list is all the type knowledge the library needs to
 * carry; see DataPackage::getDataType().
 */
enum class DataType : uint8_t
{
  BOOL,
  UINT8,
  UINT32,
  UINT64,
  INT32,
  DOUBLE,
  VECTOR3D,
  VECTOR6D,
  VECTOR6INT32,
  VECTOR6UINT32
};

/*!
 * \brief A value of one of the RTDE data types.
 *
 * The typed alternatives are exactly the members of DataType. std::monostate is the state of a
 * value whose type isn't decided yet.
 */
using DataValue = std::variant<std::monostate, bool, uint8_t, uint32_t, uint64_t, int32_t, double, vector3d_t,
                               vector6d_t, vector6int32_t, vector6uint32_t>;

/*!
 * \brief The name the RTDE protocol uses for a data type, e.g. "VECTOR6D".
 *
 * This is the spelling the robot uses on the wire and the RTDE guide uses in its field tables.
 */
std::string toString(const DataType type);

/*!
 * \brief The data type with the given RTDE wire name, or an empty optional for any other name.
 */
std::optional<DataType> dataTypeFromName(std::string_view type_name);

/// What the robot sends in place of a data type for a field it does not know.
constexpr std::string_view NOT_FOUND_NAME = "NOT_FOUND";
/// What the robot sends in place of a data type for a property that has no value.
constexpr std::string_view NOT_SET_NAME = "NOT_SET";
/// What the robot sends in place of a data type for an input field another client already writes.
constexpr std::string_view IN_USE_NAME = "IN_USE";

/*!
 * \brief Parses a comma-separated list of data type names, as the robot sends it in a setup or
 * RTDE_READ_PROPERTIES answer.
 *
 * Entry i of \p types is empty where \p names[i] is not a data type, so a caller can still tell
 * NOT_FOUND from IN_USE. A word that is neither a data type nor NOT_FOUND, NOT_SET or IN_USE is
 * logged as a warning. Both vectors are cleared and refilled, so parsing a list no longer than the
 * previous one does not allocate.
 *
 * \param list The comma-separated names. \p names views into it, so it must outlive \p names.
 * \param names The names, one per entry
 * \param types The data types, one per entry
 *
 * \returns True if every entry is a data type
 */
bool parseDataTypes(std::string_view list, std::vector<std::string_view>& names,
                    std::vector<std::optional<DataType>>& types);

/*!
 * \brief All data type names, comma-separated, for error messages.
 */
std::string knownTypeNames();

/*!
 * \brief The data type \p value holds, or an empty optional if it has none yet.
 */
std::optional<DataType> dataTypeOf(const DataValue& value);

/*!
 * \brief A zero value of \p type.
 */
DataValue makeValue(DataType type);

/*!
 * \brief Makes \p value hold \p type and parses it from \p bp.
 */
void parseValue(comm::BinParser& bp, DataType type, DataValue& value);

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_RTDE_DATA_TYPE_H_INCLUDED
