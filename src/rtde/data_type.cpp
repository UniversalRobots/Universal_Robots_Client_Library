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

#include "ur_client_library/rtde/data_type.h"

#include <type_traits>

#include "ur_client_library/exceptions.h"
#include "ur_client_library/helpers.h"
#include "ur_client_library/log.h"

namespace urcl
{
namespace rtde_interface
{
namespace
{
/*!
 * \brief The RTDE protocol's name for each data type.
 *
 * The single place the spellings live. Both directions of the name conversion read from it, so a
 * name can never disagree with itself.
 */
constexpr struct
{
  DataType type;
  std::string_view name;
} TYPE_NAMES[] = {
  { DataType::BOOL, "BOOL" },
  { DataType::UINT8, "UINT8" },
  { DataType::UINT32, "UINT32" },
  { DataType::UINT64, "UINT64" },
  { DataType::INT32, "INT32" },
  { DataType::DOUBLE, "DOUBLE" },
  { DataType::VECTOR3D, "VECTOR3D" },
  { DataType::VECTOR6D, "VECTOR6D" },
  { DataType::VECTOR6INT32, "VECTOR6INT32" },
  { DataType::VECTOR6UINT32, "VECTOR6UINT32" },
};
}  // namespace

std::string toString(const DataType type)
{
  for (const auto& entry : TYPE_NAMES)
  {
    if (entry.type == type)
    {
      return std::string(entry.name);
    }
  }
  throw UrException("Unhandled RTDE data type.");
}

std::optional<DataType> dataTypeFromName(const std::string_view type_name)
{
  for (const auto& entry : TYPE_NAMES)
  {
    if (entry.name == type_name)
    {
      return entry.type;
    }
  }
  return std::nullopt;
}

bool parseDataTypes(const std::string_view list, std::vector<std::string_view>& names,
                    std::vector<std::optional<DataType>>& types)
{
  splitStringView(list, ",", names);
  types.clear();
  bool all_data_types = true;
  for (const std::string_view name : names)
  {
    const std::optional<DataType> type = dataTypeFromName(name);
    if (!type.has_value() && name != NOT_FOUND_NAME && name != NOT_SET_NAME && name != IN_USE_NAME)
    {
      URCL_LOG_WARN("The robot reported an unknown RTDE data type '%.*s'", static_cast<int>(name.size()), name.data());
    }
    all_data_types &= type.has_value();
    types.push_back(type);
  }
  return all_data_types;
}

std::string knownTypeNames()
{
  std::string names;
  for (const auto& entry : TYPE_NAMES)
  {
    if (!names.empty())
    {
      names += ", ";
    }
    names += entry.name;
  }
  return names;
}

std::optional<DataType> dataTypeOf(const DataValue& value)
{
  if (std::holds_alternative<bool>(value))
  {
    return DataType::BOOL;
  }
  if (std::holds_alternative<uint8_t>(value))
  {
    return DataType::UINT8;
  }
  if (std::holds_alternative<uint32_t>(value))
  {
    return DataType::UINT32;
  }
  if (std::holds_alternative<uint64_t>(value))
  {
    return DataType::UINT64;
  }
  if (std::holds_alternative<int32_t>(value))
  {
    return DataType::INT32;
  }
  if (std::holds_alternative<double>(value))
  {
    return DataType::DOUBLE;
  }
  if (std::holds_alternative<vector3d_t>(value))
  {
    return DataType::VECTOR3D;
  }
  if (std::holds_alternative<vector6d_t>(value))
  {
    return DataType::VECTOR6D;
  }
  if (std::holds_alternative<vector6int32_t>(value))
  {
    return DataType::VECTOR6INT32;
  }
  if (std::holds_alternative<vector6uint32_t>(value))
  {
    return DataType::VECTOR6UINT32;
  }
  return std::nullopt;
}

namespace
{
// Switching over the enum rather than testing names in sequence means the compiler points at this
// function if a data type is ever added to the protocol. A value outside the enum yields
// std::monostate.
DataValue zeroValue(const DataType type)
{
  switch (type)
  {
    case DataType::BOOL:
      return bool();
    case DataType::UINT8:
      return uint8_t();
    case DataType::UINT32:
      return uint32_t();
    case DataType::UINT64:
      return uint64_t();
    case DataType::INT32:
      return int32_t();
    case DataType::DOUBLE:
      return double();
    case DataType::VECTOR3D:
      return vector3d_t();
    case DataType::VECTOR6D:
      return vector6d_t();
    case DataType::VECTOR6INT32:
      return vector6int32_t();
    case DataType::VECTOR6UINT32:
      return vector6uint32_t();
  }
  return std::monostate();
}
}  // namespace

bool isDataType(const DataType type)
{
  return !std::holds_alternative<std::monostate>(zeroValue(type));
}

DataValue makeValue(const DataType type)
{
  DataValue value = zeroValue(type);
  if (std::holds_alternative<std::monostate>(value))
  {
    throw UrException("Unhandled RTDE data type.");
  }
  return value;
}

void parseValue(comm::BinParser& bp, const DataType type, DataValue& value)
{
  value = makeValue(type);
  std::visit(
      [&bp](auto&& arg) {
        if constexpr (!std::is_same_v<std::decay_t<decltype(arg)>, std::monostate>)
        {
          bp.parse(arg);
        }
      },
      value);
}

}  // namespace rtde_interface
}  // namespace urcl
