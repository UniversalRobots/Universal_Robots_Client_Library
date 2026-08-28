// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/rtde/data_package.h"

#include <functional>

#include "ur_client_library/exceptions.h"

namespace urcl
{
namespace rtde_interface
{
namespace
{
/*!
 * \brief Whether the alternative a visitor was handed is the "type not decided yet" one.
 *
 * The visitors below are only reached on typed packages, but they still have to compile for every
 * alternative of the variant.
 */
template <typename T>
constexpr bool is_untyped_v = std::is_same_v<std::decay_t<T>, std::monostate>;

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
} g_type_names[] = {
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

/*!
 * \brief The data type a field holds, or an empty optional if it has none yet.
 */
std::optional<DataType> typeOf(const DataPackage::_rtde_type_variant& field)
{
  if (std::holds_alternative<bool>(field))
  {
    return DataType::BOOL;
  }
  if (std::holds_alternative<uint8_t>(field))
  {
    return DataType::UINT8;
  }
  if (std::holds_alternative<uint32_t>(field))
  {
    return DataType::UINT32;
  }
  if (std::holds_alternative<uint64_t>(field))
  {
    return DataType::UINT64;
  }
  if (std::holds_alternative<int32_t>(field))
  {
    return DataType::INT32;
  }
  if (std::holds_alternative<double>(field))
  {
    return DataType::DOUBLE;
  }
  if (std::holds_alternative<vector3d_t>(field))
  {
    return DataType::VECTOR3D;
  }
  if (std::holds_alternative<vector6d_t>(field))
  {
    return DataType::VECTOR6D;
  }
  if (std::holds_alternative<vector6int32_t>(field))
  {
    return DataType::VECTOR6INT32;
  }
  if (std::holds_alternative<vector6uint32_t>(field))
  {
    return DataType::VECTOR6UINT32;
  }
  return std::nullopt;
}

/*!
 * \brief Names the type a field holds for an error message, even if it has none.
 */
std::string typeNameOf(const DataPackage::_rtde_type_variant& field)
{
  const std::optional<DataType> type = typeOf(field);
  return type.has_value() ? toString(*type) : "unknown";
}

/*!
 * \brief Creates an empty value of the given data type.
 *
 * Switching over the enum rather than testing names in sequence means the compiler points at this
 * function if a data type is ever added to the protocol.
 */
DataPackage::_rtde_type_variant variantFor(const DataType type)
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
  throw UrException("Unhandled RTDE data type.");
}

/*!
 * \brief Creates an empty value of the RTDE data type with the given name.
 *
 * \param type_name One of the RTDE data type names as reported by the robot in a setup
 * acknowledgement
 *
 * \throws UrException if the name is not a known RTDE data type
 */
DataPackage::_rtde_type_variant variantFromTypeName(const std::string_view type_name)
{
  for (const auto& entry : g_type_names)
  {
    if (entry.name == type_name)
    {
      return variantFor(entry.type);
    }
  }

  std::stringstream ss;
  ss << "'" << type_name
     << "' is not a known RTDE data type. Expected one of BOOL, UINT8, UINT32, UINT64, INT32, "
        "DOUBLE, VECTOR3D, VECTOR6D, VECTOR6INT32 or VECTOR6UINT32.";
  throw UrException(ss.str());
}
}  // namespace

std::string toString(const DataType type)
{
  for (const auto& entry : g_type_names)
  {
    if (entry.type == type)
    {
      return std::string(entry.name);
    }
  }
  throw UrException("Unhandled RTDE data type.");
}

std::optional<rtde_interface::DataType> rtde_interface::DataPackage::getDataType(const std::string_view name) const
{
  const auto it =
      std::find_if(data_.begin(), data_.end(), [&name](const std::pair<std::string, _rtde_type_variant>& element) {
        return element.first == name;
      });
  if (it == data_.end())
  {
    return std::nullopt;
  }
  return typeOf(it->second);
}

void rtde_interface::DataPackage::reportReadFailure(const std::string_view name, const _rtde_type_variant& field)
{
  if (std::holds_alternative<std::monostate>(field))
  {
    URCL_LOG_ERROR("Cannot read the data field '%.*s', as its data type isn't known yet. The data types of a recipe "
                   "are reported by the robot during the RTDE handshake, so a data package can only be read from "
                   "after it has received data at least once.",
                   static_cast<int>(name.size()), name.data());
    return;
  }
  URCL_LOG_ERROR("Type of requested data doesn't match type of existing field for index '%.*s'. The robot reports "
                 "that field as %s.",
                 static_cast<int>(name.size()), name.data(), typeNameOf(field).c_str());
}

void rtde_interface::DataPackage::initStorage()
{
  data_.resize(recipe_.size());
  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    data_[i].first = recipe_[i];
    data_[i].second = std::monostate();
  }
}

void rtde_interface::DataPackage::initEmpty(const std::vector<std::string>& types)
{
  if (types.size() != recipe_.size())
  {
    std::stringstream ss;
    ss << "Cannot initialize an RTDE data package: got " << types.size() << " data types for a recipe with "
       << recipe_.size() << " fields.";
    throw UrException(ss.str());
  }

  // The storage was allocated by the constructor and every RTDE type lives inline in the variant,
  // so deciding the types here cannot allocate. That is what makes it safe to type a package that
  // an application is already holding, in the middle of a real-time loop.
  if (data_.size() != recipe_.size())
  {
    initStorage();
  }
  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    data_[i].second = variantFromTypeName(types[i]);
  }
}

void rtde_interface::DataPackage::initEmpty()
{
  for (auto& item : data_)
  {
    std::visit([](auto&& arg) { arg = std::decay_t<decltype(arg)>(); }, item.second);
  }
}

bool rtde_interface::DataPackage::parseWith(comm::BinParser& bp)
{
  if (!isTyped())
  {
    URCL_LOG_ERROR("Cannot parse into an RTDE data package before the data types of its recipe are known. Those are "
                   "reported by the robot during the RTDE handshake.");
    return false;
  }

  if (protocol_version_ == 2)
  {
    bp.parse(recipe_id_);
  }
  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    std::visit(
        [&bp](auto&& arg) {
          if constexpr (!is_untyped_v<decltype(arg)>)
          {
            bp.parse(arg);
          }
        },
        data_[i].second);
  }
  return true;
}

std::string rtde_interface::DataPackage::toString() const
{
  std::stringstream ss;
  for (auto& item : data_)
  {
    ss << item.first << ": ";
    if (std::holds_alternative<uint8_t>(item.second))
    {
      ss << int(std::get<uint8_t>(item.second));
    }
    else
    {
      std::visit(
          [&ss](auto&& arg) {
            if constexpr (is_untyped_v<decltype(arg)>)
            {
              ss << "<type not known yet>";
            }
            else
            {
              ss << arg;
            }
          },
          item.second);
    }
    ss << std::endl;
  }
  return ss.str();
}

size_t rtde_interface::DataPackage::serializePackage(uint8_t* buffer)
{
  if (!isTyped())
  {
    URCL_LOG_ERROR("Cannot serialize an RTDE data package before the data types of its recipe are known. Those are "
                   "reported by the robot during the RTDE handshake.");
    return 0;
  }

  uint16_t payload_size = sizeof(recipe_id_);

  for (auto& item : data_)
  {
    payload_size += std::visit(
        [](auto&& arg) -> uint16_t {
          if constexpr (is_untyped_v<decltype(arg)>)
          {
            return 0;
          }
          else
          {
            return sizeof(arg);
          }
        },
        item.second);
  }
  size_t size = 0;
  size += PackageHeader::serializeHeader(buffer, PackageType::RTDE_DATA_PACKAGE, payload_size);
  size += comm::PackageSerializer::serialize(buffer + size, recipe_id_);
  for (size_t i = 0; i < data_.size(); ++i)
  {
    size += std::visit(
        [&buffer, &size](auto&& arg) -> size_t {
          if constexpr (is_untyped_v<decltype(arg)>)
          {
            return 0;
          }
          else
          {
            return comm::PackageSerializer::serialize(buffer + size, arg);
          }
        },
        data_[i].second);
  }

  return size;
}

bool rtde_interface::DataPackage::resetData(const std::string_view name)
{
  const auto it =
      std::find_if(data_.begin(), data_.end(), [&name](const std::pair<std::string, _rtde_type_variant>& element) {
        return element.first == name;
      });
  if (it == data_.end())
  {
    return false;
  }
  std::visit([](auto&& arg) { arg = std::decay_t<decltype(arg)>(); }, it->second);
  return true;
}

bool rtde_interface::DataPackage::copySetFieldsFrom(const DataPackage& other)
{
  bool all_copied = true;
  for (const auto& source : other.data_)
  {
    if (std::holds_alternative<std::monostate>(source.second))
    {
      continue;
    }

    const auto destination =
        std::find_if(data_.begin(), data_.end(), [&source](const std::pair<std::string, _rtde_type_variant>& element) {
          return element.first == source.first;
        });
    if (destination == data_.end())
    {
      URCL_LOG_ERROR("The data field '%s' is not part of the recipe the robot acknowledged.", source.first.c_str());
      all_copied = false;
      continue;
    }
    if (source.second.index() != destination->second.index())
    {
      URCL_LOG_ERROR("The value passed for the data field '%s' is of type %s, but the robot reports that field as %s.",
                     source.first.c_str(), typeNameOf(source.second).c_str(), typeNameOf(destination->second).c_str());
      all_copied = false;
      continue;
    }
    destination->second = source.second;
  }
  return all_copied;
}
}  // namespace rtde_interface
}  // namespace urcl
