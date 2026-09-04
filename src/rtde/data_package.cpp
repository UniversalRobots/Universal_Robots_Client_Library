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

#include <algorithm>
#include <cstring>

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

constexpr uint64_t g_FNV_OFFSET_BASIS = 14695981039346656037ULL;
constexpr uint64_t g_FNV_PRIME = 1099511628211ULL;

uint64_t fnv1a(uint64_t hash, const uint8_t* data, const size_t length)
{
  for (size_t i = 0; i < length; ++i)
  {
    hash ^= data[i];
    hash *= g_FNV_PRIME;
  }
  return hash;
}

uint64_t fnv1aByte(uint64_t hash, const uint8_t value)
{
  hash ^= value;
  hash *= g_FNV_PRIME;
  return hash;
}

uint64_t hashRecipe(const std::vector<std::string>& recipe)
{
  uint64_t hash = g_FNV_OFFSET_BASIS;
  const uint64_t count = recipe.size();
  hash = fnv1a(hash, reinterpret_cast<const uint8_t*>(&count), sizeof(count));
  for (const auto& name : recipe)
  {
    hash = fnv1a(hash, reinterpret_cast<const uint8_t*>(name.data()), name.size());
    // A separator so that "ab"+"c" and "a"+"bc" cannot produce the same digest.
    hash = fnv1aByte(hash, 0);
  }
  return hash;
}

uint64_t hashLayout(const uint64_t recipe_hash, const std::vector<DataPackage::_rtde_type_variant>& values)
{
  uint64_t hash = recipe_hash;
  for (const auto& value : values)
  {
    hash = fnv1aByte(hash, static_cast<uint8_t>(value.index()));
  }
  return hash;
}

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

void copyValues(std::vector<DataPackage::_rtde_type_variant>& destination,
                const std::vector<DataPackage::_rtde_type_variant>& source)
{
  if (destination.empty())
  {
    return;
  }
  std::memcpy(destination.data(), source.data(), destination.size() * sizeof(DataPackage::_rtde_type_variant));
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

void rtde_interface::DataPackage::rebuildFieldIndex()
{
  field_index_.clear();
  field_index_.reserve(recipe_.size());
  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    field_index_.emplace(recipe_[i], i);
  }
}

std::optional<size_t> rtde_interface::DataPackage::fieldIndex(const std::string_view name) const
{
  const auto it = field_index_.find(name);
  if (it == field_index_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<rtde_interface::DataType> rtde_interface::DataPackage::getDataType(const std::string_view name) const
{
  const std::optional<size_t> index = fieldIndex(name);
  if (!index.has_value())
  {
    return std::nullopt;
  }
  return typeOf(values_[*index]);
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
  values_.assign(recipe_.size(), std::monostate());
  zeros_.assign(recipe_.size(), std::monostate());
  rebuildFieldIndex();
  recipe_hash_ = hashRecipe(recipe_);
  updateLayoutHash();
}

void rtde_interface::DataPackage::updateLayoutHash()
{
  layout_hash_ = hashLayout(recipe_hash_, values_);
  fully_typed_ = std::none_of(values_.begin(), values_.end(), [](const _rtde_type_variant& field) {
    return std::holds_alternative<std::monostate>(field);
  });
}

void rtde_interface::DataPackage::setTypes(const std::vector<std::string>& types)
{
  if (types.size() != recipe_.size())
  {
    std::stringstream ss;
    ss << "Cannot set the data types of an RTDE data package: got " << types.size() << " data types for a recipe with "
       << recipe_.size() << " fields.";
    throw UrException(ss.str());
  }

  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    values_[i] = variantFromTypeName(types[i]);
    zeros_[i] = values_[i];
  }
  updateLayoutHash();
}

void rtde_interface::DataPackage::initEmpty()
{
  copyValues(values_, zeros_);
}

rtde_interface::DataPackage rtde_interface::DataPackage::emptyCopy() const
{
  // The delegated constructor allocates the storage, builds the name-to-index map and computes the
  // recipe hash; the field types and their zero values are what this package contributes.
  DataPackage package(recipe_, protocol_version_);
  package.values_ = zeros_;
  package.zeros_ = zeros_;
  package.updateLayoutHash();
  return package;
}

void rtde_interface::DataPackage::reportSlowCopyOnce()
{
  if (slow_copy_reported_)
  {
    return;
  }
  slow_copy_reported_ = true;
  URCL_LOG_WARN("Copying an RTDE data package that is not fully typed walks each field instead of "
                "copying the value array in one step. That is the path a package takes when it is "
                "constructed from a recipe and only some of its fields are written. A package that "
                "already carries the same field names and types as this one can be copied in one "
                "memcpy.");
}

bool rtde_interface::DataPackage::copyFrom(const DataPackage& other)
{
  if (!isTyped())
  {
    URCL_LOG_ERROR("Cannot copy into an RTDE data package before the data types of its recipe are known. Those are "
                   "reported by the robot during the RTDE handshake.");
    return false;
  }

  // Same field names and the same type on every field, so the whole value array can go across at
  // once. This is the path a real-time loop takes.
  if (layout_hash_ == other.layout_hash_ && values_.size() == other.values_.size())
  {
    copyValues(values_, other.values_);
    return true;
  }

  if (recipe_hash_ != other.recipe_hash_ || values_.size() != other.values_.size())
  {
    URCL_LOG_ERROR("Cannot copy from an RTDE data package built from a different recipe.");
    return false;
  }

  // Same recipe, so field i here is field i there. Validate before writing anything, so a package
  // that is rejected leaves the values already in here alone.
  for (size_t i = 0; i < values_.size(); ++i)
  {
    if (!std::holds_alternative<std::monostate>(other.values_[i]) && other.values_[i].index() != values_[i].index())
    {
      URCL_LOG_ERROR("The value passed for the data field '%s' is of type %s, but the robot reports that field as "
                     "%s.",
                     recipe_[i].c_str(), typeNameOf(other.values_[i]).c_str(), typeNameOf(values_[i]).c_str());
      return false;
    }
  }

  for (size_t i = 0; i < values_.size(); ++i)
  {
    values_[i] = std::holds_alternative<std::monostate>(other.values_[i]) ? zeros_[i] : other.values_[i];
  }

  reportSlowCopyOnce();
  return true;
}

bool rtde_interface::DataPackage::parseWith(comm::BinParser& bp)
{
  if (!isTyped())
  {
    URCL_LOG_ERROR("Cannot parse into an RTDE data package before the data types of its recipe are known. Those are "
                   "reported by the robot during the RTDE handshake.");
    return false;
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
        values_[i]);
  }
  return true;
}

std::string rtde_interface::DataPackage::toString() const
{
  std::stringstream ss;
  for (size_t i = 0; i < recipe_.size(); ++i)
  {
    ss << recipe_[i] << ": ";
    if (std::holds_alternative<uint8_t>(values_[i]))
    {
      ss << int(std::get<uint8_t>(values_[i]));
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
          values_[i]);
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

  uint16_t payload_size = 0;
  if (protocol_version_ == 2)
  {
    payload_size += sizeof(recipe_id_);
  }

  for (const auto& value : values_)
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
        value);
  }
  size_t size = 0;
  size += PackageHeader::serializeHeader(buffer, PackageType::RTDE_DATA_PACKAGE, payload_size);
  if (protocol_version_ == 2)
  {
    size += comm::PackageSerializer::serialize(buffer + size, recipe_id_);
  }
  for (size_t i = 0; i < values_.size(); ++i)
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
        values_[i]);
  }

  return size;
}

bool rtde_interface::DataPackage::resetData(const std::string_view name)
{
  const std::optional<size_t> index = fieldIndex(name);
  if (!index.has_value())
  {
    return false;
  }
  values_[*index] = zeros_[*index];
  return true;
}

}  // namespace rtde_interface
}  // namespace urcl
