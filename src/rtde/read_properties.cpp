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
 * \date    2026-09-24
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/rtde/read_properties.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

#include "ur_client_library/log.h"

namespace urcl
{
namespace rtde_interface
{
namespace
{
constexpr std::string_view SOFTWARE_VERSION = "v1.software.version";
constexpr std::string_view CONTROL_BOX_TYPE = "v1.control_box.type";
constexpr std::string_view TOOL_FLANGE_TYPE = "v1.robot_arm.tool_flange.type";
// Longest type name, VECTOR6UINT32, plus its comma.
constexpr size_t TYPE_NAME_CAPACITY = 14;

// The request is a plain comma-separated list with no escaping.
bool isValidName(const std::string_view name)
{
  return name.find_first_not_of(" \t\r\n") != std::string_view::npos && name.find(',') == std::string_view::npos;
}
}  // namespace

const std::vector<PropertySpec>& builtinPropertyCatalog()
{
  // The first properties the controller shipped, so every controller that answers
  // RTDE_READ_PROPERTIES has them. Later properties add a row with a minimum version per product
  // line.
  static const std::vector<PropertySpec> catalog = {
    { std::string(SOFTWARE_VERSION), std::nullopt, std::nullopt },
    { std::string(CONTROL_BOX_TYPE), std::nullopt, std::nullopt },
    { std::string(TOOL_FLANGE_TYPE), std::nullopt, std::nullopt },
  };
  return catalog;
}

std::vector<std::string_view> propertyNamesForSoftwareVersion(const VersionInformation& version,
                                                              const std::vector<PropertySpec>& catalog)
{
  std::vector<std::string_view> names;
  const bool polyscope_5 = version.major < 10;
  for (const auto& spec : catalog)
  {
    if (!spec.polyscope_5_minimum.has_value() && !spec.polyscope_x_minimum.has_value())
    {
      names.push_back(spec.name);
      continue;
    }
    const std::optional<VersionInformation>& minimum =
        polyscope_5 ? spec.polyscope_5_minimum : spec.polyscope_x_minimum;
    if (minimum.has_value() && version >= *minimum)
    {
      names.push_back(spec.name);
    }
  }
  return names;
}

ReadProperties::ReadProperties() : RTDEPackage(PackageType::RTDE_READ_PROPERTIES)
{
  reserve(PREALLOCATED_PROPERTIES, PREALLOCATED_TYPES_LENGTH);
}

ReadProperties::ReadProperties(const std::vector<std::string>& names)
  : RTDEPackage(PackageType::RTDE_READ_PROPERTIES), names_(names)
{
  reserve(names_.size(), names_.size() * TYPE_NAME_CAPACITY);
}

ReadProperties::ReadProperties(const ReadProperties& other)
  : RTDEPackage(PackageType::RTDE_READ_PROPERTIES)
  , names_(other.names_)
  , data_types_(other.data_types_)
  , values_(other.values_)
  , has_values_(other.has_values_)
{
}

ReadProperties::ReadProperties(ReadProperties&& other) noexcept
  : RTDEPackage(PackageType::RTDE_READ_PROPERTIES)
  , names_(std::move(other.names_))
  , data_types_(std::move(other.data_types_))
  , values_(std::move(other.values_))
  , has_values_(other.has_values_)
{
  other.names_.clear();
  other.clearAnswer();
}

ReadProperties& ReadProperties::operator=(const ReadProperties& other)
{
  if (this != &other)
  {
    names_ = other.names_;
    data_types_ = other.data_types_;
    values_ = other.values_;
    has_values_ = other.has_values_;
  }
  return *this;
}

ReadProperties& ReadProperties::operator=(ReadProperties&& other) noexcept
{
  if (this != &other)
  {
    names_ = std::move(other.names_);
    data_types_ = std::move(other.data_types_);
    values_ = std::move(other.values_);
    has_values_ = other.has_values_;
    other.names_.clear();
    other.clearAnswer();
  }
  return *this;
}

void ReadProperties::reserve(const size_t properties, const size_t types_length)
{
  data_types_.reserve(properties);
  values_.reserve(properties);
  type_names_.reserve(types_length);
  type_parts_.reserve(properties);
}

void ReadProperties::clearAnswer()
{
  data_types_.clear();
  values_.clear();
  has_values_ = false;
}

bool ReadProperties::parseWith(comm::BinParser& bp)
{
  uint16_t types_length = 0;
  bp.parse(types_length);
  bp.parse(type_names_, types_length);
  values_.clear();
  has_values_ = false;

  const bool all_data_types = parseDataTypes(type_names_, type_parts_, data_types_);

  // Every entry has to be a data type before any value can be read: a single NOT_FOUND or NOT_SET
  // means the controller sent no values at all.
  if (!all_data_types)
  {
    if (!bp.empty())
    {
      URCL_LOG_ERROR("RTDE_READ_PROPERTIES included values after an entry that is not a data type");
      return false;
    }
    return true;
  }

  // The values are encoded like the fields of a data package, back to back in the reported types.
  for (const std::optional<DataType>& type : data_types_)
  {
    values_.emplace_back();
    parseValue(bp, *type, values_.back());
  }
  has_values_ = true;
  return true;
}

std::string ReadProperties::toString() const
{
  std::stringstream ss;
  ss << "property data types:";
  for (const std::optional<DataType>& type : data_types_)
  {
    ss << " " << (type.has_value() ? rtde_interface::toString(*type) : "<not a data type>");
  }
  ss << std::endl;
  ss << "property values present: " << (has_values_ ? "true" : "false") << std::endl;
  return ss.str();
}

size_t ReadProperties::serializeRequest(uint8_t* buffer, const size_t buffer_size) const
{
  if (names_.empty())
  {
    return 0;
  }
  size_t payload_size = names_.size() - 1;
  for (const auto& name : names_)
  {
    if (!isValidName(name))
    {
      return 0;
    }
    payload_size += name.size();
  }
  const size_t header_size = sizeof(PackageHeader::_package_size_type) + sizeof(PackageType);
  if (payload_size > std::numeric_limits<uint16_t>::max() - header_size || header_size + payload_size > buffer_size)
  {
    return 0;
  }

  size_t size =
      PackageHeader::serializeHeader(buffer, PackageType::RTDE_READ_PROPERTIES, static_cast<uint16_t>(payload_size));
  for (size_t i = 0; i < names_.size(); ++i)
  {
    if (i > 0)
    {
      buffer[size++] = ',';
    }
    std::memcpy(buffer + size, names_[i].data(), names_[i].size());
    size += names_[i].size();
  }
  return size;
}

void ReadProperties::setNames(const std::vector<std::string_view>& names)
{
  clearAnswer();
  // A reconnect usually asks for the same names again; keeping them avoids reassigning strings.
  if (std::equal(names_.begin(), names_.end(), names.begin(), names.end()))
  {
    return;
  }
  // \p names may view the strings in names_, so copy them all before names_ changes.
  std::vector<std::string> copied(names.begin(), names.end());
  names_.swap(copied);
  reserve(names_.size(), names_.size() * TYPE_NAME_CAPACITY);
}

bool ReadProperties::takeAnswer(ReadProperties&& answer)
{
  if (answer.size() != names_.size() || (answer.hasValues() && answer.values().size() != names_.size()))
  {
    URCL_LOG_ERROR("RTDE_READ_PROPERTIES answered with %zu entries for %zu names", answer.size(), names_.size());
    clearAnswer();
    return false;
  }
  data_types_ = std::move(answer.data_types_);
  values_ = std::move(answer.values_);
  has_values_ = answer.has_values_;
  answer.clearAnswer();
  return true;
}

std::optional<size_t> ReadProperties::nameIndex(const std::string_view name) const
{
  for (size_t i = 0; i < names_.size(); ++i)
  {
    if (names_[i] == name)
    {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<size_t> ReadProperties::valueIndex(const std::string_view name) const
{
  const std::optional<size_t> index = nameIndex(name);
  if (!has_values_ || !index.has_value() || *index >= values_.size())
  {
    return std::nullopt;
  }
  return index;
}

std::optional<DataType> ReadProperties::getReportedType(const std::string_view name) const
{
  const std::optional<size_t> index = nameIndex(name);
  if (!index.has_value() || *index >= data_types_.size())
  {
    return std::nullopt;
  }
  return data_types_[*index];
}

std::optional<DataType> ReadProperties::getDataType(const std::string_view name) const
{
  const std::optional<size_t> index = valueIndex(name);
  if (!index.has_value())
  {
    return std::nullopt;
  }
  return data_types_[*index];
}

std::optional<VersionInformation> ReadProperties::getSoftwareVersion() const
{
  const std::optional<size_t> index = valueIndex(SOFTWARE_VERSION);
  const uint64_t* wire = index.has_value() ? std::get_if<uint64_t>(&values_[*index]) : nullptr;
  if (wire == nullptr)
  {
    return std::nullopt;
  }
  VersionInformation version;
  version.major = static_cast<uint32_t>((*wire >> 48) & 0xFFFF);
  version.minor = static_cast<uint32_t>((*wire >> 32) & 0xFFFF);
  version.bugfix = static_cast<uint32_t>((*wire >> 16) & 0xFFFF);
  version.build = 0;
  return version;
}

std::optional<ControlBoxProperty> ReadProperties::getControlBoxType() const
{
  const std::optional<size_t> index = valueIndex(CONTROL_BOX_TYPE);
  const uint32_t* wire = index.has_value() ? std::get_if<uint32_t>(&values_[*index]) : nullptr;
  if (wire == nullptr)
  {
    return std::nullopt;
  }
  ControlBoxProperty control_box;
  control_box.type = controlBoxTypeFromWire(static_cast<uint16_t>((*wire >> 24) & 0xFF));
  control_box.subtype = static_cast<uint8_t>((*wire >> 16) & 0xFF);
  return control_box;
}

std::optional<ToolFlangeProperty> ReadProperties::getToolFlangeType() const
{
  const std::optional<size_t> index = valueIndex(TOOL_FLANGE_TYPE);
  const uint32_t* wire = index.has_value() ? std::get_if<uint32_t>(&values_[*index]) : nullptr;
  if (wire == nullptr)
  {
    return std::nullopt;
  }
  ToolFlangeProperty tool_flange;
  tool_flange.type = static_cast<uint8_t>((*wire >> 24) & 0xFF);
  tool_flange.revision = static_cast<uint8_t>((*wire >> 16) & 0xFF);
  return tool_flange;
}

}  // namespace rtde_interface
}  // namespace urcl
