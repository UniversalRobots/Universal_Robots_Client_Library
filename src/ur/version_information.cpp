// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
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
 * \author  "Felix Exner" <exner@fzi.de>
 * \date    2022-11-22
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/exceptions.h>
#include <ur_client_library/helpers.h>
#include <ur_client_library/ur/version_information.h>

#include <limits>

namespace urcl
{
namespace
{
uint32_t parseVersionComponent(const std::string& component, const std::string& original)
{
  if (component.empty())
  {
    throw UrException("Given string '" + original + "' does not conform a version string format.");
  }

  uint32_t value = 0;
  for (const char c : component)
  {
    if (c < '0' || c > '9')
    {
      throw UrException("Given string '" + original + "' does not conform a version string format.");
    }
    const uint32_t digit = static_cast<uint32_t>(c - '0');
    if (value > (std::numeric_limits<uint32_t>::max() - digit) / 10)
    {
      throw UrException("Given string '" + original + "' contains a version component outside the supported range.");
    }
    value = value * 10 + digit;
  }
  return value;
}
}  // namespace

VersionInformation::VersionInformation()
{
  // Since 'major' and 'minor' are keywords in  <sys/types> we don't use the initializer list and
  // specify this->major explicitly.
  this->major = 0;
  this->minor = 0;
  this->bugfix = 0;
  this->build = 0;
}

VersionInformation VersionInformation::fromString(const std::string& str)
{
  const auto components = splitString(str, ".");
  if (components.size() < 2 || components.size() > 4)
  {
    throw UrException("Given string '" + str + "' does not conform a version string format.");
  }

  VersionInformation info;
  info.major = parseVersionComponent(components[0], str);
  info.minor = parseVersionComponent(components[1], str);
  if (components.size() >= 3)
  {
    info.bugfix = parseVersionComponent(components[2], str);
  }
  if (components.size() == 4)
  {
    info.build = parseVersionComponent(components[3], str);
  }

  return info;
}
std::string VersionInformation::toString() const
{
  return std::to_string(this->major) + "." + std::to_string(this->minor) + "." + std::to_string(this->bugfix) + "." +
         std::to_string(this->build);
}

bool VersionInformation::isESeries() const
{
  return this->major >= 5;
}

bool operator==(const VersionInformation& v1, const VersionInformation& v2)
{
  return v1.major == v2.major && v1.minor == v2.minor && v1.bugfix == v2.bugfix && v1.build == v2.build;
}

bool operator!=(const VersionInformation& v1, const VersionInformation& v2)
{
  return !(v1 == v2);
}

bool operator<(const VersionInformation& v1, const VersionInformation& v2)
{
  if (v1.major <= v2.major)
  {
    if (v1.major < v2.major)
    {
      return true;
    }
    if (v1.minor <= v2.minor)
    {
      if (v1.minor < v2.minor)
      {
        return true;
      }
      if (v1.bugfix <= v2.bugfix)
      {
        if (v1.bugfix < v2.bugfix)
        {
          return true;
        }
      }
      if (v1.build < v2.build)
      {
        return true;
      }
    }
  }
  return false;
}

bool operator<=(const VersionInformation& v1, const VersionInformation& v2)
{
  return v1 < v2 || v1 == v2;
}

bool operator>(const VersionInformation& v1, const VersionInformation& v2)
{
  return !(v1 <= v2);
}

bool operator>=(const VersionInformation& v1, const VersionInformation& v2)
{
  return !(v1 < v2);
}
}  // namespace urcl
