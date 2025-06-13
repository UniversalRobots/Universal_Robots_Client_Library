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
#include <ur_client_library/ur/version_information.h>

namespace urcl
{
std::vector<std::string> splitString(std::string input, const std::string& delimiter)
{
  std::vector<std::string> result;
  size_t pos = 0;
  std::string substring;
  while ((pos = input.find(delimiter)) != std::string::npos)
  {
    substring = input.substr(0, pos);
    result.push_back(substring);
    input.erase(0, pos + delimiter.length());
  }
  result.push_back(input);
  return result;
}

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
  auto components = splitString(str);
  VersionInformation info;
  if (components.size() >= 2)
  {
    info.major = std::stoi(components[0]);
    info.minor = std::stoi(components[1]);
    if (components.size() >= 3)
    {
      info.bugfix = std::stoi(components[2]);
      if (components.size() == 4)
      {
        info.build = std::stoi(components[3]);
      }
      else if (components.size() > 4)
      {
        throw UrException("Given string '" + str + "' does not conform a version string format.");
      }
    }
  }
  else
  {
    throw UrException("Given string '" + str + "' does not conform a version string format.");
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