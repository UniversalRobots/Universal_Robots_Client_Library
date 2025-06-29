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
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_UR_VERSION_INFORMATION_H_INCLUDED
#define UR_CLIENT_LIBRARY_UR_VERSION_INFORMATION_H_INCLUDED

#include <string>
#include <vector>

#include <ur_client_library/types.h>

namespace urcl
{
/*!
 * \brief Struct containing a robot's version information
 */
class VersionInformation
{
public:
  VersionInformation();
  ~VersionInformation() = default;

  /*!
   * \brief Parses a version string into a VersionInformation object
   *
   * \param str Version string such as "5.12.0.1101319"
   *
   * \returns A parsed VersionInformation object
   */
  static VersionInformation fromString(const std::string& str);

  /*!
   * \brief Generates a string representation of the version information such as "5.12.0.1101319"
   */
  std::string toString() const;

  bool isESeries() const;

  friend bool operator==(const VersionInformation& v1, const VersionInformation& v2);
  friend bool operator!=(const VersionInformation& v1, const VersionInformation& v2);
  friend bool operator<(const VersionInformation& v1, const VersionInformation& v2);
  friend bool operator<=(const VersionInformation& v1, const VersionInformation& v2);
  friend bool operator>(const VersionInformation& v1, const VersionInformation& v2);
  friend bool operator>=(const VersionInformation& v1, const VersionInformation& v2);

  friend std::ostream& operator<<(std::ostream& os, const VersionInformation& version_info)
  {
    os << version_info.major << "." << version_info.minor << "." << version_info.bugfix << "-" << version_info.build;
    return os;
  }
  uint32_t major;   ///< Major version number
  uint32_t minor;   ///< Minor version number
  uint32_t bugfix;  ///< Bugfix version number
  uint32_t build;   ///< Build number
};

std::vector<std::string> splitString(std::string input, const std::string& delimiter = ".");
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_UR_VERSION_INFORMATION_H_INCLUDED