/*
 * Copyright 2019, FZI Forschungszentrum Informatik (refactor)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <vector>
#include "ur_client_library/comm/parser.h"
#include "ur_client_library/comm/bin_parser.h"
#include "ur_client_library/comm/pipeline.h"

#include "ur_client_library/rtde/control_package_pause.h"
#include "ur_client_library/rtde/control_package_setup_inputs.h"
#include "ur_client_library/rtde/control_package_setup_outputs.h"
#include "ur_client_library/rtde/control_package_start.h"
#include "ur_client_library/rtde/data_package.h"
#include "ur_client_library/rtde/get_urcontrol_version.h"
#include "ur_client_library/rtde/package_header.h"
#include "ur_client_library/rtde/request_protocol_version.h"
#include "ur_client_library/rtde/text_message.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief The RTDE specific parser. Interprets a given byte stream as serialized RTDE packages
 * and parses it accordingly.
 */
class RTDEParser : public comm::Parser<RTDEPackage>
{
public:
  RTDEParser() = delete;
  /*!
   * \brief Creates a new RTDEParser object, registering the used recipe.
   *
   * \param recipe The recipe used in RTDE data communication
   */
  RTDEParser(const std::vector<std::string>& recipe) : recipe_(recipe), protocol_version_(1)
  {
  }
  virtual ~RTDEParser() = default;

  /*!
   * \brief Uses the given BinParser to fill single package object from the contained serialization.
   *
   *
   * \param bp A BinParser holding a serialized RTDE package
   * \param result A pointer to the created RTDE package object. Ideally, the passed \p result is a pre-allocated
   * package of the type expected to be read. For example, when RTDE communication has been setup it enters the data
   * communication phase, where the expected package is a DataPackage. If the package content inside the \p bp object
   * being doesn't match the result package's type or if the \p result is a nullptr, a new package will be allocated.
   *
   * \returns True, if the byte stream could successfully be parsed as an RTDE package, false
   * otherwise
   */
  bool parse(comm::BinParser& bp, std::unique_ptr<RTDEPackage>& result) override;

  /*!
   * \brief Uses the given BinParser to create package objects from the contained serialization.
   *
   * \param bp A BinParser holding one or more serialized RTDE packages
   * \param results A vector of pointers to created RTDE package objects
   *
   * \returns True, if the byte stream could successfully be parsed as RTDE packages, false
   * otherwise
   */
  bool parse(comm::BinParser& bp, std::vector<std::unique_ptr<RTDEPackage>>& results) override;

  void setProtocolVersion(uint16_t protocol_version)
  {
    protocol_version_ = protocol_version;
  }

  uint16_t getProtocolVersion() const
  {
    return protocol_version_;
  }

private:
  std::vector<std::string> recipe_;
  PackageType getPackageTypeFromHeader(comm::BinParser& bp) const;
  RTDEPackage* createNewPackageFromType(PackageType type) const;

  uint16_t protocol_version_;
};

}  // namespace rtde_interface
}  // namespace urcl
