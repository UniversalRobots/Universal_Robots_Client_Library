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
#include <optional>
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
   * The data types belonging to the recipe are only known once the robot has acknowledged it, so
   * setRecipeTypes() has to be called before data packages can be parsed.
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
   * A DataPackage built from the registered recipe is not replaced even when it carries no data types yet: those are
   * applied to it in place, which allocates nothing. Only a DataPackage built from a different recipe is replaced.
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
  [[deprecated("This method allocates memory on each call. Please use the overload which takes a single unique ptr to "
               "a pre-allocated package. This function will be removed in May 2027.")]]
  bool parse(comm::BinParser& bp, std::vector<std::unique_ptr<RTDEPackage>>& results) override;

  void setProtocolVersion(uint16_t protocol_version)
  {
    protocol_version_ = protocol_version;
    if (typed_template_.has_value())
    {
      typed_template_->setProtocolVersion(protocol_version);
    }
  }

  uint16_t getProtocolVersion() const
  {
    return protocol_version_;
  }

  /*!
   * \brief Registers the data types belonging to the recipe, as reported by the robot in the RTDE
   * setup acknowledgement.
   *
   * This has to be called before the robot starts sending data packages, i.e. before the
   * RTDE_CONTROL_PACKAGE_START request is sent.
   *
   * \param types The data types, in the same order as the recipe
   */
  void setRecipeTypes(const std::vector<std::string>& types)
  {
    recipe_types_ = types;
    // A package carrying the negotiated layout. Its hashes are the reference a passed-in package is
    // held against, and it is the blueprint for any package this parser has to allocate itself.
    typed_template_.emplace(recipe_);
    typed_template_->setTypes(recipe_types_);
    typed_template_->setProtocolVersion(protocol_version_);
  }

private:
  std::unique_ptr<DataPackage> makeTypedDataPackage() const;
  bool parseDataPackagePayload(comm::BinParser& bp, DataPackage& package) const;

  std::vector<std::string> recipe_;
  std::vector<std::string> recipe_types_;
  std::optional<DataPackage> typed_template_;
  bool recipeTypesKnown() const;
  PackageType getPackageTypeFromHeader(comm::BinParser& bp) const;
  RTDEPackage* createNewPackageFromType(PackageType type) const;

  uint16_t protocol_version_;
};

}  // namespace rtde_interface
}  // namespace urcl
