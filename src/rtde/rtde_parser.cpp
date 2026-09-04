/*
 * Copyright 2025, Universal Robots A/S (refactor)
 *
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
#include "ur_client_library/rtde/rtde_parser.h"
#include "ur_client_library/rtde/package_header.h"
#include "ur_client_library/rtde/rtde_package.h"

namespace urcl
{
namespace rtde_interface
{
// Only reached when the caller didn't hand in a package we can use. Copying the template gives the
// negotiated recipe and data types without having to reapply them.
std::unique_ptr<DataPackage> RTDEParser::makeTypedDataPackage() const
{
  return std::make_unique<DataPackage>(*typed_template_);
}

bool RTDEParser::parseDataPackagePayload(comm::BinParser& bp, DataPackage& package) const
{
  if (protocol_version_ == 2)
  {
    uint8_t recipe_id = 0;
    bp.parse(recipe_id);
    package.setRecipeID(recipe_id);
  }
  return package.parseWith(bp);
}

bool RTDEParser::recipeTypesKnown() const
{
  if (typed_template_.has_value())
  {
    return true;
  }
  URCL_LOG_ERROR("Received an RTDE data package while the data types of the output recipe are unknown. Those are "
                 "reported by the robot when it acknowledges the recipe, so this means a data package arrived before "
                 "the RTDE handshake was completed.");
  return false;
}

bool RTDEParser::parse(comm::BinParser& bp, std::vector<std::unique_ptr<RTDEPackage>>& results)
{
  static bool warning_printed = false;
  if (!warning_printed)
  {
    URCL_LOG_WARN("Calling RTDEParser::parse(...) with a vector of products is deprecated. It will allocate memory in "
                  "each cycle in order to create parsed packages. Please use the overloaded function that accepts a "
                  "pre-allocated unique pointer to a package. This message is for application developers using this "
                  "function.");
    warning_printed = true;
  }
  PackageType type;
  try
  {
    type = getPackageTypeFromHeader(bp);
  }
  catch (const UrException& e)
  {
    URCL_LOG_ERROR("Exception during RTDE package parsing: %s", e.what());
    return false;
  }

  switch (type)
  {
    case PackageType::RTDE_DATA_PACKAGE:
    {
      if (!recipeTypesKnown())
      {
        return false;
      }
      std::unique_ptr<DataPackage> package = makeTypedDataPackage();

      if (!parseDataPackagePayload(bp, *package))
      {
        URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
        return false;
      }
      results.push_back(std::move(package));
      break;
    }
    default:
    {
      std::unique_ptr<RTDEPackage> package(createNewPackageFromType(type));
      if (!package->parseWith(bp))
      {
        URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
        return false;
      }

      results.push_back(std::move(package));
      break;
    }
  }
  if (!bp.empty())
  {
    URCL_LOG_ERROR("Package of type %d was not parsed completely!", static_cast<int>(type));
    bp.debug();
    return false;
  }

  return true;
}

bool RTDEParser::parse(comm::BinParser& bp, std::unique_ptr<RTDEPackage>& result)
{
  PackageType type;
  try
  {
    type = getPackageTypeFromHeader(bp);
  }
  catch (const UrException& e)
  {
    URCL_LOG_ERROR("Exception during RTDE package parsing: %s", e.what());
    return false;
  }

  switch (type)
  {
    case PackageType::RTDE_DATA_PACKAGE:
    {
      if (!recipeTypesKnown())
      {
        return false;
      }
      if (result == nullptr || result->getType() != PackageType::RTDE_DATA_PACKAGE)
      {
        if (result == nullptr)
        {
          URCL_LOG_WARN("The passed result pointer is empty. A new DataPackage will "
                        "have to be allocated. Please pass a pre-allocated DataPackage if you expect a DataPackage "
                        "would be sent.");
        }
        else
        {
          URCL_LOG_WARN("Passed a pre-allocated RTDE package of type %u while a DataPackage was received. A new "
                        "DataPackage will have to be allocated. Please pass a pre-allocated DataPackage if you expect "
                        "a DataPackage would be sent.",
                        result->getType());
        }
        result = makeTypedDataPackage();
      }

      DataPackage* data_package = dynamic_cast<DataPackage*>(result.get());
      if (data_package->layoutHash() != typed_template_->layoutHash())
      {
        if (data_package->recipeHash() == typed_template_->recipeHash())
        {
          // Built from our recipe, so its storage is already the right shape and only the data
          // types are missing or stale. Applying them writes into that storage without allocating,
          // which is what lets an application hand in a package it built from the recipe alone.
          data_package->setTypes(recipe_types_);
        }
        else
        {
          URCL_LOG_WARN("The passed pre-allocated DataPackage was built from a different recipe. A new DataPackage "
                        "will have to be allocated.");
          result = makeTypedDataPackage();
          data_package = dynamic_cast<DataPackage*>(result.get());
        }
      }

      if (!parseDataPackagePayload(bp, *data_package))
      {
        URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
        return false;
      }
      break;
    }
    default:
    {
      if (result == nullptr || result->getType() != type)
      {
        // For all non data-packages real-time communication isn't critical. Hence, we silently
        // allocate a new package if preallocation isn't given.
        result = std::unique_ptr<RTDEPackage>(createNewPackageFromType(type));
      }
      if (!result->parseWith(bp))
      {
        URCL_LOG_ERROR("Package parsing of type %d failed!", static_cast<int>(type));
        return false;
      }

      break;
    }
  }
  if (!bp.empty())
  {
    URCL_LOG_ERROR("Package of type %d was not parsed completely!", static_cast<int>(type));
    bp.debug();
    return false;
  }

  return true;
}

PackageType RTDEParser::getPackageTypeFromHeader(comm::BinParser& bp) const
{
  PackageHeader::_package_size_type size;
  PackageType type;
  bp.parse(size);
  bp.parse(type);

  if (!bp.checkSize(size - sizeof(size) - sizeof(type)))
  {
    throw UrException("Buffer len shorter than expected packet length");
  }
  return type;
}

RTDEPackage* RTDEParser::createNewPackageFromType(PackageType type) const
{
  switch (type)
  {
    case PackageType::RTDE_TEXT_MESSAGE:
      return new TextMessage(protocol_version_);
      break;
    case PackageType::RTDE_GET_URCONTROL_VERSION:
      return new GetUrcontrolVersion;
      break;
    case PackageType::RTDE_REQUEST_PROTOCOL_VERSION:
      return new RequestProtocolVersion;
      break;
    case PackageType::RTDE_CONTROL_PACKAGE_PAUSE:
      return new ControlPackagePause;
      break;
    case PackageType::RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
      return new ControlPackageSetupInputs;
      break;
    case PackageType::RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
      return new ControlPackageSetupOutputs(protocol_version_);
      break;
    case PackageType::RTDE_CONTROL_PACKAGE_START:
      return new ControlPackageStart;
      break;
    default:
      return new RTDEPackage(type);
  }
}

}  // namespace rtde_interface
}  // namespace urcl
