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

bool RTDEParser::parse(comm::BinParser& bp, std::vector<std::unique_ptr<RTDEPackage>>& results)
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
      std::unique_ptr<RTDEPackage> package(new DataPackage(recipe_, protocol_version_));

      if (!package->parseWith(bp))
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
  PackageType type = getPackageTypeFromHeader(bp);

  switch (type)
  {
    case PackageType::RTDE_DATA_PACKAGE:
    {
      if (result == nullptr || result->getType() != PackageType::RTDE_DATA_PACKAGE)
      {
        result = std::make_unique<DataPackage>(recipe_, protocol_version_);
        URCL_LOG_WARN("The passed result pointer is empty or does not contain a DataPackage. A new DataPackage will "
                      "have to be allocated. Please pass a pre-allocated DataPackage if you expect a DataPackage "
                      "would be sent.");
      }

      if (!dynamic_cast<DataPackage*>(result.get())->parseWith(bp))
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
