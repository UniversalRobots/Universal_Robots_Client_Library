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
//    * Neither the name of the {copyright_holder} nor the names of its
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

#pragma once

// The parser and the writer keep setRecipeTypes() out of their public interface. Tests, and the
// fake server standing in for the robot, reach it through these subclasses.

#include <string>
#include <vector>

#include <ur_client_library/rtde/data_package.h>
#include <ur_client_library/rtde/rtde_parser.h>
#include <ur_client_library/rtde/rtde_writer.h>

namespace urcl
{
namespace test
{
class TestableDataPackage : public rtde_interface::DataPackage
{
public:
  using rtde_interface::DataPackage::DataPackage;
};

class TestableRTDEParser : public rtde_interface::RTDEParser
{
public:
  explicit TestableRTDEParser(const std::vector<std::string>& recipe) : rtde_interface::RTDEParser(recipe)
  {
  }

  using rtde_interface::RTDEParser::setRecipeTypes;
};

class TestableRTDEWriter : public rtde_interface::RTDEWriter
{
public:
  TestableRTDEWriter(comm::URStream<rtde_interface::RTDEPackage>* stream, const std::vector<std::string>& recipe)
    : rtde_interface::RTDEWriter(stream, recipe)
  {
  }

  using rtde_interface::RTDEWriter::setRecipeTypes;
};

/*!
 * \brief Builds a data package the way the library does: allocate from the recipe, then apply the
 * data types the robot reported for it.
 */
inline TestableDataPackage typedPackage(const std::vector<std::string>& recipe, const std::vector<std::string>& types)
{
  TestableDataPackage package(recipe);
  package.setTypes(types);
  return package;
}
}  // namespace test
}  // namespace urcl
