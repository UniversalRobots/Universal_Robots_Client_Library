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

#ifndef UR_CLIENT_LIBRARY_RTDE_READ_PROPERTIES_H_INCLUDED
#define UR_CLIENT_LIBRARY_RTDE_READ_PROPERTIES_H_INCLUDED

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "ur_client_library/rtde/data_type.h"
#include "ur_client_library/rtde/rtde_package.h"
#include "ur_client_library/ur/datatypes.h"
#include "ur_client_library/ur/version_information.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief Tool-flange type byte and revision byte from v1.robot_arm.tool_flange.type.
 *
 * These are the bytes from the RTDE property, not ToolFlangeType from the primary interface.
 */
struct ToolFlangeProperty
{
  uint8_t type = 0;
  uint8_t revision = 0;

  /*!
   * \brief Text covering both fields, such as "type 2, revision 4".
   */
  std::string toString() const
  {
    return "type " + std::to_string(type) + ", revision " + std::to_string(revision);
  }
};

/*!
 * \brief Control-box type and subtype byte from v1.control_box.type.
 */
struct ControlBoxProperty
{
  ControlBoxType type = ControlBoxType::UNKNOWN;
  uint8_t subtype = 0;

  /*!
   * \brief Text covering both fields, such as "type CB5.2".
   */
  std::string toString() const
  {
    return "type " + controlBoxTypeString(type) + "." + std::to_string(subtype);
  }
};

/*!
 * \brief A catalog row. A property with neither minimum is available on every controller that
 * answers RTDE_READ_PROPERTIES.
 *
 * Asking for a name the controller does not know makes it answer NOT_FOUND and drop the values of
 * every other name in the same request, so the client must only ask for names the controller's
 * version has. A property added later therefore records the first public version that has it, per
 * product line: 5.x and 10.x are released in parallel and are not one ordered sequence.
 */
struct PropertySpec
{
  std::string name;
  std::optional<VersionInformation> polyscope_5_minimum;
  std::optional<VersionInformation> polyscope_x_minimum;
};

/*!
 * \brief The properties this library knows how to ask for.
 *
 * Extend this when the controller gains a property; the client then asks for it on controllers
 * whose version is at least the row's minimum.
 */
const std::vector<PropertySpec>& builtinPropertyCatalog();

/*!
 * \brief The catalog names a controller of \p version can be asked for.
 *
 * Names with no minimum are always included. Other names are included only when \p version is at
 * least that name's minimum on its own product line (major below 10 uses the 5.x minimum).
 *
 * The result views the strings in \p catalog, so it is only valid for as long as \p catalog is.
 */
std::vector<std::string_view> propertyNamesForSoftwareVersion(const VersionInformation& version,
                                                              const std::vector<PropertySpec>& catalog);

/*!
 * \brief The properties of an RTDE_READ_PROPERTIES exchange: the names asked for and the controller's answer.
 *
 * Works like a DataPackage whose recipe is the property names. The package serializes its own
 * request, parses the answer, and looks values up by name. All storage is reserved at
 * construction, so reading the same names again does not allocate.
 *
 * When the controller reports any property as NOT_FOUND or NOT_SET, it sends no values at all. The
 * types it did report are still available through getReportedType().
 */
class ReadProperties : public RTDEPackage
{
public:
  /// Capacity reserved by the default constructor, which the parser uses for answers it receives.
  static constexpr size_t PREALLOCATED_PROPERTIES = 32;
  static constexpr size_t PREALLOCATED_TYPES_LENGTH = 512;

  /*!
   * \brief Creates a package without names, able to receive an answer of up to PREALLOCATED_PROPERTIES entries.
   */
  ReadProperties();

  /*!
   * \brief Creates a package asking for \p names, with room for their answer.
   */
  explicit ReadProperties(const std::vector<std::string>& names);
  ~ReadProperties() override = default;

  /*!
   * \brief Parses the answer: a uint16 length, the comma-separated types, then the values unless
   * an entry is not a data type.
   *
   * An entry that is not a data type, such as NOT_FOUND or NOT_SET, is stored as an empty
   * optional. Values cannot be read past such an entry, so the answer must then end there.
   */
  bool parseWith(comm::BinParser& bp) override;
  std::string toString() const override;

  /*!
   * \brief Serializes the request for names(), comma-separated.
   *
   * \returns The package size, or 0 when there are no names, a name is blank, or the request does
   * not fit in \p buffer_size bytes
   */
  size_t serializeRequest(uint8_t* buffer, size_t buffer_size) const;

  /*!
   * \brief Replaces the names asked for and drops any answer. Passing the current names keeps the storage.
   */
  void setNames(const std::vector<std::string_view>& names);

  /*!
   * \brief Takes the answer parsed into \p answer, keeping this package's names and storage.
   *
   * The answer on the wire does not repeat the names, so the parser produces a package without
   * them. This pairs that answer with the request that asked for it.
   *
   * \returns False if \p answer does not have one entry per name
   */
  bool takeAnswer(const ReadProperties& answer);

  /*!
   * \brief Drops the answer and keeps the names.
   */
  void clearAnswer();

  /*!
   * \brief Copies names and answer from \p other, reusing this package's storage.
   */
  void copyFrom(const ReadProperties& other);

  const std::vector<std::string>& names() const
  {
    return names_;
  }

  /*!
   * \brief Number of entries in the answer.
   */
  size_t size() const
  {
    return data_types_.size();
  }

  /*!
   * \brief The data type reported for answer entry \p index, or an empty optional if the entry is
   * not a data type, such as NOT_FOUND or NOT_SET.
   */
  std::optional<DataType> dataType(const size_t index) const
  {
    return data_types_[index];
  }

  /*!
   * \brief Whether the answer carried values, i.e. every entry is a data type.
   */
  bool hasValues() const
  {
    return has_values_;
  }

  const std::vector<DataValue>& values() const
  {
    return values_;
  }

  /*!
   * \brief The data type the controller reported for \p name, whether or not values were sent.
   *
   * \returns An empty optional if \p name was not asked for, no answer has been taken, or the
   * controller reported something other than a data type, such as NOT_FOUND or NOT_SET
   */
  std::optional<DataType> getReportedType(std::string_view name) const;

  /*!
   * \brief The data type of \p name, or an empty optional if it has no value.
   */
  std::optional<DataType> getDataType(std::string_view name) const;

  /*!
   * \brief Gets the value of property \p name.
   *
   * \returns True on success, false if \p name was not asked for or the answer carried no values
   *
   * \throws std::bad_variant_access if the value does not hold T
   */
  template <typename T>
  bool getData(const std::string_view name, T& val) const
  {
    const std::optional<size_t> index = valueIndex(name);
    if (!index.has_value())
    {
      return false;
    }
    val = std::get<T>(values_[*index]);
    return true;
  }

  /*!
   * \brief Decodes v1.software.version. Components are 2 bytes each, major then minor then patch
   * from MSB to LSB. The last 2 bytes are reserved.
   *
   * \returns An empty optional if v1.software.version was not asked for or has no value
   */
  std::optional<VersionInformation> getSoftwareVersion() const;

  /*!
   * \brief Decodes v1.control_box.type. Byte 0 is the type, byte 1 the subtype.
   *
   * \returns An empty optional if v1.control_box.type was not asked for or has no value
   */
  std::optional<ControlBoxProperty> getControlBoxType() const;

  /*!
   * \brief Decodes v1.robot_arm.tool_flange.type. Byte 0 is the type, byte 1 the revision.
   *
   * \returns An empty optional if v1.robot_arm.tool_flange.type was not asked for or has no value
   */
  std::optional<ToolFlangeProperty> getToolFlangeType() const;

private:
  void reserve(size_t properties, size_t types_length);
  std::optional<size_t> nameIndex(std::string_view name) const;
  std::optional<size_t> valueIndex(std::string_view name) const;

  // The request. Entry i of the answer below belongs to names_[i].
  std::vector<std::string> names_;
  // The answer: one type per entry, empty where the entry is not a data type, and one value per
  // entry when every entry is a data type.
  std::vector<std::optional<DataType>> data_types_;
  std::vector<DataValue> values_;
  bool has_values_ = false;
  // Scratch space for parseWith(): the received type names and their split. Reserved with the
  // rest so parsing does not allocate, and not part of the answer.
  std::string type_names_;
  std::vector<std::string_view> type_parts_;
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_RTDE_READ_PROPERTIES_H_INCLUDED
