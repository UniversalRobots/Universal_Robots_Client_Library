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
 * \author  Lea Steffen steffen@fzi.de
 * \date    2019-04-01
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_DATA_PACKAGE_H_INCLUDED
#define UR_CLIENT_LIBRARY_DATA_PACKAGE_H_INCLUDED

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include "ur_client_library/log.h"
#include "ur_client_library/types.h"
#include "ur_client_library/rtde/rtde_package.h"

namespace urcl
{
namespace rtde_interface
{
/*!
 * \brief Possible values for the runtime state
 */
enum class RUNTIME_STATE : uint32_t
{
  STOPPING = 0,
  STOPPED = 1,
  PLAYING = 2,
  PAUSING = 3,
  PAUSED = 4,
  RESUMING = 5
};

/*!
 * \brief The data types an RTDE field can have.
 *
 * This is the complete set the protocol defines. Which one a given field has is decided by the
 * robot when it acknowledges a recipe, so this list is all the type knowledge the library needs to
 * carry; see DataPackage::getDataType().
 */
enum class DataType : uint8_t
{
  BOOL,
  UINT8,
  UINT32,
  UINT64,
  INT32,
  DOUBLE,
  VECTOR3D,
  VECTOR6D,
  VECTOR6INT32,
  VECTOR6UINT32
};

/*!
 * \brief The name the RTDE protocol uses for a data type, e.g. "VECTOR6D".
 *
 * This is the spelling the robot uses on the wire and the RTDE guide uses in its field tables.
 */
std::string toString(const DataType type);

/*!
 * \brief The DataPackage class handles communication in the form of RTDE data packages both to and
 * from the robot. It contains functionality to parse and serialize packages for arbitrary recipes.
 *
 * A recipe only names the fields to exchange; their data types are reported by the robot in the
 * RTDE setup acknowledgement. Constructing a package from a recipe therefore allocates all of its
 * storage but leaves it *untyped*, and the acknowledgement later decides which type each field
 * holds. Since every RTDE data type is trivially copyable with inline storage, that second step
 * costs no memory, which is why a package can be created before a connection exists and still be
 * used in a real-time loop:
 *
 * \code
 * rtde_interface::DataPackage data_pkg(my_client.getOutputRecipe());  // allocates here
 * while (true)
 * {
 *   my_client.getDataPackage(data_pkg, timeout);  // types it once, then never allocates
 * }
 * \endcode
 *
 * Until a package has been typed, either by receiving into it or by writing to it with setData(),
 * it cannot be parsed into or serialized and getData() will fail.
 */
class DataPackage : public RTDEPackage
{
public:
  /*!
   * \brief The type a data field can hold.
   *
   * The typed alternatives are exactly the members of DataType. std::monostate is the state of a
   * field whose type isn't decided yet, which is how a package constructed from a recipe alone
   * starts out.
   */
  using _rtde_type_variant = std::variant<std::monostate, bool, uint8_t, uint32_t, uint64_t, int32_t, double,
                                          vector3d_t, vector6d_t, vector6int32_t, vector6uint32_t>;

  // A data package is created before the connection exists and then retyped in place from the
  // robot's acknowledgement, so no alternative may own heap memory: retyping has to stay a
  // write into the variant's inline storage.
  static_assert(std::is_trivially_copyable_v<_rtde_type_variant>, "An RTDE data field must not own heap memory.");

  DataPackage() = delete;

  DataPackage(const DataPackage& other)
    : RTDEPackage(PackageType::RTDE_DATA_PACKAGE)
    , recipe_id_(other.recipe_id_)
    , recipe_(other.recipe_)
    , values_(other.values_)
    , zeros_(other.zeros_)
    , protocol_version_(other.protocol_version_)
    , recipe_hash_(other.recipe_hash_)
    , layout_hash_(other.layout_hash_)
    , fully_typed_(other.fully_typed_)
  {
    rebuildFieldIndex();
  }

  /*!
   * \brief Copies recipe, type information and values from another package.
   *
   * The recipe id is deliberately left untouched: an RTDEWriter's send buffers own the id that was
   * negotiated during the input setup, while packages passed in by an application have none.
   */
  DataPackage& operator=(const DataPackage& other)
  {
    // The name-to-index map holds string_views into recipe_. Replacing recipe_ would dangle those
    // views and would allocate, so a same-recipe assignment (the receive path) leaves both alone.
    if (recipe_hash_ != other.recipe_hash_ || recipe_.size() != other.recipe_.size())
    {
      this->recipe_ = other.recipe_;
      this->recipe_hash_ = other.recipe_hash_;
      rebuildFieldIndex();
    }
    this->values_ = other.values_;
    this->zeros_ = other.zeros_;
    this->protocol_version_ = other.protocol_version_;
    this->layout_hash_ = other.layout_hash_;
    this->fully_typed_ = other.fully_typed_;
    return *this;
  }

  /*!
   * \brief Creates a new DataPackage object based on a given recipe, allocating all of its storage.
   *
   * The data types of the recipe's fields are only known once the robot has acknowledged the
   * recipe, so the package starts out *untyped*: it cannot be parsed into or serialized, and
   * getData() fails, until it has been typed. That happens either by receiving into it (see
   * RTDEClient::getDataPackage()) or, for input recipes, by writing to it with setData().
   *
   * Typing a package does not allocate, so this constructor is the only point at which the package
   * touches the heap. Call it wherever suits your application; it needs no connection.
   *
   * \param recipe The used recipe
   * \param protocol_version Protocol version used for the RTDE communication
   */
  DataPackage(const std::vector<std::string>& recipe, const uint16_t& protocol_version = 2)
    : RTDEPackage(PackageType::RTDE_DATA_PACKAGE), recipe_(recipe), protocol_version_(protocol_version)
  {
    initStorage();
  }

  virtual ~DataPackage() = default;

  /*!
   * \brief Resets every data field to a default-constructed value of its own type.
   *
   * The types are left alone, so a typed package stays typed.
   */
  void initEmpty();

  /*!
   * \brief A package with this one's recipe, protocol version and data types, all values zero.
   *
   * Since it carries the same field names and types, it has this package's layout hash and can be
   * copied into it with a single memcpy; see copyFrom().
   *
   * \returns A zeroed package with this package's layout
   */
  DataPackage emptyCopy() const;

  /*!
   * \brief Get the data type the robot reported for a field.
   *
   * Which type a field holds is decided by the robot when it acknowledges the recipe, so this is
   * the way to find out what to pass to getData() without hardcoding it. A package that hasn't
   * been acknowledged yet has no answer to give.
   *
   * \param name The string identifier for the data field as used in the documentation.
   *
   * \returns The field's data type, or an empty optional if the field cannot be found inside the
   * package or if its type isn't known yet.
   */
  std::optional<DataType> getDataType(const std::string_view name) const;

  /*!
   * \brief Sets the attributes of the package by parsing a serialized representation of the
   * package.
   *
   * \param bp A parser containing a serialized version of the package
   *
   * \returns True, if the package was parsed successfully, false otherwise
   */
  virtual bool parseWith(comm::BinParser& bp);
  /*!
   * \brief Produces a human readable representation of the package object.
   *
   * \returns A string representing the object
   */
  virtual std::string toString() const;

  /*!
   * \brief Serializes the package.
   *
   * Version 2 data packages start with a recipe-id byte; version 1 packages do not. The writer
   * records the negotiated version with setProtocolVersion() before serializing.
   *
   * \param buffer Buffer to fill with the serialization
   *
   * \returns The total size of the serialized package
   */
  size_t serializePackage(uint8_t* buffer);

  /*!
   * \brief Get a data field from the DataPackage.
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Target variable. Make sure, it's the correct type.
   *
   * \returns True on success, false if the field cannot be found inside the package or if its type
   * doesn't match the requested one.
   */
  template <typename T>
  bool getData(const std::string_view name, T& val) const
  {
    const std::optional<size_t> index = fieldIndex(name);
    if (!index.has_value())
    {
      return false;
    }
    const T* value = std::get_if<T>(&values_[*index]);
    if (value == nullptr)
    {
      reportReadFailure(recipe_[*index], values_[*index]);
      return false;
    }
    val = *value;
    return true;
  }

  /*!
   * \brief Get a data field from the DataPackage as bitset
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Target variable. Make sure, it's the correct type.
   *
   * \returns True on success, false if the field cannot be found inside the package or if its type
   * doesn't match the requested one.
   */
  template <typename T, size_t N>
  bool getData(const std::string_view name, std::bitset<N>& val) const
  {
    static_assert(sizeof(T) * 8 >= N, "Bitset is too large for underlying variable");
    T recipe_type;
    if (!getData(name, recipe_type))
    {
      return false;
    }
    val = std::bitset<N>(recipe_type);
    return true;
  }

  /*!
   * \brief Set a data field in the DataPackage.
   *
   * The data package contains a lot of different data fields, depending on the recipe.
   *
   * On a field whose type isn't decided yet this establishes the type from \p val. Whether that
   * matches what the robot expects is checked when the package is sent, since only then is the
   * robot's acknowledgement available. On a field that already has a type, \p val has to match it.
   *
   * \param name The string identifier for the data field as used in the documentation.
   * \param val Value to set. Make sure, it's the correct type.
   *
   * \returns True on success, false if the field cannot be found inside the package or if its type
   * doesn't match the passed one.
   */
  template <typename T>
  bool setData(const std::string_view name, const T& val)
  {
    const std::optional<size_t> index = fieldIndex(name);
    if (!index.has_value())
    {
      return false;
    }
    _rtde_type_variant& field = values_[*index];
    if (!std::holds_alternative<T>(field) && !std::holds_alternative<std::monostate>(field))
    {
      // TODO: It might be better to replace the return type by void and use exceptions for the
      // error case.
      URCL_LOG_ERROR("Type of passed data doesn't match type of existing field for index '%.*s'",
                     static_cast<int>(name.size()), name.data());
      return false;
    }
    const bool type_changed = std::holds_alternative<std::monostate>(field);
    field = val;
    if (type_changed)
    {
      zeros_[*index] = T();
      updateLayoutHash();
    }
    return true;
  }

  /*!
   * \brief Setter of the recipe id value used to identify the used recipe to the robot.
   *
   * \param recipe_id The new value
   */
  void setRecipeID(const uint8_t& recipe_id)
  {
    recipe_id_ = recipe_id;
  }

  /*!
   * \brief Records the RTDE protocol version this package will serialize.
   *
   * Version 2 data packages start with a recipe-id byte; version 1 packages do not. The
   * constructor defaults to version 2.
   */
  void setProtocolVersion(const uint16_t protocol_version)
  {
    protocol_version_ = protocol_version;
  }

  /*!
   * \brief Applies the data types reported by the robot, resetting all values to zero.
   *
   * The storage was already allocated by the constructor, so this only decides which type each
   * field holds and therefore performs no memory allocation. That is what allows a package to be
   * created before the recipe has been acknowledged and still be used in a real-time loop.
   *
   * \param types The data types of the recipe's fields, in the same order as the recipe
   *
   * \throws UrException if the number of types doesn't match the recipe or if a type is unknown
   */
  void setTypes(const std::vector<std::string>& types);

  /*!
   * \brief Takes over the values of \p other, sending fields it has not written as zeros.
   *
   * This package must already be typed. \p other has to be built from the same recipe, and every
   * field it has a value for has to carry the type this package has for it. Recipe id and protocol
   * version are left untouched.
   *
   * When \p other has the same field names and the same type on every one of them, which is what a
   * package has after the robot's acknowledgement, the copy is a layout-hash compare and a memcpy
   * of the value array. The hashes are a 64-bit identity of the field names and each field's
   * variant index; a collision would skip a validation that should have failed, which is accepted
   * for this path. A package an application typed by writing only the fields it cares about is
   * instead merged position by position, with unwritten fields sent as zeros.
   *
   * \param other The package to copy from
   *
   * \returns True on success, false if this package is untyped, if \p other was built from a
   * different recipe, or if a field \p other has written has a different type
   */
  bool copyFrom(const DataPackage& other);

  /*!
   * \brief Resets a data field to a default-constructed value of its own type.
   *
   * \param name The string identifier for the data field as used in the documentation.
   *
   * \returns True on success, false if the field cannot be found inside the package.
   */
  bool resetData(const std::string_view name);

  /*!
   * \brief Whether every field of this package has a data type.
   *
   * A package constructed from a recipe alone is untyped until either the robot's setup
   * acknowledgement has been applied to it or setData() has been used to write to every field. An
   * untyped package cannot be parsed into or serialized, and getData() fails on it.
   *
   * \returns True if the package carries type information for all of its fields
   */
  bool isTyped() const
  {
    return fully_typed_;
  }

  /*!
   * \brief FNV-1a identity of this package's field names, in order.
   *
   * Not sent on the wire. Used together with layoutHash() to identify a recipe without comparing
   * field-name strings.
   */
  uint64_t recipeHash() const
  {
    return recipe_hash_;
  }

  /*!
   * \brief FNV-1a identity of this package's field names and each field's current variant index.
   *
   * Not sent on the wire. Combined from the recipe hash and the type of every field, so it changes
   * when a field first acquires a type and when setTypes() is applied, and does not change when a
   * value is overwritten, reset or parsed.
   */
  uint64_t layoutHash() const
  {
    return layout_hash_;
  }

private:
  /*!
   * \brief Logs once that a copy walked fields instead of memcpy'ing the value array.
   */
  void reportSlowCopyOnce();

  /*!
   * \brief Allocates one slot per recipe field, with the type left undecided.
   */
  void initStorage();

  /*!
   * \brief Recomputes layout_hash_ and fully_typed_ from the current values.
   */
  void updateLayoutHash();

  /*!
   * \brief Rebuilds the name-to-index map from recipe_.
   *
   * The keys are string_views into recipe_, so this must run after recipe_ is in its final place.
   */
  void rebuildFieldIndex();

  /*!
   * \brief The recipe index of \p name, or empty if the name is not in this package.
   */
  std::optional<size_t> fieldIndex(const std::string_view name) const;

  /*!
   * \brief Logs why reading \p field didn't produce the requested type.
   */
  static void reportReadFailure(const std::string_view name, const _rtde_type_variant& field);

  uint8_t recipe_id_ = 0;
  std::vector<std::string> recipe_;
  std::unordered_map<std::string_view, size_t> field_index_;
  std::vector<_rtde_type_variant> values_;
  std::vector<_rtde_type_variant> zeros_;
  uint16_t protocol_version_ = 2;
  uint64_t recipe_hash_ = 0;
  uint64_t layout_hash_ = 0;
  bool fully_typed_ = false;
  bool slow_copy_reported_ = false;
};

}  // namespace rtde_interface
}  // namespace urcl

#endif  // ifndef UR_CLIENT_LIBRARY_DATA_PACKAGE_H_INCLUDED
