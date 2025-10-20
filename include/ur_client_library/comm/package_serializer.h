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
 * \author  Tristan Schnell schnell@fzi.de
 * \date    2019-04-10
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_PACKAGE_SERIALIZER_H_INCLUDED
#define UR_CLIENT_LIBRARY_PACKAGE_SERIALIZER_H_INCLUDED

#include <endian.h>
#include <cstdint>
#include <cstring>
#include <string>

#include "ur_client_library/types.h"

namespace urcl
{
namespace comm
{
/*!
 * \brief A helper class to serialize packages. Contains methods for serializing all relevant
 * datatypes.
 */
class PackageSerializer
{
public:
  /*!
   * \brief A generalized serialization method for arbitrary datatypes.
   *
   * @tparam T The type to serialize
   * \param buffer The buffer to write the serialization into
   * \param val The value to serialize
   *
   * \returns Size in byte of the serialization
   */
  template <typename T>
  static size_t serialize(uint8_t* buffer, T val)
  {
    size_t size = sizeof(T);
    T tmp = encode(val);
    std::memcpy(buffer, &tmp, size);
    return size;
  }

  /*!
   * \brief A serialization method for double values.
   *
   * \param buffer The buffer to write the serialization into.
   * \param val The value to serialize.
   *
   * \returns Size in byte of the serialization.
   */
  static size_t serialize(uint8_t* buffer, double val)
  {
    static_assert(sizeof(double) == sizeof(uint64_t), "Unexpected double size");
    size_t size = sizeof(double);
    uint64_t inner;
    std::memcpy(&inner, &val, size);
    inner = encode(inner);
    std::memcpy(buffer, &inner, size);
    return size;
  }

  /*!
   * \brief A serialization method for float values.
   *
   * \param buffer The buffer to write the serialization into.
   * \param val The value to serialize.
   *
   * \returns Size in byte of the serialization.
   */
  static size_t serialize(uint8_t* buffer, float val)
  {
    static_assert(sizeof(float) == sizeof(uint32_t), "Unexpected float size");
    size_t size = sizeof(float);
    uint32_t inner;
    std::memcpy(&inner, &val, size);
    inner = encode(inner);
    std::memcpy(buffer, &inner, size);
    return size;
  }

  /*!
   * \brief A serialization method for strings.
   *
   * \param buffer The buffer to write the serialization into.
   * \param val The string to serialize.
   *
   * \returns Size in byte of the serialization.
   */
  static size_t serialize(uint8_t* buffer, std::string val)
  {
    const uint8_t* c_val = reinterpret_cast<const uint8_t*>(val.c_str());

    for (size_t i = 0; i < val.size(); i++)
    {
      buffer[i] = c_val[i];
    }
    return val.size();
  }

  /*!
   * \brief A serialization method for std::arrays (works for urcl::vector6d_t, etc).
   *
   * \param buffer The buffer to write the serialization into.
   * \param val The array to serialize.
   *
   * \returns Size in byte of the serialization.
   */
  template <typename T, size_t N>
  static size_t serialize(uint8_t* buffer, const std::array<T, N>& val)
  {
    for (size_t i = 0; i < N; ++i)
    {
      serialize(buffer + i * sizeof(T), val[i]);
    }
    return N * sizeof(T);
  }

private:
  template <typename T>
  static T encode(T val)
  {
    return val;
  }
  static uint16_t encode(uint16_t val)
  {
    return htobe16(val);
  }
  static uint32_t encode(uint32_t val)
  {
    return htobe32(val);
  }
  static uint64_t encode(uint64_t val)
  {
    return htobe64(val);
  }
  static int16_t encode(int16_t val)
  {
    return htobe16(val);
  }
  static int32_t encode(int32_t val)
  {
    return htobe32(val);
  }
  static int64_t encode(int64_t val)
  {
    return htobe64(val);
  }
};

}  // namespace comm
}  // namespace urcl
#endif  // UR_CLIENT_LIBRARY_PACKAGE_SERIALIZER_H_INCLUDED
