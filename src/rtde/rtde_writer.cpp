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
 * \date    2019-07-25
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/rtde/rtde_writer.h"
#include <mutex>
#include "ur_client_library/log.h"

namespace urcl
{
namespace rtde_interface
{
const std::vector<std::string> RTDEWriter::g_preallocated_input_bit_register_keys =
    RTDEWriter::initializePreallocatedKeys("input_bit_register_", 128);
const std::vector<std::string> RTDEWriter::g_preallocated_input_int_register_keys =
    RTDEWriter::initializePreallocatedKeys("input_int_register_", 48);
const std::vector<std::string> RTDEWriter::g_preallocated_input_double_register_keys =
    RTDEWriter::initializePreallocatedKeys("input_double_register_", 48);

const std::vector<std::string> RTDEWriter::initializePreallocatedKeys(const std::string& prefix, const size_t size)
{
  std::vector<std::string> keys(size);
  for (size_t i = 0; i < size; ++i)
  {
    keys[i] = prefix + std::to_string(i);
  }
  return keys;
}

RTDEWriter::RTDEWriter(comm::URStream<RTDEPackage>* stream, const std::vector<std::string>& recipe)
  : stream_(stream), recipe_id_(0), running_(false)
{
  setInputRecipe(recipe);

  for (const auto& field : recipe)
  {
    if (field.size() >= 5 && field.substr(field.size() - 5) == "_mask")
    {
      used_masks_.push_back(field);
    }
  }
}

void RTDEWriter::setInputRecipe(const std::vector<std::string>& recipe)
{
  if (running_)
  {
    throw UrException("Requesting to change the input recipe while the RTDEWriter is running. The writer has to be "
                      "stopped before setting the recipe.");
  }
  std::lock_guard<std::mutex> lock_guard(store_mutex_);
  recipe_ = recipe;
  data_buffer0_ = std::make_shared<DataPackage>(recipe_);
  data_buffer1_ = std::make_shared<DataPackage>(recipe_);

  current_store_buffer_ = data_buffer0_;
  current_send_buffer_ = data_buffer1_;
}

void RTDEWriter::init(uint8_t recipe_id)
{
  if (running_)
  {
    throw UrException("Requesting to init a RTDEWriter while it is running. The writer has to be "
                      "stopped before initializing it.");
  }
  {
    std::lock_guard<std::mutex> lock_guard(store_mutex_);
    data_buffer0_->setRecipeID(recipe_id);
    data_buffer1_->setRecipeID(recipe_id);
  }
  recipe_id_ = recipe_id;
  new_data_available_ = false;
  running_ = true;
  writer_thread_ = std::thread(&RTDEWriter::run, this);
}

void RTDEWriter::run()
{
  uint8_t buffer[4096];
  size_t size;
  size_t written;
  while (running_)
  {
    std::unique_lock<std::mutex> lock(store_mutex_);
    data_available_cv_.wait(lock, [this] { return new_data_available_.load() || !running_.load(); });
    {
      if (new_data_available_.load() && running_.load())
      {
        std::swap(current_store_buffer_, current_send_buffer_);
        new_data_available_ = false;
        lock.unlock();
        size = current_send_buffer_->serializePackage(buffer);
        stream_->write(buffer, size, written);
        resetMasks(current_send_buffer_);
      }
      else
      {
        lock.unlock();
      }
    }
  }
  URCL_LOG_DEBUG("Write thread ended.");
}

void RTDEWriter::stop()
{
  {
    std::lock_guard<std::mutex> lock(store_mutex_);
    running_ = false;
    data_available_cv_.notify_one();
  }
  if (writer_thread_.joinable())
  {
    writer_thread_.join();
  }
}

bool RTDEWriter::sendPackage(const DataPackage& package)
{
  std::lock_guard<std::mutex> guard(store_mutex_);
  *current_store_buffer_ = package;
  markStorageToBeSent();
  return true;
}

bool RTDEWriter::sendSpeedSlider(double speed_slider_fraction)
{
  if (speed_slider_fraction > 1.0 || speed_slider_fraction < 0.0)
  {
    std::stringstream ss;
    ss << "Speed slider fraction should be between 0 and 1. The speed slider fraction is "
       << static_cast<int>(speed_slider_fraction);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  static const std::string key = "speed_slider_fraction";
  static const std::string mask_key = "speed_slider_mask";
  std::lock_guard<std::mutex> guard(store_mutex_);
  uint32_t mask = 1;
  bool success = true;
  success = current_store_buffer_->setData(mask_key, mask);
  success = success && current_store_buffer_->setData(key, speed_slider_fraction);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendStandardDigitalOutput(uint8_t output_pin, bool value)
{
  if (output_pin > 7)
  {
    std::stringstream ss;
    ss << "Standard digital output pins goes from 0 to 7. The output pin to change is " << static_cast<int>(output_pin);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  static const std::string key_mask = "standard_digital_output_mask";
  static const std::string key_output = "standard_digital_output";
  std::lock_guard<std::mutex> guard(store_mutex_);
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = current_store_buffer_->setData(key_mask, mask);
  success = success && current_store_buffer_->setData(key_output, digital_output);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendConfigurableDigitalOutput(uint8_t output_pin, bool value)
{
  if (output_pin > 7)
  {
    std::stringstream ss;
    ss << "Configurable digital output pins goes from 0 to 7. The output pin to change is "
       << static_cast<int>(output_pin);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  static const std::string key_mask = "configurable_digital_output_mask";
  static const std::string key_output = "configurable_digital_output";
  std::lock_guard<std::mutex> guard(store_mutex_);
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = current_store_buffer_->setData(key_mask, mask);
  success = success && current_store_buffer_->setData(key_output, digital_output);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendToolDigitalOutput(uint8_t output_pin, bool value)
{
  if (output_pin > 1)
  {
    std::stringstream ss;
    ss << "Tool digital output pins goes from 0 to 1. The output pin to change is " << static_cast<int>(output_pin);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  static const std::string key_mask = "tool_digital_output_mask";
  static const std::string key_output = "tool_digital_output";
  std::lock_guard<std::mutex> guard(store_mutex_);
  uint8_t mask = pinToMask(output_pin);
  bool success = true;
  uint8_t digital_output;
  if (value)
  {
    digital_output = 255;
  }
  else
  {
    digital_output = 0;
  }
  success = current_store_buffer_->setData(key_mask, mask);
  success = success && current_store_buffer_->setData(key_output, digital_output);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendStandardAnalogOutput(uint8_t output_pin, double value, const AnalogOutputType type)
{
  if (output_pin > 1)
  {
    std::stringstream ss;
    ss << "Standard analog output goes from 0 to 1. The output pin to change is " << static_cast<int>(output_pin);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }
  if (value > 1.0 || value < 0.0)
  {
    std::stringstream ss;
    ss << "Analog output value should be between 0 and 1. The value is " << static_cast<double>(value);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  std::lock_guard<std::mutex> guard(store_mutex_);
  uint8_t mask = pinToMask(output_pin);

  bool success = true;
  static const std::string key_mask = "standard_analog_output_mask";
  success = current_store_buffer_->setData(key_mask, mask);
  if (type != AnalogOutputType::SET_ON_TEACH_PENDANT)
  {
    static const std::string key_type = "standard_analog_output_type";
    auto output_type_bits = [](const uint8_t pin, const uint8_t type) { return type << pin; };
    uint8_t output_type = output_type_bits(output_pin, toUnderlying(type));
    success = success && current_store_buffer_->setData(key_type, output_type);
  }
  static const std::string key_output_0 = "standard_analog_output_0";
  success = success && current_store_buffer_->setData(key_output_0, value);
  static const std::string key_output_1 = "standard_analog_output_1";
  success = success && current_store_buffer_->setData(key_output_1, value);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

uint8_t RTDEWriter::pinToMask(uint8_t pin)
{
  if (pin > 7)
  {
    return 0;
  }

  return 1 << pin;
}

bool RTDEWriter::sendInputBitRegister(uint32_t register_id, bool value)
{
  if (register_id < 64 || register_id > 127)
  {
    std::stringstream ss;
    ss << "Input bit register goes from 64 to 127. The register id to change is " << static_cast<int>(register_id);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  std::lock_guard<std::mutex> guard(store_mutex_);
  bool success = current_store_buffer_->setData(g_preallocated_input_bit_register_keys[register_id], value);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendInputIntRegister(uint32_t register_id, int32_t value)
{
  if (register_id < 24 || register_id > 47)
  {
    std::stringstream ss;
    ss << "Input int register goes from 24 to 47. The register id to change is " << static_cast<int>(register_id);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  std::lock_guard<std::mutex> guard(store_mutex_);
  bool success = current_store_buffer_->setData(g_preallocated_input_int_register_keys[register_id], value);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendInputDoubleRegister(uint32_t register_id, double value)
{
  if (register_id < 24 || register_id > 47)
  {
    std::stringstream ss;
    ss << "Input double register goes from 24 to 47. The register id to change is " << static_cast<int>(register_id);
    URCL_LOG_ERROR(ss.str().c_str());
    return false;
  }

  std::lock_guard<std::mutex> guard(store_mutex_);
  bool success = current_store_buffer_->setData(g_preallocated_input_double_register_keys[register_id], value);
  if (success)
  {
    markStorageToBeSent();
  }

  return success;
}

bool RTDEWriter::sendExternalForceTorque(const vector6d_t& external_force_torque)
{
  static const std::string key = "external_force_torque";
  std::lock_guard<std::mutex> guard(store_mutex_);
  bool success = current_store_buffer_->setData(key, external_force_torque);
  if (success)
  {
    markStorageToBeSent();
  }
  return success;
}

void RTDEWriter::resetMasks(const std::shared_ptr<DataPackage>& buffer)
{
  for (const auto& mask_name : used_masks_)
  {
    // "speed_slider_mask" is uint32_t, all others are uint8_t
    // If we reset it to the wrong type, serialization will be wrong
    if (mask_name == "speed_slider_mask")

    {
      uint32_t mask = 0;
      buffer->setData<uint32_t>(mask_name, mask);
    }
    else
    {
      uint8_t mask = 0;
      buffer->setData<uint8_t>(mask_name, mask);
    }
  }
}

void RTDEWriter::markStorageToBeSent()
{
  new_data_available_ = true;
  data_available_cv_.notify_one();
}

}  // namespace rtde_interface
}  // namespace urcl
