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
 * \date    2026-09-25
 *
 */
//----------------------------------------------------------------------

#include "ur_client_library/rtde/robot_properties.h"

#include <memory>
#include <vector>

#include "ur_client_library/exceptions.h"
#include "ur_client_library/log.h"
#include "ur_client_library/rtde/text_message.h"

namespace urcl
{
namespace rtde_interface
{
namespace
{
// Packages a controller is expected to send ahead of the answer, such as notices on connect.
// Reading continues past this, so a late answer is still taken off the stream.
constexpr unsigned MAX_UNEXPECTED_PACKAGES = 5;
// Packages read before giving up on the answer. A controller still sending at this point would
// leave the answer for the next setup step, so the connection has to be set up again instead.
constexpr unsigned MAX_RECEIVED_PACKAGES = 32;

enum class Exchange
{
  ANSWERED,
  NOT_ANSWERED,
  // The stream may still hold the answer, so the next request could be paired with it.
  OUT_OF_SYNC
};

// Waits for the answer to a request that has just been sent. A text message does not end the
// wait: the controller sends notices such as "SafetySetup has not been confirmed yet" on connect,
// and the answer follows them. Stopping before the answer or a read timeout would leave the answer
// queued for the next setup step to trip over.
Exchange receiveAnswer(comm::URProducer<RTDEPackage>& producer, ReadProperties& properties)
{
  std::unique_ptr<RTDEPackage> package = std::make_unique<ReadProperties>();
  for (unsigned received = 0; received < MAX_RECEIVED_PACKAGES; ++received)
  {
    if (received == MAX_UNEXPECTED_PACKAGES + 1)
    {
      URCL_LOG_WARN("Still waiting for the RTDE_READ_PROPERTIES answer after %u other packages", received);
    }
    if (!producer.tryGet(package))
    {
      URCL_LOG_ERROR("No answer to RTDE_READ_PROPERTIES was received");
      return Exchange::NOT_ANSWERED;
    }
    if (const ReadProperties* answer = dynamic_cast<const ReadProperties*>(package.get()))
    {
      return properties.takeAnswer(*answer) ? Exchange::ANSWERED : Exchange::NOT_ANSWERED;
    }
    if (const TextMessage* text = dynamic_cast<const TextMessage*>(package.get()))
    {
      URCL_LOG_INFO("Text message from the controller while reading properties: %s", text->message_.c_str());
      continue;
    }
    URCL_LOG_WARN("Unexpected RTDE package while reading properties:\n%s", package->toString().c_str());
  }
  URCL_LOG_ERROR("No answer to RTDE_READ_PROPERTIES among %u packages from the controller", MAX_RECEIVED_PACKAGES);
  return Exchange::OUT_OF_SYNC;
}

// One request/answer round trip for the names in \p properties.
Exchange readProperties(comm::URStream<RTDEPackage>& stream, comm::URProducer<RTDEPackage>& producer,
                        ReadProperties& properties)
{
  properties.clearAnswer();
  uint8_t buffer[4096];
  const size_t size = properties.serializeRequest(buffer, sizeof(buffer));
  if (size == 0)
  {
    URCL_LOG_ERROR("RTDE_READ_PROPERTIES requires a non-empty list of non-blank property names");
    return Exchange::NOT_ANSWERED;
  }
  size_t written = 0;
  if (!stream.write(buffer, size, written))
  {
    URCL_LOG_ERROR("Sending RTDE_READ_PROPERTIES request failed");
    return Exchange::NOT_ANSWERED;
  }
  const Exchange exchange = receiveAnswer(producer, properties);
  if (exchange != Exchange::ANSWERED)
  {
    properties.clearAnswer();
    return exchange;
  }
  if (!properties.hasValues())
  {
    // The controller drops every value when any one name is NOT_FOUND or NOT_SET.
    for (size_t i = 0; i < properties.size(); ++i)
    {
      if (!properties.dataType(i).has_value())
      {
        URCL_LOG_WARN("The controller reported no value for robot property %s", properties.names()[i].c_str());
      }
    }
    properties.clearAnswer();
    return Exchange::NOT_ANSWERED;
  }
  return Exchange::ANSWERED;
}

// Two round trips. A single unknown name would make the controller drop every value in the
// answer, so the software version is read on its own first and then used to pick only the
// catalog names this controller has.
Exchange readSupportedProperties(comm::URStream<RTDEPackage>& stream, comm::URProducer<RTDEPackage>& producer,
                                 ReadProperties& properties)
{
  ReadProperties software_version(std::vector<std::string>{ "v1.software.version" });
  const Exchange exchange = readProperties(stream, producer, software_version);
  if (exchange != Exchange::ANSWERED)
  {
    properties.clearAnswer();
    return exchange;
  }
  const std::optional<VersionInformation> version = software_version.getSoftwareVersion();
  if (!version.has_value())
  {
    // Without a usable version it is unknown which other names are safe to ask for.
    URCL_LOG_WARN("The controller reported v1.software.version with an unexpected type");
    properties.clearAnswer();
    return Exchange::NOT_ANSWERED;
  }
  properties.setNames(propertyNamesForSoftwareVersion(*version, builtinPropertyCatalog()));
  return readProperties(stream, producer, properties);
}
}  // namespace

bool RobotProperties::fetch(comm::URStream<RTDEPackage>& stream, comm::URProducer<RTDEPackage>& producer)
{
  // Read into a local package so the mutex is only held for the copy, not for the round trips.
  // Otherwise an application thread calling get() during a reconnect would wait on the network.
  ReadProperties fetched;
  Exchange exchange = Exchange::NOT_ANSWERED;
  try
  {
    exchange = readSupportedProperties(stream, producer, fetched);
  }
  catch (const UrException& error)
  {
    // The properties are optional, so a malformed answer must not abort the RTDE handshake.
    URCL_LOG_ERROR("Parsing the RTDE_READ_PROPERTIES answer failed: %s", error.what());
  }
  const bool read = exchange == Exchange::ANSWERED;
  if (!read)
  {
    URCL_LOG_WARN("Could not read the RTDE robot properties. RTDEClient::getRobotProperties() will not provide any.");
  }
  // A failed read also forgets the previous controller's properties, so after a reconnect to a
  // different controller get() never reports stale data.
  std::lock_guard<std::mutex> lock(mutex_);
  valid_ = read;
  if (read)
  {
    properties_.copyFrom(fetched);
  }
  else
  {
    properties_.clearAnswer();
  }
  return exchange != Exchange::OUT_OF_SYNC;
}

void RobotProperties::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  valid_ = false;
  properties_.clearAnswer();
}

bool RobotProperties::get(ReadProperties& properties) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_)
  {
    return false;
  }
  properties.copyFrom(properties_);
  return true;
}
}  // namespace rtde_interface
}  // namespace urcl
