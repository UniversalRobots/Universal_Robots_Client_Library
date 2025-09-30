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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-11
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_EXCEPTIONS_H_INCLUDED
#define UR_CLIENT_LIBRARY_EXCEPTIONS_H_INCLUDED

#include <chrono>
#include <optional>
#include <stdexcept>
#include <sstream>
#include "ur/version_information.h"

#ifdef _WIN32
#  define NOMINMAX
#  define WIN32_LEAN_AND_MEAN
#  include <WinSock2.h>
#  ifdef ERROR
#    undef ERROR
#  endif  // ERROR
#else
#  include <sys/time.h>
#endif

namespace urcl
{
/*!
 * \brief Our base class for exceptions. Specialized exceptions should inherit from those.
 */
class UrException : virtual public std::runtime_error
{
public:
  explicit UrException() : std::runtime_error("")
  {
  }
  explicit UrException(const std::string& what_arg) : std::runtime_error(what_arg)
  {
  }
  explicit UrException(const char* what_arg) : std::runtime_error(what_arg)
  {
  }

  virtual ~UrException() = default;

private:
  /* data */
};

/*!
 * \brief A specialized exception representing detection of a not supported UR control version.
 */
class VersionMismatch : public UrException
{
public:
  explicit VersionMismatch() : VersionMismatch("", VersionInformation(), VersionInformation())
  {
  }
  explicit VersionMismatch(const std::string& text, const VersionInformation version_req,
                           const VersionInformation version_actual)
    : std::runtime_error(text)
  {
    version_required_ = version_req;
    version_actual_ = version_actual;
    std::stringstream ss;
    ss << text << "(Required version: " << version_required_ << ", actual version: " << version_actual_ << ")";
    text_ = ss.str();
  }
  virtual ~VersionMismatch() = default;

  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }

private:
  VersionInformation version_required_;
  VersionInformation version_actual_;
  std::string text_;
};

/*!
 * \brief A specialized exception representing that communication to the tool is not possible.
 */
class ToolCommNotAvailable : public VersionMismatch
{
public:
  explicit ToolCommNotAvailable() : ToolCommNotAvailable("", VersionInformation(), VersionInformation())
  {
  }
  explicit ToolCommNotAvailable(const std::string& text, const VersionInformation version_req,
                                const VersionInformation version_actual)
    : std::runtime_error(text), VersionMismatch(text, version_req, version_actual)
  {
  }
};

/*!
 * \brief A specialized exception representing that communication to the tool is not possible.
 */
class TimeoutException : public UrException
{
public:
  explicit TimeoutException() = delete;
  explicit TimeoutException(const std::string& text, std::chrono::milliseconds timeout) : std::runtime_error(text)
  {
    std::stringstream ss;
    ss << text << "(Configured timeout: " << timeout.count() / 1000.0 << " sec)";
    text_ = ss.str();
  }

  explicit TimeoutException(const std::string& text, timeval timeout) : std::runtime_error(text)
  {
    std::stringstream ss;
    ss << text << "(Configured timeout: " << timeout.tv_sec + timeout.tv_usec * 1e-6 << " sec)";
    text_ = ss.str();
  }
  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }

private:
  std::string text_;
};

class IncompatibleRobotVersion : public UrException
{
public:
  explicit IncompatibleRobotVersion() = delete;
  explicit IncompatibleRobotVersion(const std::string& text, const VersionInformation& minimum_robot_version,
                                    const VersionInformation& actual_robot_version)
    : std::runtime_error(text)
  {
    std::stringstream ss;
    ss << text << "\n"
       << "The requested feature is incompatible with the connected robot. Minimum required Polyscope version: "
       << minimum_robot_version << ", actual Polyscope version: " << actual_robot_version;
    text_ = ss.str();
  }
  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }

private:
  std::string text_;
};

class InvalidRange : public UrException
{
private:
  std::string text_;

public:
  explicit InvalidRange() = delete;
  explicit InvalidRange(std::string text) : std::runtime_error(text)
  {
    text_ = text;
  }
  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }
};

class MissingArgument : public UrException
{
private:
  std::string text_;

public:
  explicit MissingArgument() = delete;
  explicit MissingArgument(std::string text, std::string function_name, std::string argument_name, float default_value)
    : std::runtime_error("")
  {
    std::stringstream ss;
    ss << text << "\nMissing argument when calling function: " << function_name
       << ". \nArgument missing: " << argument_name
       << ". \nSet to default value if not important, default value is: " << default_value;
    text_ = ss.str();
  }
  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }
};

class UnsupportedMotionType : public UrException
{
public:
  explicit UnsupportedMotionType() : std::runtime_error("Unsupported motion type")
  {
  }
};

class UnknownVariable : public UrException
{
private:
  std::string text_;

public:
  explicit UnknownVariable() = delete;
  explicit UnknownVariable(const std::string& variable_name) : std::runtime_error("Unknown variable: " + variable_name)
  {
  }
};

class NotImplementedException : public UrException
{
public:
  explicit NotImplementedException() : std::runtime_error("Not implemented")
  {
  }
  explicit NotImplementedException(const std::string& text) : std::runtime_error(text)
  {
  }
  virtual ~NotImplementedException() = default;

  virtual const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};

class UnexpectedResponse : public UrException
{
public:
  explicit UnexpectedResponse() : std::runtime_error("Unexpected response")
  {
  }
  explicit UnexpectedResponse(const std::string& text) : std::runtime_error(text)
  {
    text_ = text;
  }
  explicit UnexpectedResponse(const std::string& text, int status) : std::runtime_error(text), status(status)
  {
    std::stringstream ss;
    ss << "Message: " << text << ". \nStatus code: " << status;
    text_ = ss.str();
  }
  virtual ~UnexpectedResponse() = default;

  virtual const char* what() const noexcept override
  {
    return text_.c_str();
  }
  std::optional<int> status = std::nullopt;  // Optional status code, if available
private:
  std::string text_;
};
}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_EXCEPTIONS_H_INCLUDED
