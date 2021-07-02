/*
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
#include <inttypes.h>
#include <memory>

#define URCL_LOG_DEBUG(...) urcl::log(__FILE__, __LINE__, urcl::LogLevel::DEBUG, __VA_ARGS__)
#define URCL_LOG_WARN(...) urcl::log(__FILE__, __LINE__, urcl::LogLevel::WARN, __VA_ARGS__)
#define URCL_LOG_INFO(...) urcl::log(__FILE__, __LINE__, urcl::LogLevel::INFO, __VA_ARGS__)
#define URCL_LOG_ERROR(...) urcl::log(__FILE__, __LINE__, urcl::LogLevel::ERROR, __VA_ARGS__)
#define URCL_LOG_FATAL(...) urcl::log(__FILE__, __LINE__, urcl::LogLevel::FATAL, __VA_ARGS__)

namespace urcl
{
/*!
 * \brief Different log levels
 */
enum class LogLevel
{
  DEBUG = 0,
  INFO,
  WARN,
  ERROR,
  FATAL,
  NONE
};

/*!
 * \brief Inherit from this class to change the behavior when logging messages.
 */
class LogHandler
{
public:
  virtual ~LogHandler() = default;
  /*!
   * \brief Function to log a message
   *
   * \param file The log message comes from this file
   * \param line The log message comes from this line
   * \param loglevel Indicates the severity of the log message
   * \param log Log message
   */
  virtual void log(const char* file, int line, LogLevel loglevel, const char* log) = 0;
};

/*!
 * \brief Register a new LogHandler object, for handling log messages.
 *
 * \param loghandler Pointer to the new object
 */
void registerLogHandler(std::unique_ptr<LogHandler> loghandler);

/*!
 * \brief Unregister current log handler, this will enable default log handler.
 */
void unregisterLogHandler();

/*!
 * \brief Set log level this will disable messages with lower log level.
 *
 * \param level desired log level
 */
void setLogLevel(LogLevel level);

/*!
 * \brief Log a message, this is used internally by the macros to unpack the log message.
 * Use the macros instead of this function directly.
 *
 * \param file The log message comes from this file
 * \param line The log message comes from this line
 * \param level Severity of the log message
 * \param fmt Format string
 */
void log(const char* file, int line, LogLevel level, const char* fmt, ...);

}  // namespace urcl
