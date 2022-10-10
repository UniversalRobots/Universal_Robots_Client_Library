// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
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

// All source code contained in and/or linked to in this message (the “Source Code”) is subject to the copyright of
// Universal Robots A/S and/or its licensors. THE SOURCE CODE IS PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING – BUT NOT LIMITED TO – WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR
// NONINFRINGEMENT. USE OF THE SOURCE CODE IS AT YOUR OWN RISK AND UNIVERSAL ROBOTS A/S AND ITS LICENSORS SHALL, TO THE
// MAXIMUM EXTENT PERMITTED BY LAW, NOT BE LIABLE FOR ANY ERRORS OR MALICIOUS CODE IN THE SOURCE CODE, ANY THIRD-PARTY
// CLAIMS, OR ANY OTHER CLAIMS AND DAMAGES, INCLUDING INDIRECT, INCIDENTAL, SPECIAL, CONSEQUENTIAL OR PUNITIVE DAMAGES,
// OR ANY LOSS OF PROFITS, EXPECTED SAVINGS, OR REVENUES, WHETHER INCURRED DIRECTLY OR INDIRECTLY, OR ANY LOSS OF DATA,
// USE, GOODWILL, OR OTHER INTANGIBLE LOSSES, RESULTING FROM YOUR USE OF THE SOURCE CODE. You may make copies of the
// Source Code for use in connection with a Universal Robots or UR+ product, provided that you include (i) an
// appropriate copyright notice (“©  [the year in which you received the Source Code or the Source Code was first
// published, e.g. “2021”] Universal Robots A/S and/or its licensors”) along with the capitalized section of this notice
// in all copies of the Source Code. By using the Source Code, you agree to the above terms. For more information,
// please contact legal@universal-robots.com.
// -- END LICENSE BLOCK ------------------------------------------------

#include "ur_client_library/log.h"
#include "ur_client_library/default_log_handler.h"
#include <cstdarg>
#include <cstdio>

namespace urcl
{
class Logger
{
public:
  Logger()
  {
    log_level_ = LogLevel::WARN;
    log_handler_.reset(new DefaultLogHandler());
  }

  ~Logger()
  {
    if (log_handler_)
    {
      log_handler_.reset();
    }
  }

  void registerLogHandler(std::unique_ptr<LogHandler> loghandler)
  {
    log_handler_ = std::move(loghandler);
  }

  void unregisterLogHandler()
  {
    log_handler_.reset(new DefaultLogHandler());
  }

  void log(const char* file, int line, LogLevel level, const char* txt)
  {
    if (!log_handler_)
    {
      log_handler_.reset(new DefaultLogHandler());
    }

    log_handler_->log(file, line, level, txt);
  }

  void setLogLevel(LogLevel level)
  {
    log_level_ = level;
  }

  LogLevel getLogLevel()
  {
    return log_level_;
  }

private:
  std::unique_ptr<LogHandler> log_handler_;
  LogLevel log_level_;
};
Logger g_logger;

void registerLogHandler(std::unique_ptr<LogHandler> loghandler)
{
  g_logger.registerLogHandler(std::move(loghandler));
}

void unregisterLogHandler()
{
  g_logger.unregisterLogHandler();
}

void setLogLevel(LogLevel level)
{
  g_logger.setLogLevel(level);
}

void log(const char* file, int line, LogLevel level, const char* fmt, ...)
{
  if (level >= g_logger.getLogLevel())
  {
    size_t buffer_size = 1024;
    std::unique_ptr<char> buffer;
    buffer.reset(new char[buffer_size]);

    va_list args;
    va_start(args, fmt);
    va_list args_copy;
    va_copy(args_copy, args);

    size_t characters = 1 + std::vsnprintf(buffer.get(), buffer_size, fmt, args);

    if (characters >= buffer_size)
    {
      buffer_size = characters + 1;
      buffer.reset(new char[buffer_size]);
      std::vsnprintf(buffer.get(), buffer_size, fmt, args_copy);
    }

    va_end(args);
    va_end(args_copy);

    g_logger.log(file, line, level, buffer.get());
  }
}

}  // namespace urcl
