// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 Universal Robots A/S
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
    log_level_ = LogLevel::INFO;
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
    std::unique_ptr<char[]> buffer;
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
