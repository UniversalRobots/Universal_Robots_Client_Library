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

#include "ur_client_library/default_log_handler.h"
#include <stdio.h>

namespace urcl
{
DefaultLogHandler::DefaultLogHandler() = default;

void DefaultLogHandler::log(const char* file, int line, LogLevel loglevel, const char* log)
{
  switch (loglevel)
  {
    case LogLevel::INFO:
      printf("%s%s %i: %s \n", "INFO ", file, line, log);
      break;
    case LogLevel::DEBUG:
      printf("%s%s %i: %s \n", "DEBUG ", file, line, log);
      break;
    case LogLevel::WARN:
      printf("%s%s %i: %s \n", "WARN ", file, line, log);
      break;
    case LogLevel::ERROR:
      printf("%s%s %i: %s \n", "ERROR ", file, line, log);
      break;
    case LogLevel::FATAL:
      printf("%s%s %i: %s \n", "FATAL ", file, line, log);
      break;
    default:
      break;
  }
}

}  // namespace urcl
