/*
 * Copyright 2024, RoboDK Inc.
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

#ifdef _WIN32

#  define NOMINMAX
#  define WIN32_LEAN_AND_MEAN
#  include <WinSock2.h>
#  include <ws2tcpip.h>

#  ifndef TCP_QUICKACK
#    define TCP_QUICKACK 12
#  endif

#  ifdef ERROR
#    undef ERROR
#  endif  // ERROR

typedef SOCKET socket_t;
typedef SSIZE_T ssize_t;

static inline int ur_setsockopt(socket_t s, int level, int optname, const void* optval, unsigned int optlen)
{
  return ::setsockopt(s, level, optname, reinterpret_cast<const char*>(optval), static_cast<int>(optlen));
}

static inline int ur_close(socket_t s)
{
  return ::closesocket(s);
}

#else  // _WIN32

#  include <arpa/inet.h>
#  include <netdb.h>
#  include <sys/select.h>
#  include <sys/socket.h>
#  include <sys/time.h>
#  include <sys/types.h>
#  include <unistd.h>

typedef int socket_t;

#  ifndef INVALID_SOCKET
#    define INVALID_SOCKET (-1)
#  endif

#  define ur_setsockopt setsockopt
#  define ur_close close

#endif  //  _WIN32
