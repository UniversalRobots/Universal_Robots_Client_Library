// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
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
 * \date    2022-12-15
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_HELPERS_H_INCLUDED
#define UR_CLIENT_LIBRARY_HELPERS_H_INCLUDED

#include <string>
#include <chrono>
#include <functional>
#ifdef _WIN32

#  define NOMINMAX
#  define WIN32_LEAN_AND_MEAN
#  include <Windows.h>

#  ifdef ERROR
#    undef ERROR
#  endif  // ERROR

#  define SCHED_FIFO (1)

typedef HANDLE pthread_t;

static inline pthread_t pthread_self()
{
  return ::GetCurrentThread();
}

static inline int sched_get_priority_max(int policy)
{
  (void)policy;
  return THREAD_PRIORITY_TIME_CRITICAL;
}

#else  // _WIN32

#  include <pthread.h>

#endif  // _WIN32

namespace urcl
{
bool setFiFoScheduling(pthread_t& thread, const int priority);

/*!
 * \brief Wait for a condition to be true.
 *
 * This function will wait for a condition to be true. The condition is checked in intervals of \p check_interval.
 * If the condition is not met after \p timeout, the function will throw a urcl::TimeoutException.
 *
 * \param condition The condition to be checked.
 * \param timeout The maximum time to wait for the condition to be true.
 * \param check_interval The interval in which the condition is checked.
 */
void waitFor(std::function<bool()> condition, const std::chrono::milliseconds timeout,
             const std::chrono::milliseconds check_interval = std::chrono::milliseconds(50));

/*!
 * \brief Parses a boolean value from a string.
 *
 * The string can be one of
 *  - true, True, TRUE
 *  - on, On, ON
 *  - yes, Yes, YES
 *  - 1
 *  - false, False, FALSE
 *  - off, Off, OFF
 *  - no, No, NO
 *  - 0
 *
 * \param str string to be parsed
 * \throws urcl::UrException If the string doesn't match one of the options
 * \return The boolean representation of the string
 */
bool parseBoolean(const std::string& str);

}  // namespace urcl
#endif  // ifndef UR_CLIENT_LIBRARY_HELPERS_H_INCLUDED
