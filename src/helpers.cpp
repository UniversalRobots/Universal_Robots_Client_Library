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

#include <ur_client_library/exceptions.h>
#include <ur_client_library/helpers.h>
#include <ur_client_library/log.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>

// clang-format off
// We want to keep the URL in one line to avoid formatting issues. This will make it easier to
// extract the URL for an automatic check.
const std::string RT_DOC_URL = "https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/real_time.html";
// clang-format on

namespace urcl
{
bool setFiFoScheduling(pthread_t& thread, const int priority)
{
#ifdef _WIN32
  return ::SetThreadPriority(thread, priority);
#else  // _WIN32
  struct sched_param params;
  params.sched_priority = priority;
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
  if (ret != 0)
  {
    switch (ret)
    {
      case EPERM:
      {
        URCL_LOG_WARN("Your system/user seems not to be setup for FIFO scheduling. We recommend using a lowlatency "
                      "kernel with FIFO scheduling. See "
                      "%s for details.",
                      RT_DOC_URL.c_str());
        break;
      }
      default:

      {
        URCL_LOG_ERROR("Unsuccessful in setting thread to FIFO scheduling with priority %i. %s", priority,
                       strerror(ret));
      }
    }

    return false;
  }

  // Now verify the change in thread priority
  int policy = 0;
  ret = pthread_getschedparam(thread, &policy, &params);
  if (ret != 0)
  {
    URCL_LOG_ERROR("Couldn't retrieve scheduling parameters");
    return false;
  }

  // Check the correct policy was applied
  if (policy != SCHED_FIFO)
  {
    URCL_LOG_ERROR("Scheduling is NOT SCHED_FIFO!");
    return false;
  }
  else
  {
    URCL_LOG_INFO("SCHED_FIFO OK, priority %i", params.sched_priority);
    if (params.sched_priority != priority)
    {
      URCL_LOG_ERROR("Thread priority is %i instead of the expected %i", params.sched_priority, priority);
      return false;
    }
  }
  return true;
#endif
}

void waitFor(std::function<bool()> condition, const std::chrono::milliseconds timeout,
             const std::chrono::milliseconds check_interval)
{
  auto start_time = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start_time < timeout)
  {
    if (condition())
    {
      return;
    }
    URCL_LOG_DEBUG("Waiting for condition to be met...");
    std::this_thread::sleep_for(check_interval);
  }
  throw urcl::TimeoutException("Timeout while waiting for condition to be met", timeout);
}

bool parseBoolean(const std::string& str)
{
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });

  if (lower == "true" || lower == "1" || lower == "yes" || lower == "on")
  {
    return true;
  }
  else if (lower == "false" || lower == "0" || lower == "no" || lower == "off")
  {
    return false;
  }
  else
  {
    std::stringstream ss;
    ss << "Invalid boolean value: '" << str << "'. Expected 'true', 'false', '1', '0', 'yes', 'no', 'on', or 'off'.";
    URCL_LOG_ERROR(ss.str().c_str());
    throw UrException(ss.str().c_str());
  }
}
}  // namespace urcl
