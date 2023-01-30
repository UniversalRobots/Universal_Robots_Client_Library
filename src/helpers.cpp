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

#include <ur_client_library/helpers.h>
#include <ur_client_library/log.h>

#include <cstring>
#include <fstream>
#include <iostream>

namespace urcl
{
bool setFiFoScheduling(pthread_t& thread, const int priority)
{
  struct sched_param params;
  params.sched_priority = priority;
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
  if (ret != 0)
  {
    switch (ret)
    {
      case EPERM:
      {
        URCL_LOG_ERROR("Your system/user seems not to be setup for FIFO scheduling. We recommend using a lowlatency "
                       "kernel with FIFO scheduling. See "
                       "https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/"
                       "doc/real_time.md for details.");
        break;
      }
      default:

      {
        URCL_LOG_ERROR("Unsuccessful in setting thread to FIFO scheduling with priority %i. %s", priority,
                       strerror(ret));
      }
    }
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
      return false;
    }
  }
  return true;
}
}  // namespace urcl
