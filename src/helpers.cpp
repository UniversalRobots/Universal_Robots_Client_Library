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
#include <cctype>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <cerrno>

// clang-format off
// We want to keep the URL in one line to avoid formatting issues. This will make it easier to
// extract the URL for an automatic check.
const std::string RT_DOC_URL = "https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/real_time.html";
// clang-format on

namespace urcl
{

#ifdef _WIN32

const char* processPriorityToString(DWORD priority)
{
  switch (priority)
  {
    case IDLE_PRIORITY_CLASS:
      return "IDLE";
    case BELOW_NORMAL_PRIORITY_CLASS:
      return "BELOW_NORMAL";
    case NORMAL_PRIORITY_CLASS:
      return "NORMAL";
    case ABOVE_NORMAL_PRIORITY_CLASS:
      return "ABOVE_NORMAL";
    case HIGH_PRIORITY_CLASS:
      return "HIGH";
    case REALTIME_PRIORITY_CLASS:
      return "REALTIME";
    default:
      return "UNKNOWN";
  }
}

const char* threadPriorityToString(int priority)
{
  switch (priority)
  {
    case THREAD_PRIORITY_LOWEST:
      return "LOWEST";
    case THREAD_PRIORITY_BELOW_NORMAL:
      return "BELOW_NORMAL";
    case THREAD_PRIORITY_NORMAL:
      return "NORMAL";
    case THREAD_PRIORITY_ABOVE_NORMAL:
      return "ABOVE_NORMAL";
    case THREAD_PRIORITY_HIGHEST:
      return "HIGHEST";
    case THREAD_PRIORITY_TIME_CRITICAL:
      return "TIME_CRITICAL";
    default:
      return "UNKNOWN";
  }
}

bool isProcessElevated()
{
  bool isElevated = false;
  HANDLE hToken = nullptr;
  if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &hToken))
  {
    TOKEN_ELEVATION elevation;
    DWORD dwSize = sizeof(elevation);
    if (GetTokenInformation(hToken, TokenElevation, &elevation, sizeof(elevation), &dwSize))
    {
      isElevated = elevation.TokenIsElevated;
    }
  }
  if (hToken)
  {
    CloseHandle(hToken);
  }
  return isElevated;
}

std::string maskToCpuList(uint64_t mask)
{
  std::string out = "[";
  for (int i = 0; i < 64; ++i)
  {
    if (mask & (1ULL << i))
    {
      if (out.size() > 1)
        out += ",";
      out += std::to_string(i);
    }
  }
  out += "]";
  return out;
}

std::string getLastWindowsErrorMsg(DWORD error_id)
{
  LPSTR buffer = nullptr;

  DWORD size =
      FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
                     nullptr, error_id, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, nullptr);

  if (size == 0 || buffer == nullptr)
  {
    return "Unknown Windows error";
  }

  std::string message(buffer, size);
  LocalFree(buffer);

  return message;
};

bool setProcessPriority(pprocess_t& process, DWORD priority)
{
  if (!isProcessElevated() && priority == REALTIME_PRIORITY_CLASS)
  {
    URCL_LOG_WARN("Process is not running with elevated privileges (UAC). "
                  "REALTIME_PRIORITY_CLASS may fail. Try 'Run as Administrator'.");
  }

  if (!::SetPriorityClass(process, priority))
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in setting the process priority to %s (%llX). Error: %lu (%s)",
                   processPriorityToString(priority), priority, err, getLastWindowsErrorMsg(err).c_str());
    return false;
  }

  DWORD priority_applied = ::GetPriorityClass(process);
  if (priority_applied == 0)
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in retrieving the process priority for verification. Error: %lu (%s)", err,
                   getLastWindowsErrorMsg(err).c_str());
    return false;
  }

  URCL_LOG_INFO("Process priority successfully set to %s (0x%X)", processPriorityToString(priority_applied),
                priority_applied);

  if (priority_applied != priority)
  {
    URCL_LOG_WARN("Process priority mismatch. Expected %s (0x%X), got %s (0x%X)", processPriorityToString(priority),
                  priority, processPriorityToString(priority_applied), priority_applied);
    return false;
  }

  return true;
}

bool setProcessAffinity(pprocess_t& process, DWORD_PTR cpu_mask)
{
  if (!::SetProcessAffinityMask(process, cpu_mask))
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in setting process affinity to CPUs %s (mask=0x%llX). Error: %lu (%s)",
                   maskToCpuList(cpu_mask).c_str(), static_cast<uint64_t>(cpu_mask), err,
                   getLastWindowsErrorMsg(err).c_str());

    return false;
  }
  DWORD_PTR process_mask = 0;
  DWORD_PTR system_mask = 0;
  if (!::GetProcessAffinityMask(process, &process_mask, &system_mask))
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in setting process affinity to %s. Error: %lu (%s)", maskToCpuList(cpu_mask).c_str(),
                   err, getLastWindowsErrorMsg(err).c_str());
    return false;
  }
  URCL_LOG_INFO("Process affinity set to CPUs %s (mask=0x%llX)", maskToCpuList(process_mask).c_str(),
                static_cast<uint64_t>(process_mask));
  if (process_mask != cpu_mask)
  {
    URCL_LOG_WARN("Process affinity mismatch. Expected %s, got %s", maskToCpuList(cpu_mask).c_str(),
                  maskToCpuList(process_mask).c_str());
    return false;
  }
  return true;
}

bool setThreadAffinity(pthread_t& thread, DWORD_PTR cpu_mask)
{
  DWORD_PTR result = ::SetThreadAffinityMask(thread, cpu_mask);

  if (result == 0)
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in setting thread affinity to %s. Error: %lu (%s)", maskToCpuList(cpu_mask).c_str(),
                   err, getLastWindowsErrorMsg(err).c_str());
    return false;
  }

  URCL_LOG_INFO("Thread affinity successfully set to CPUs %s (mask=0x%llX)", maskToCpuList(cpu_mask).c_str(),
                static_cast<uint64_t>(cpu_mask));
  return true;
}

bool setThreadPriority(pthread_t& thread, const int priority)
{
  if (!::SetThreadPriority(thread, priority))
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in setting thread priority to %s (%d). Error: %lu (%s)",
                   threadPriorityToString(priority), priority, err, getLastWindowsErrorMsg(err).c_str());

    return false;
  }

  int applied = ::GetThreadPriority(thread);
  if (applied == THREAD_PRIORITY_ERROR_RETURN)
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in retrieving the thread priority for verification. Error: %lu (%s)", err,
                   getLastWindowsErrorMsg(err).c_str());
    return false;
  }

  URCL_LOG_INFO("Thread priority successfully set to %s (%d)", threadPriorityToString(applied), applied);

  if (applied != priority)
  {
    URCL_LOG_WARN("Thread priority mismatch. Expected %s (%d), got %s (%d)", threadPriorityToString(priority), priority,
                  threadPriorityToString(applied), applied);
    return false;
  }

  return true;
}

#elif __linux__

std::string cpuSetToString(const cpu_set_t& cpuset)
{
  std::string out = "[";

  for (int i = 0; i < CPU_SETSIZE; ++i)
  {
    if (CPU_ISSET(i, &cpuset))
    {
      if (out.size() > 1)
      {
        out += ",";
      }
      out += std::to_string(i);
    }
  }

  out += "]";
  return out;
}

bool setThreadAffinity(pthread_t& thread, const cpu_set_t& cpuset)
{
  int ret = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

  if (ret != 0)
  {
    URCL_LOG_ERROR("Unsuccessful in setting thread affinity. Error: %s", strerror(ret));
    return false;
  }

  cpu_set_t applied_set;
  CPU_ZERO(&applied_set);

  if (pthread_getaffinity_np(thread, sizeof(cpu_set_t), &applied_set) == 0)
  {
    bool match = true;

    for (int i = 0; i < CPU_SETSIZE; ++i)
    {
      if (CPU_ISSET(i, &cpuset) != CPU_ISSET(i, &applied_set))
      {
        match = false;
        break;
      }
    }

    if (!match)
    {
      URCL_LOG_WARN("Thread affinity mismatch. Requested %s, got %s", cpuSetToString(cpuset).c_str(),
                    cpuSetToString(applied_set).c_str());

      return false;
    }
  }
  else
  {
    URCL_LOG_WARN("Could not retrieve thread affinity");
    return false;
  }

  URCL_LOG_INFO("Thread affinity successfully set to CPUs %s", cpuSetToString(applied_set).c_str());

  return true;
}

#endif

bool setFiFoScheduling(pthread_t& thread, int priority)
{
#ifdef _WIN32
  pprocess_t process = pprocess_self();

  DWORD old_process_priority = ::GetPriorityClass(process);
  if (old_process_priority == 0)
  {
    DWORD err = GetLastError();
    URCL_LOG_ERROR("Unsuccessful in retrieving the current process priority. Error: %lu (%s)", err,
                   getLastWindowsErrorMsg(err).c_str());
    return false;
  }

  if (!setProcessPriority(process, REALTIME_PRIORITY_CLASS))
  {
    URCL_LOG_ERROR("Unsuccessful in setting process to REALTIME_PRIORITY_CLASS");
    return false;
  }

  if (!setThreadPriority(thread, priority))
  {
    URCL_LOG_ERROR("Unsuccessful in setting thread priority to %s (%d)", threadPriorityToString(priority), priority);

    if (!setProcessPriority(process, old_process_priority))
    {
      URCL_LOG_ERROR("Failed to restore previous process priority %s (0x%X)",
                     processPriorityToString(old_process_priority), old_process_priority);
    }

    return false;
  }

  return true;

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
                       strerror_portable(ret).c_str());
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
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

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

std::vector<std::string> splitString(const std::string& input, const std::string& delimiter)
{
  std::vector<std::string> result;
  size_t pos = 0;
  size_t pos_end = pos;
  std::string substring;
  while ((pos_end = input.find(delimiter, pos)) != std::string::npos)
  {
    substring = input.substr(pos, pos_end - pos);
    result.push_back(substring);
    pos = pos_end + delimiter.length();
  }
  substring = input.substr(pos);
  result.push_back(substring);
  return result;
}

RobotSeries robotSeriesFromTypeAndVersion(const RobotType type, const VersionInformation& version_info)
{
  switch (type)
  {
    case RobotType::UR3:
    case RobotType::UR5:
    case RobotType::UR10:
      if (version_info.major >= 5)
      {
        return RobotSeries::E_SERIES;
      }
      else
      {
        return RobotSeries::CB3;
      }
    case RobotType::UR16:
      if (version_info.major >= 5)
      {
        return RobotSeries::E_SERIES;
      }
      else
      {
        return RobotSeries::UNDEFINED;
      }
    case RobotType::UR15:
    case RobotType::UR18:
    case RobotType::UR20:
    case RobotType::UR30:
    case RobotType::UR8LONG:
      if (version_info.major >= 5)
      {
        return RobotSeries::UR_SERIES;
      }
      else
      {
        return RobotSeries::UNDEFINED;
      }
    case RobotType::UNDEFINED:
      return RobotSeries::UNDEFINED;
  }
  return RobotSeries::UNDEFINED;
}

RobotType robotTypeFromString(const std::string& robot_type_str)
{
  // RobotType has no dedicated entries for UR7/UR12, so UR7e and UR12e are mapped to their
  // closest siblings UR5 and UR10 respectively, matching what the robot reports over primary.
  static const std::unordered_map<std::string, RobotType> string_to_robot_type{
    { "ur3", RobotType::UR3 },    { "ur3e", RobotType::UR3 },        { "ur5", RobotType::UR5 },
    { "ur5e", RobotType::UR5 },   { "ur7e", RobotType::UR5 },        { "ur10", RobotType::UR10 },
    { "ur10e", RobotType::UR10 }, { "ur12e", RobotType::UR10 },      { "ur16e", RobotType::UR16 },
    { "ur15", RobotType::UR15 },  { "ur18", RobotType::UR18 },       { "ur20", RobotType::UR20 },
    { "ur30", RobotType::UR30 },  { "ur8long", RobotType::UR8LONG },
  };

  const auto it = string_to_robot_type.find(robot_type_str);
  if (it == string_to_robot_type.end())
  {
    throw std::invalid_argument("Unknown robot type: " + robot_type_str);
  }
  return it->second;
}

}  // namespace urcl
