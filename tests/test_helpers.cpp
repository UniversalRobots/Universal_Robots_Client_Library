// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2025 Universal Robots A/S
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

#include <gtest/gtest.h>

#include <ur_client_library/helpers.h>
#include <ur_client_library/exceptions.h>
#include <ur_client_library/ur/datatypes.h>
#include <ur_client_library/ur/version_information.h>

using namespace urcl;

TEST(TestHelpers, test_parse_boolean)
{
  EXPECT_TRUE(parseBoolean("true"));
  EXPECT_TRUE(parseBoolean("True"));
  EXPECT_TRUE(parseBoolean("TRUE"));
  EXPECT_TRUE(parseBoolean("on"));
  EXPECT_TRUE(parseBoolean("On"));
  EXPECT_TRUE(parseBoolean("ON"));
  EXPECT_TRUE(parseBoolean("yes"));
  EXPECT_TRUE(parseBoolean("Yes"));
  EXPECT_TRUE(parseBoolean("YES"));
  EXPECT_TRUE(parseBoolean("1"));
  EXPECT_FALSE(parseBoolean("false"));
  EXPECT_FALSE(parseBoolean("False"));
  EXPECT_FALSE(parseBoolean("FALSE"));
  EXPECT_FALSE(parseBoolean("off"));
  EXPECT_FALSE(parseBoolean("Off"));
  EXPECT_FALSE(parseBoolean("OFF"));
  EXPECT_FALSE(parseBoolean("no"));
  EXPECT_FALSE(parseBoolean("No"));
  EXPECT_FALSE(parseBoolean("NO"));
  EXPECT_FALSE(parseBoolean("0"));
  EXPECT_THROW(parseBoolean("notabool"), urcl::UrException);
}

TEST(TestHelpers, splitString)
{
  std::vector<std::string> test_vec{ "this", "is", "very", "simple" };
  {
    std::string combined = test_vec[0];
    for (std::size_t i = 1; i < test_vec.size(); ++i)
    {
      combined += "," + test_vec[i];
    }

    EXPECT_EQ(test_vec, urcl::splitString(combined, ","));
  }
  {
    std::string combined = test_vec[0];
    for (std::size_t i = 1; i < test_vec.size(); ++i)
    {
      combined += "--?--" + test_vec[i];
    }

    EXPECT_EQ(test_vec, urcl::splitString(combined, "--?--"));
  }

  const std::string version_string1 = "5.12.0.1101319";
  std::vector<std::string> expected = { "5", "12", "0", "1101319" };
  EXPECT_EQ(expected, splitString(version_string1, "."));
}

TEST(TestHelpers, robotSeriesFromTypeAndVersion)
{
  const VersionInformation cb3_version = VersionInformation::fromString("3.15.0.0");
  const VersionInformation polyscope_5_version = VersionInformation::fromString("5.12.0.1101319");
  const VersionInformation polyscope_x_version = VersionInformation::fromString("10.0.0.0");

  // UR3/UR5/UR10: major >= 5 -> E_SERIES, otherwise CB3
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, cb3_version), RobotSeries::CB3);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, cb3_version), RobotSeries::CB3);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, cb3_version), RobotSeries::CB3);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, polyscope_5_version), RobotSeries::E_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR3, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR5, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10, polyscope_x_version), RobotSeries::E_SERIES);

  // UR16: major >= 5 -> E_SERIES, otherwise UNDEFINED
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, polyscope_5_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, polyscope_x_version), RobotSeries::E_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR16, cb3_version), RobotSeries::UNDEFINED);

  // UR15/UR18/UR20/UR30/UR8LONG: major >= 5 -> UR_SERIES, otherwise UNDEFINED
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, polyscope_x_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, polyscope_x_version), RobotSeries::UR_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, polyscope_5_version), RobotSeries::UR_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, polyscope_5_version), RobotSeries::UR_SERIES);

  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR15, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR20, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR30, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR8LONG, cb3_version), RobotSeries::UNDEFINED);

  // UNDEFINED robot type yields UNDEFINED series
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_5_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_x_version), RobotSeries::UNDEFINED);
}

TEST(TestHelpers, robotSeriesString)
{
  EXPECT_EQ(robotSeriesString(RobotSeries::CB3), "CB3");
  EXPECT_EQ(robotSeriesString(RobotSeries::E_SERIES), "E_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::UR_SERIES), "UR_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::UNDEFINED), "UNDEFINED");
}

// Thread Affinity Tests

TEST(TestHelpers, setThreadAffinity_basic)
{
#ifdef _WIN32
  pthread_t thread = pthread_self();
  DWORD_PTR mask = (1ULL << 0);
  EXPECT_TRUE(setThreadAffinity(thread, mask));
#elif __linux__
  pthread_t thread = pthread_self();
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  EXPECT_TRUE(setThreadAffinity(thread, cpuset));
#endif
}

TEST(TestHelpers, setThreadAffinity_mult)
{
#ifdef _WIN32
  pthread_t thread = pthread_self();
  DWORD_PTR mask = (1ULL << 0) | (1ULL << 1);
  EXPECT_TRUE(setThreadAffinity(thread, mask));
#elif __linux__
  pthread_t thread = pthread_self();
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  CPU_SET(1, &cpuset);
  EXPECT_TRUE(setThreadAffinity(thread, cpuset));
#endif
}

TEST(TestHelpers, setThreadAffinity_invalid)
{
#ifdef _WIN32
  pthread_t thread = pthread_self();
  uint64_t invalid_mask = 1ULL << 63;
  EXPECT_FALSE(setThreadAffinity(thread, invalid_mask));
#elif __linux__
  pthread_t thread = pthread_self();
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(63, &cpuset);
  EXPECT_FALSE(setThreadAffinity(thread, cpuset));
#endif
}

TEST(TestHelpers, setThreadAffinity_invalidHandle)
{
#ifdef _WIN32
  pthread_t thread = nullptr;
  EXPECT_FALSE(setThreadAffinity(thread, (1ULL << 0)));
#elif __linux__
  pthread_t thread{};
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  EXPECT_DEATH(setThreadAffinity(thread, cpuset), "");
#endif
}

TEST(TestHelpers, setThreadAffinity_empty)
{
#ifdef _WIN32
  pthread_t thread = pthread_self();
  EXPECT_FALSE(setThreadAffinity(thread, 0));
#elif __linux__
  pthread_t thread = pthread_self();
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  EXPECT_FALSE(setThreadAffinity(thread, cpuset));
#endif
}

// Thread Priority Tests

#ifdef _WIN32
TEST(TestHelpers, setThreadPriority_allValidPriorities)
{
  pthread_t thread = pthread_self();

  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_LOWEST));
  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_BELOW_NORMAL));
  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_NORMAL));
  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_ABOVE_NORMAL));
  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_HIGHEST));
  EXPECT_TRUE(setThreadPriority(thread, THREAD_PRIORITY_TIME_CRITICAL));
}

TEST(TestHelpers, setThreadPriority_invalid)
{
  pthread_t thread = pthread_self();
  EXPECT_FALSE(setThreadPriority(thread, 999999));
}

TEST(TestHelpers, setThreadPriority_invalidHandle)
{
  pthread_t thread = nullptr;
  EXPECT_FALSE(setThreadPriority(thread, THREAD_PRIORITY_HIGHEST));
}

// Process Priority tests

TEST(TestHelpers, setProcessPriority_allValidPriorities)
{
  pprocess_t process = pprocess_self();

  EXPECT_TRUE(setProcessPriority(process, IDLE_PRIORITY_CLASS));
  EXPECT_TRUE(setProcessPriority(process, BELOW_NORMAL_PRIORITY_CLASS));
  EXPECT_TRUE(setProcessPriority(process, NORMAL_PRIORITY_CLASS));
  EXPECT_TRUE(setProcessPriority(process, ABOVE_NORMAL_PRIORITY_CLASS));
  EXPECT_TRUE(setProcessPriority(process, HIGH_PRIORITY_CLASS));
}

TEST(TestHelpers, setProcessPriority_invalid)
{
  pprocess_t process = pprocess_self();
  EXPECT_FALSE(setProcessPriority(process, 999999));
}

TEST(TestHelpers, setProcessPriority_invalidHandle)
{
  pprocess_t process = nullptr;
  EXPECT_FALSE(setProcessPriority(process, NORMAL_PRIORITY_CLASS));
}

// Process Affinity Tests

TEST(TestHelpers, setProcessAffinity_basic)
{
  pprocess_t process = pprocess_self();
  DWORD_PTR mask = (1ULL << 0);
  EXPECT_TRUE(setProcessAffinity(process, mask));
}

TEST(TestHelpers, setProcessAffinity_multi)
{
  pprocess_t process = pprocess_self();
  DWORD_PTR mask = (1ULL << 0) | (1ULL << 1);
  EXPECT_TRUE(setProcessAffinity(process, mask));
}

TEST(TestHelpers, setProcessAffinity_invalidHandle)
{
  pprocess_t process = nullptr;
  EXPECT_FALSE(setProcessAffinity(process, (1ULL << 0)));
}

TEST(TestHelpers, setProcessAffinity_zeroMask)
{
  pprocess_t process = pprocess_self();
  EXPECT_FALSE(setProcessAffinity(process, 0));
}

TEST(TestHelpers, setProcessAffinity_invalidMask)
{
  pprocess_t process = pprocess_self();
  DWORD_PTR mask = 0xFFFFFFFFFFFFFFFFULL;
  EXPECT_FALSE(setProcessAffinity(process, mask));
}
#endif
