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

#include <chrono>
#include <fstream>
#include <filesystem>
#include <string>
#include <iterator>
#include <system_error>
#include <vector>

#ifndef _WIN32
#  include <csignal>
#  include <cstdlib>
#  include <sys/resource.h>
#  include <unistd.h>
#endif

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

TEST(TestHelpers, splitStringView)
{
  using Parts = std::vector<std::string_view>;
  const auto split = [](const std::string_view input, const std::string_view delimiter) {
    Parts parts;
    splitStringView(input, delimiter, parts);
    return parts;
  };

  const std::vector<std::pair<std::string, std::string>> same_as_split_string = {
    { "this,is,very,simple", "," },
    { "this--?--is--?--very--?--simple", "--?--" },
    { "5.12.0.1101319", "." },
  };
  for (const auto& [input, delimiter] : same_as_split_string)
  {
    const std::vector<std::string> copies = splitString(input, delimiter);
    EXPECT_EQ(Parts(copies.begin(), copies.end()), split(input, delimiter)) << input;
  }

  EXPECT_EQ(split("", ","), Parts{ "" });
  EXPECT_EQ(split("a,", ","), (Parts{ "a", "" }));
  EXPECT_EQ(split(",a", ","), (Parts{ "", "a" }));
  EXPECT_EQ(split("a,,b", ","), (Parts{ "a", "", "b" }));
  EXPECT_EQ(split(",", ","), (Parts{ "", "" }));

  EXPECT_EQ(split("a,b", ""), Parts{ "a,b" });

  const std::string input = "UINT64,UINT32,DOUBLE";
  const Parts parts = split(input, ",");
  ASSERT_EQ(parts.size(), 3u);
  for (const std::string_view part : parts)
  {
    EXPECT_GE(part.data(), input.data());
    EXPECT_LE(part.data() + part.size(), input.data() + input.size());
  }
  EXPECT_EQ(parts[1].data(), input.data() + 7);
}

TEST(TestHelpers, splitStringView_reuses_the_parts_vector)
{
  std::vector<std::string_view> parts{ "stale", "stale", "stale", "stale" };
  const std::string_view* storage = parts.data();
  splitStringView("a,b", ",", parts);
  EXPECT_EQ(parts, (std::vector<std::string_view>{ "a", "b" }));
  EXPECT_EQ(parts.data(), storage);
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

  // g-Series: version >= 10.15.0 -> G_SERIES, otherwise UNDEFINED
  const VersionInformation before_g_series = VersionInformation::fromString("10.14.99.0");
  const VersionInformation first_g_series = VersionInformation::fromString("10.15.0.0");
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10G_1750, before_g_series), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR17G_1300, before_g_series), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18G_950, before_g_series), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR10G_1750, first_g_series), RobotSeries::G_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR17G_1300, first_g_series), RobotSeries::G_SERIES);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UR18G_950, first_g_series), RobotSeries::G_SERIES);

  // UNDEFINED robot type yields UNDEFINED series
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_5_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, cb3_version), RobotSeries::UNDEFINED);
  EXPECT_EQ(robotSeriesFromTypeAndVersion(RobotType::UNDEFINED, polyscope_x_version), RobotSeries::UNDEFINED);
}

TEST(TestHelpers, robotSeriesFromTypeAndVersion_of_an_unknown_type_is_undefined)
{
  EXPECT_EQ(robotSeriesFromTypeAndVersion(static_cast<RobotType>(99), VersionInformation::fromString("5.12.0.0")),
            RobotSeries::UNDEFINED);
}

TEST(TestHelpers, robotSeriesString)
{
  EXPECT_EQ(robotSeriesString(RobotSeries::CB3), "CB3");
  EXPECT_EQ(robotSeriesString(RobotSeries::E_SERIES), "E_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::UR_SERIES), "UR_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::G_SERIES), "G_SERIES");
  EXPECT_EQ(robotSeriesString(RobotSeries::UNDEFINED), "UNDEFINED");
}

TEST(TestHelpers, robotTypeFromString)
{
  EXPECT_EQ(robotTypeFromString("ur10g-1750"), RobotType::UR10G_1750);
  EXPECT_EQ(robotTypeFromString("ur17g-1300"), RobotType::UR17G_1300);
  EXPECT_EQ(robotTypeFromString("ur18g-950"), RobotType::UR18G_950);
  EXPECT_THROW(robotTypeFromString("ur10g"), std::invalid_argument);
  EXPECT_THROW(robotTypeFromString("ur17g"), std::invalid_argument);
  EXPECT_THROW(robotTypeFromString("ur18g"), std::invalid_argument);
  EXPECT_THROW(robotTypeFromString("ur10g_1750"), std::invalid_argument);
}

TEST(TestHelpers, stringFromMotionTarget)
{
  const auto q = Q{ 0.0, 0.1, 0.2, 0.3, 0.4, 0.5 };
  const MotionTarget joint_target = q;
  EXPECT_EQ(stringFromMotionTarget(joint_target), "Q([0, 0.1, 0.2, 0.3, 0.4, 0.5])");

  const MotionTarget pose_target = Pose{ 1.0, 2.0, 3.0, 0.1, 0.2, 0.3 };
  EXPECT_EQ(stringFromMotionTarget(pose_target), "Pose(x = 1, y = 2, z = 3, rx = 0.1, ry = 0.2, rz = 0.3)");
  const MotionTarget pose_with_q_near = Pose{ 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, q };
  EXPECT_EQ(stringFromMotionTarget(pose_with_q_near), "Pose(x = 1, y = 2, z = 3, rx = 0.1, ry = 0.2, rz = 0.3)\nWith "
                                                      "Q_near([0, 0.1, 0.2, 0.3, 0.4, 0.5])");
}

namespace
{
std::string readBinaryFile(const std::filesystem::path& path)
{
  std::ifstream in(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(in), {});
}

std::vector<std::string> filenamesIn(const std::filesystem::path& dir)
{
  std::vector<std::string> names;
  for (const auto& entry : std::filesystem::directory_iterator(dir))
  {
    names.push_back(entry.path().filename().string());
  }
  return names;
}

class UniqueTempDir
{
public:
  UniqueTempDir()
  {
#ifndef _WIN32
    std::string tmpl = (std::filesystem::temp_directory_path() / "urcl_wfa_XXXXXX").string();
    if (mkdtemp(tmpl.data()) == nullptr)
    {
      return;
    }
    path_ = tmpl;
#else
    path_ = std::filesystem::temp_directory_path() /
            ("urcl_wfa_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
#endif
  }

  ~UniqueTempDir()
  {
    std::error_code ec;
    std::filesystem::remove_all(path_, ec);
  }

  bool valid() const
  {
    return !path_.empty();
  }

  const std::filesystem::path& path() const
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

class ScopedCurrentPath
{
public:
  explicit ScopedCurrentPath(const std::filesystem::path& path) : previous_(std::filesystem::current_path())
  {
    std::filesystem::current_path(path);
  }

  ~ScopedCurrentPath()
  {
    std::error_code ec;
    std::filesystem::current_path(previous_, ec);
  }

private:
  std::filesystem::path previous_;
};

#ifndef _WIN32
class IgnoreSigxfsz
{
public:
  IgnoreSigxfsz()
  {
    struct sigaction action = {};
    action.sa_handler = SIG_IGN;
    installed_ = sigaction(SIGXFSZ, &action, &previous_) == 0;
  }

  ~IgnoreSigxfsz()
  {
    if (installed_)
    {
      sigaction(SIGXFSZ, &previous_, nullptr);
    }
  }

  bool installed() const
  {
    return installed_;
  }

private:
  struct sigaction previous_{};
  bool installed_{ false };
};

class ScopedFileSizeLimit
{
public:
  explicit ScopedFileSizeLimit(rlim_t bytes)
  {
    if (getrlimit(RLIMIT_FSIZE, &previous_) != 0)
    {
      return;
    }
    rlimit tiny = previous_;
    tiny.rlim_cur = bytes;
    active_ = setrlimit(RLIMIT_FSIZE, &tiny) == 0;
  }

  ~ScopedFileSizeLimit()
  {
    if (active_)
    {
      setrlimit(RLIMIT_FSIZE, &previous_);
    }
  }

  bool active() const
  {
    return active_;
  }

private:
  rlimit previous_{};
  bool active_{ false };
};
#endif
}  // namespace

TEST(TestHelpers, writeFileAtomically)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path dest = dir.path() / "file.bin";

  std::string error;
  ASSERT_TRUE(writeFileAtomically(dest.string(), "first", error)) << error;
  EXPECT_EQ(readBinaryFile(dest), "first");
  EXPECT_EQ(filenamesIn(dir.path()), std::vector<std::string>{ "file.bin" });

  ASSERT_TRUE(writeFileAtomically(dest.string(), "replacement", error)) << error;
  EXPECT_EQ(readBinaryFile(dest), "replacement");
  EXPECT_EQ(filenamesIn(dir.path()), std::vector<std::string>{ "file.bin" });
}

TEST(TestHelpers, writeFileAtomicallyEmptyContents)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path dest = dir.path() / "empty.bin";

  std::string error;
  ASSERT_TRUE(writeFileAtomically(dest.string(), "", error)) << error;
  EXPECT_TRUE(std::filesystem::exists(dest));
  EXPECT_EQ(std::filesystem::file_size(dest), 0u);
}

TEST(TestHelpers, writeFileAtomicallyBinaryContents)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path dest = dir.path() / "binary.bin";
  const std::string payload({ 'a', '\0', 'b', static_cast<char>(0xff), 'c' });

  std::string error;
  ASSERT_TRUE(writeFileAtomically(dest.string(), payload, error)) << error;
  EXPECT_EQ(readBinaryFile(dest), payload);
}

TEST(TestHelpers, writeFileAtomicallyRelativePathUsesCurrentDirectory)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  ScopedCurrentPath cwd(dir.path());

  std::string error;
  ASSERT_TRUE(writeFileAtomically("relative.bin", "payload", error)) << error;
  EXPECT_EQ(readBinaryFile(dir.path() / "relative.bin"), "payload");
}

TEST(TestHelpers, writeFileAtomicallyMissingParent)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::string dest = (dir.path() / "missing_parent" / "file.bin").string();

  std::string error;
  EXPECT_FALSE(writeFileAtomically(dest, "payload", error));
  EXPECT_NE(error.find("Failed to create temporary file"), std::string::npos);
  EXPECT_TRUE(filenamesIn(dir.path()).empty());
}

TEST(TestHelpers, writeFileAtomicallyParentIsFile)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path parent = dir.path() / "not_a_directory";
  {
    std::ofstream out(parent);
    out << "blocker";
  }

  std::string error;
  EXPECT_FALSE(writeFileAtomically((parent / "file.bin").string(), "payload", error));
  EXPECT_NE(error.find("Failed to create temporary file"), std::string::npos);
  EXPECT_EQ(readBinaryFile(parent), "blocker");
}

TEST(TestHelpers, writeFileAtomicallyDestinationIsDirectory)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path dest = dir.path() / "existing_dir";
  std::filesystem::create_directories(dest);

  std::string error;
  EXPECT_FALSE(writeFileAtomically(dest.string(), "payload", error));
  EXPECT_NE(error.find("Failed to rename temporary file"), std::string::npos);
  EXPECT_TRUE(std::filesystem::is_directory(dest));
  EXPECT_EQ(filenamesIn(dir.path()), std::vector<std::string>{ "existing_dir" });
}

TEST(TestHelpers, writeFileAtomicallyReplacesSymlinkWithoutFollowingIt)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path secret = dir.path() / "secret.bin";
  const std::filesystem::path dest = dir.path() / "dest.bin";
  {
    std::ofstream out(secret, std::ios::binary);
    out << "secret";
  }
  std::filesystem::create_symlink(secret, dest);

  std::string error;
  ASSERT_TRUE(writeFileAtomically(dest.string(), "safe", error)) << error;
  EXPECT_EQ(readBinaryFile(dest), "safe");
  EXPECT_EQ(readBinaryFile(secret), "secret");
  EXPECT_FALSE(std::filesystem::is_symlink(dest));
}

#ifndef _WIN32
TEST(TestHelpers, writeFileAtomicallyWriteError)
{
  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  const std::filesystem::path dest = dir.path() / "file.bin";

  IgnoreSigxfsz ignore_sigxfsz;
  ASSERT_TRUE(ignore_sigxfsz.installed());
  ScopedFileSizeLimit limit(1);
  ASSERT_TRUE(limit.active());

  std::string error;
  const bool ok = writeFileAtomically(dest.string(), std::string(4096, 'x'), error);
  if (ok)
  {
    GTEST_SKIP() << "Kernel did not enforce RLIMIT_FSIZE; cannot exercise the write-error path.";
  }

  EXPECT_NE(error.find("Write error"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(dest));
  EXPECT_TRUE(filenamesIn(dir.path()).empty());
}

TEST(TestHelpers, writeFileAtomicallyUnwritableDirectory)
{
  if (geteuid() == 0)
  {
    GTEST_SKIP() << "Root can write to a directory without owner-write permission.";
  }

  UniqueTempDir dir;
  ASSERT_TRUE(dir.valid());
  std::filesystem::permissions(dir.path(), std::filesystem::perms::owner_read | std::filesystem::perms::owner_exec);

  std::string error;
  const bool ok = writeFileAtomically((dir.path() / "file.bin").string(), "payload", error);
  std::filesystem::permissions(dir.path(), std::filesystem::perms::owner_all);

  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("Failed to create temporary file"), std::string::npos);
  EXPECT_TRUE(filenamesIn(dir.path()).empty());
}
#endif
