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
#include "ur_client_library/control/script_reader.h"

#include <fstream>

using namespace urcl::control;

class ScriptReaderTest : public ::testing::Test
{
protected:
  std::string valid_script_path_;
  std::string invalid_script_path_;
  std::string empty_script_path_;

  std::stringstream simple_script_;

  ScriptReader::RobotInfo robot_info_;

  void SetUp() override
  {
    invalid_script_path_ = "test_resources/non_existent_script.urscript";
    empty_script_path_ = "resources/empty.txt";
    char existing_script_file[] = "urscript.XXXXXX";
#ifdef _WIN32
#  define mkstemp _mktemp_s
#endif
    std::ignore = mkstemp(existing_script_file);
    std::ofstream ofs(existing_script_file);
    if (ofs.bad())
    {
      std::cout << "Failed to create temporary files" << std::endl;
      GTEST_FAIL();
    }
    ofs.close();

    valid_script_path_ = existing_script_file;

    simple_script_ << "movej([0,0,0,0,0,0])";

    // Create test resources
    std::ofstream valid_script(valid_script_path_);
    valid_script << simple_script_.str();
    valid_script.close();

    std::ofstream empty_script(empty_script_path_);
    empty_script.close();

    robot_info_.robot_type = urcl::RobotType::UR3;
  }

  void TearDown() override
  {
    std::remove(valid_script_path_.c_str());
    std::remove(empty_script_path_.c_str());
  }
};

TEST_F(ScriptReaderTest, ReadValidScript)
{
  ScriptReader reader(robot_info_);
  std::string content = reader.readScriptFile(valid_script_path_);
  EXPECT_EQ(content, "movej([0,0,0,0,0,0])");
}

TEST_F(ScriptReaderTest, ReadEmptyScript)
{
  ScriptReader reader(robot_info_);
  std::string content = reader.readScriptFile(empty_script_path_);
  EXPECT_EQ(content, "");
}

TEST_F(ScriptReaderTest, ReadNonExistentScript)
{
  ScriptReader reader(robot_info_);
  EXPECT_THROW(reader.readScriptFile(invalid_script_path_), std::runtime_error);
}