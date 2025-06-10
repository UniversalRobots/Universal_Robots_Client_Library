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

#include "ur_client_library/exceptions.h"

#include <gtest/gtest.h>
#include "ur_client_library/control/script_reader.h"

#include <fstream>

#ifdef _WIN32
#  define mkstemp _mktemp_s
#endif

using namespace urcl::control;

class ScriptReaderTest : public ::testing::Test
{
protected:
  std::string valid_script_path_;
  std::string invalid_script_path_;
  std::string empty_script_path_;

  std::stringstream simple_script_;

  void SetUp() override
  {
    invalid_script_path_ = "test_resources/non_existent_script.urscript";
    empty_script_path_ = "resources/empty.txt";
    char existing_script_file[] = "urscript.XXXXXX";
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
  }

  void TearDown() override
  {
    std::remove(valid_script_path_.c_str());
  }
};

TEST_F(ScriptReaderTest, ReadValidScript)
{
  ScriptReader reader;
  std::string content = reader.readScriptFile(valid_script_path_);
  EXPECT_EQ(content, simple_script_.str() + "\n");
}

TEST_F(ScriptReaderTest, ReadEmptyScript)
{
  ScriptReader reader;
  std::string content = reader.readScriptFile(empty_script_path_);
  EXPECT_EQ(content, "");
}

TEST_F(ScriptReaderTest, ReadNonExistentScript)
{
  ScriptReader reader;
  EXPECT_THROW(reader.readScriptFile(invalid_script_path_), std::runtime_error);
}

TEST_F(ScriptReaderTest, ReplaceIncludes)
{
  ScriptReader reader;

  char existing_script_file[] = "main_script.XXXXXX";
  std::ignore = mkstemp(existing_script_file);
  char existing_include_file[] = "included_script.XXXXXX";
  std::ignore = mkstemp(existing_include_file);
  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  std::string script_with_include = "{% include '" + std::string(existing_include_file) + "' %}";
  ofs << script_with_include;
  ofs.close();

  // Create a temporary included script
  std::ofstream ofs_included(existing_include_file);
  if (ofs_included.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  ofs_included << "movej([1,2,3,4,5,6])";
  ofs_included.close();

  std::string processed_script = reader.readScriptFile(existing_script_file);
  EXPECT_EQ(processed_script, "movej([1,2,3,4,5,6])\n");

  std::remove(existing_script_file);
  std::remove(existing_include_file);
}

TEST_F(ScriptReaderTest, ReplaceVariables)
{
  ScriptReader reader;
  ScriptReader::DataDict data;
  data["VAR1"] = "value1";
  data["VAR2"] = 42;
  data["VAR3"] = 6.28;
  data["VAR4"] = true;
  data["VAR5"] = false;

  char existing_script_file[] = "main_script.XXXXXX";
  std::ignore = mkstemp(existing_script_file);
  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  ofs << "movej([{{VAR1}}, {{VAR2}}, {{VAR3}}, 0, 0, 0])" << std::endl;
  ofs << "local is_true = {{VAR4}}" << std::endl;
  ofs << "local is_false = {{VAR5}}" << std::endl;
  ofs << "This is just a line without any replacement" << std::endl;
  ofs.close();

  std::string script = reader.readScriptFile(existing_script_file, data);

  // By default std::to_string will convert double to 6 decimal places
  EXPECT_EQ(script, "movej([value1, 42, 6.280000, 0, 0, 0])\nlocal is_true = True\nlocal is_false = False\nThis is "
                    "just a line without any replacement\n");
}

TEST_F(ScriptReaderTest, VariableNotInDictThrowsError)
{
  ScriptReader reader;
  ScriptReader::DataDict data;
  data["VAR1"] = "value1";

  char existing_script_file[] = "main_script.XXXXXX";
  std::ignore = mkstemp(existing_script_file);
  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  ofs << "movej([{{VAR1}}, {{VAR2}}, {{VAR3}}, 0, 0, 0])" << std::endl;
  ofs << "This is just a line without any replacement" << std::endl;
  ofs.close();

  EXPECT_THROW(reader.readScriptFile(existing_script_file, data), urcl::UnknownVariable);
}

TEST_F(ScriptReaderTest, ReplaceConditionals)
{
  ScriptReader reader;
  ScriptReader::DataDict data;
  data["is_logged_in"] = true;
  data["is_guest"] = false;
  data["has_username"] = true;
  data["username"] = "test_user";

  char existing_script_file[] = "main_script.XXXXXX";
  std::ignore = mkstemp(existing_script_file);
  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  ofs <<
      R"({% if is_logged_in %}
Welcome back, {{ username }}!
{% elif is_guest %}
  {% if has_username %}
Welcome, {{ username }}!
  {%else %}
Welcome, guest!
  {% endif %}
{% else %}
Please log in.
{% endif %}
)";
  ofs.close();

  std::string script = reader.readScriptFile(existing_script_file, data);

  EXPECT_EQ(script, "Welcome back, test_user!\n");

  data["is_logged_in"] = false;
  data["is_guest"] = true;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Welcome, test_user!\n");

  data["has_username"] = false;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Welcome, guest!\n");

  data["is_guest"] = false;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Please log in.\n");
}