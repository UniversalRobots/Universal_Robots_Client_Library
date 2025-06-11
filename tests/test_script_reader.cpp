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
    urcl::setLogLevel(urcl::LogLevel::INFO);
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
  {% if username != "" %}
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

  data["username"] = "";
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Welcome, guest!\n");

  data["is_guest"] = false;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Please log in.\n");
}

TEST_F(ScriptReaderTest, CheckCondition)
{
  ScriptReader reader;
  ScriptReader::DataDict data;
  data["A"] = true;
  data["B"] = false;
  data["X"] = 5;
  data["Y"] = 10;
  data["PI"] = 3.14159;
  data["S"] = "hello";
  data["T"] = "world";

  // True/False
  EXPECT_TRUE(reader.checkCondition("  A", data));
  EXPECT_FALSE(reader.checkCondition("B", data));

  // Equality
  EXPECT_TRUE(reader.checkCondition("X == 5", data));
  EXPECT_FALSE(reader.checkCondition("X == 6", data));
  EXPECT_TRUE(reader.checkCondition("S == \"hello\"", data));
  EXPECT_TRUE(reader.checkCondition("S == 'hello'", data));
  EXPECT_FALSE(reader.checkCondition("S == \"world\"", data));
  EXPECT_FALSE(reader.checkCondition("S == 'world'", data));
  EXPECT_FALSE(reader.checkCondition("S == T", data));
  EXPECT_TRUE(reader.checkCondition("S == S", data));
  EXPECT_TRUE(reader.checkCondition("A == true", data));
  EXPECT_TRUE(reader.checkCondition("A == True", data));
  EXPECT_TRUE(reader.checkCondition("A == TRUE", data));
  EXPECT_TRUE(reader.checkCondition("A == 1", data));
  EXPECT_TRUE(reader.checkCondition("A == on", data));
  EXPECT_TRUE(reader.checkCondition("A == On", data));
  EXPECT_TRUE(reader.checkCondition("A == ON", data));
  EXPECT_FALSE(reader.checkCondition("B == true", data));
  EXPECT_FALSE(reader.checkCondition("B == True", data));
  EXPECT_FALSE(reader.checkCondition("B == TRUE", data));
  EXPECT_FALSE(reader.checkCondition("B == 1", data));
  EXPECT_FALSE(reader.checkCondition("B == on", data));
  EXPECT_FALSE(reader.checkCondition("B == On", data));
  EXPECT_FALSE(reader.checkCondition("B == ON", data));
  EXPECT_FALSE(reader.checkCondition("A == B", data));

  // Inequality
  EXPECT_TRUE(reader.checkCondition("X != 6", data));
  EXPECT_FALSE(reader.checkCondition("X != 5", data));
  EXPECT_TRUE(reader.checkCondition("A != B", data));

  // Greater/Less
  EXPECT_TRUE(reader.checkCondition("Y > X", data));
  EXPECT_FALSE(reader.checkCondition("X > Y", data));
  EXPECT_TRUE(reader.checkCondition("X < Y", data));
  EXPECT_FALSE(reader.checkCondition("Y < X", data));
  EXPECT_TRUE(reader.checkCondition("PI > 3", data));
  EXPECT_FALSE(reader.checkCondition("PI < 3", data));
  EXPECT_TRUE(reader.checkCondition("PI >= 3.14159", data));
  EXPECT_FALSE(reader.checkCondition("PI < 3.14159", data));
  EXPECT_TRUE(reader.checkCondition("PI < X", data));

  // String not empty
  EXPECT_TRUE(reader.checkCondition("S != ''", data));
  EXPECT_TRUE(reader.checkCondition("S != \"\"", data));
  EXPECT_FALSE(reader.checkCondition("S == \"\"", data));

  // Provoke errors
  // Non-existing operator
  EXPECT_THROW(reader.checkCondition("X ~= 5", data), std::runtime_error);
  EXPECT_THROW(reader.checkCondition("This is not an expression at all", data), std::runtime_error);
  EXPECT_TRUE(reader.checkCondition("S != \"This is not an expression at all\"", data));
  // Non-existing variable
  EXPECT_THROW(reader.checkCondition("non_existing == 5", data), urcl::UnknownVariable);
  EXPECT_THROW(reader.checkCondition("A == non_existing", data), urcl::UnknownVariable);
  EXPECT_THROW(reader.checkCondition("IDONTEXIST", data), urcl::UnknownVariable);
  // <, >, <=, >= is only available for numeric types
  EXPECT_THROW(reader.checkCondition("A < 5", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("S < T", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("X < True", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("A > 5", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("S > T", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("X > True", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("A <= 5", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("S <= T", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("X <= True", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("A >= 5", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("S >= T", data), std::invalid_argument);
  EXPECT_THROW(reader.checkCondition("X >= True", data), std::invalid_argument);
}

TEST_F(ScriptReaderTest, ParseBoolean)
{
  EXPECT_TRUE(ScriptReader::parseBoolean("true"));
  EXPECT_TRUE(ScriptReader::parseBoolean("True"));
  EXPECT_TRUE(ScriptReader::parseBoolean("TRUE"));
  EXPECT_TRUE(ScriptReader::parseBoolean("on"));
  EXPECT_TRUE(ScriptReader::parseBoolean("On"));
  EXPECT_TRUE(ScriptReader::parseBoolean("ON"));
  EXPECT_TRUE(ScriptReader::parseBoolean("1"));
  EXPECT_FALSE(ScriptReader::parseBoolean("false"));
  EXPECT_FALSE(ScriptReader::parseBoolean("False"));
  EXPECT_FALSE(ScriptReader::parseBoolean("FALSE"));
  EXPECT_FALSE(ScriptReader::parseBoolean("off"));
  EXPECT_FALSE(ScriptReader::parseBoolean("Off"));
  EXPECT_FALSE(ScriptReader::parseBoolean("OFF"));
  EXPECT_FALSE(ScriptReader::parseBoolean("0"));
  EXPECT_THROW(ScriptReader::parseBoolean("notabool"), urcl::UrException);
}

TEST_F(ScriptReaderTest, DataVariantOperators)
{
  ScriptReader::DataDict data;
  data["int1"] = 5;
  data["int2"] = 10;
  data["double1"] = 3.14;
  data["double2"] = 3.14;
  data["str1"] = "foo";
  data["str2"] = "bar";
  data["bool1"] = true;
  data["bool2"] = false;

  // Equality
  EXPECT_TRUE(data["int1"] == 5);
  EXPECT_FALSE(data["int1"] == 6);
  EXPECT_TRUE(data["double1"] == 3.14);
  EXPECT_FALSE(data["double1"] == data["int1"]);
  EXPECT_TRUE(data["str1"] == std::string("foo"));
  EXPECT_FALSE(data["str1"] == std::string("bar"));
  EXPECT_TRUE(data["bool1"] == true);
  EXPECT_FALSE(data["bool2"] == true);
  EXPECT_TRUE(data["bool1"] == 1);
  EXPECT_TRUE(data["bool1"] == 1.0);
  EXPECT_FALSE(data["bool1"] == 0.0);
  EXPECT_FALSE(data["bool1"] == 3.14);
  EXPECT_FALSE(data["bool1"] == 42);

  // Inequality
  EXPECT_TRUE(data["int1"] != 6);
  EXPECT_FALSE(data["int1"] != 5);
  EXPECT_TRUE(data["str1"] != std::string("bar"));
  EXPECT_FALSE(data["str1"] != std::string("foo"));

  // Less, Greater, etc. (numeric only)
  EXPECT_TRUE(data["int1"] < data["int2"]);
  EXPECT_TRUE(data["int2"] > data["int1"]);
  EXPECT_TRUE(data["int1"] <= data["int2"]);
  EXPECT_TRUE(data["int2"] >= data["int1"]);
  EXPECT_TRUE(data["double1"] <= data["double2"]);
  EXPECT_TRUE(data["double1"] >= data["double2"]);
  EXPECT_FALSE(data["int1"] < data["double1"]);

  // Invalid comparisons (should throw)
  EXPECT_THROW((void)(data["str1"] < data["str2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["bool1"] < data["bool2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["str1"] > data["str2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["bool1"] > data["bool2"]), std::invalid_argument);
  EXPECT_THROW(data["str1"] == data["bool1"], std::invalid_argument);
  EXPECT_THROW(data["double1"] == data["str1"], std::invalid_argument);
}