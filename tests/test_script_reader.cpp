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
#include "ur_client_library/ur/version_information.h"
#include "ur_client_library/control/reverse_interface.h"
#include "ur_client_library/control/trajectory_point_interface.h"

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
  EXPECT_EQ(content, simple_script_.str());
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
  EXPECT_EQ(processed_script, "movej([1,2,3,4,5,6])");

  std::remove(existing_script_file);
  std::remove(existing_include_file);
}

TEST_F(ScriptReaderTest, ReplaceVariables)
{
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

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["VAR1"] = "value1";
  data["VAR2"] = 42;
  data["VAR3"] = 6.28;
  data["VAR4"] = true;
  data["VAR5"] = false;
  std::string script = reader.readScriptFile(existing_script_file, data);

  // By default std::to_string will convert double to 6 decimal places
  EXPECT_EQ(script, "movej([value1, 42, 6.280000, 0, 0, 0])\nlocal is_true = True\nlocal is_false = False\nThis is "
                    "just a line without any replacement");
  std::remove(existing_script_file);
}

TEST_F(ScriptReaderTest, VariableNotInDictThrowsError)
{
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

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["VAR1"] = "value1";

  EXPECT_THROW(reader.readScriptFile(existing_script_file, data), urcl::UnknownVariable);
  std::remove(existing_script_file);
}

TEST_F(ScriptReaderTest, ReplaceConditionals)
{
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

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["is_logged_in"] = true;
  data["is_guest"] = false;
  data["username"] = "test_user";

  std::string script = reader.readScriptFile(existing_script_file, data);

  EXPECT_EQ(script, "Welcome back, test_user!");

  data["is_logged_in"] = false;
  data["is_guest"] = true;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Welcome, test_user!");

  data["username"] = "";
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Welcome, guest!");

  data["is_guest"] = false;
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "Please log in.");
  std::remove(existing_script_file);
}

TEST_F(ScriptReaderTest, TestNestedConditionals)
{
  char existing_script_file[] = "main_script.XXXXXX";
  std::ignore = mkstemp(existing_script_file);
  std::ofstream ofs(existing_script_file);
  if (ofs.bad())
  {
    std::cout << "Failed to create temporary files" << std::endl;
    GTEST_FAIL();
  }
  ofs <<
      R"({% if PI < THE_ANSWER_TO_EVERYTHING %}
  {% if TAU < PI %}
It's a strange universe you live in...
  {%elif pie_tastes == "great" %}
What's better than a pie? 2 pies!
  {% else %}
You don't like pie?
  {% endif %}
{% else %}
How can something be greater than the answer to everything?
{% endif %}
)";
  ofs.close();

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["PI"] = 3.14;
  data["TAU"] = 6.28;
  data["THE_ANSWER_TO_EVERYTHING"] = 42;

  data["pie_tastes"] = "great";
  std::string script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "What's better than a pie? 2 pies!");

  data["pie_tastes"] = "aweful";
  script = reader.readScriptFile(existing_script_file, data);
  EXPECT_EQ(script, "You don't like pie?");

  std::remove(existing_script_file);
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
  EXPECT_TRUE(reader.evaluateExpression("  A", data));
  EXPECT_FALSE(reader.evaluateExpression("B", data));

  // Equality
  EXPECT_TRUE(reader.evaluateExpression("X == 5", data));
  EXPECT_FALSE(reader.evaluateExpression("X == 6", data));
  EXPECT_TRUE(reader.evaluateExpression("S == \"hello\"", data));
  EXPECT_TRUE(reader.evaluateExpression("S == 'hello'", data));
  EXPECT_FALSE(reader.evaluateExpression("S == \"world\"", data));
  EXPECT_FALSE(reader.evaluateExpression("S == 'world'", data));
  EXPECT_FALSE(reader.evaluateExpression("S == T", data));
  EXPECT_TRUE(reader.evaluateExpression("S == S", data));
  EXPECT_TRUE(reader.evaluateExpression("A == true", data));
  EXPECT_TRUE(reader.evaluateExpression("A == True", data));
  EXPECT_TRUE(reader.evaluateExpression("A == TRUE", data));
  EXPECT_TRUE(reader.evaluateExpression("A == 1", data));
  EXPECT_TRUE(reader.evaluateExpression("A == on", data));
  EXPECT_TRUE(reader.evaluateExpression("A == On", data));
  EXPECT_TRUE(reader.evaluateExpression("A == ON", data));
  EXPECT_FALSE(reader.evaluateExpression("B == true", data));
  EXPECT_FALSE(reader.evaluateExpression("B == True", data));
  EXPECT_FALSE(reader.evaluateExpression("B == TRUE", data));
  EXPECT_FALSE(reader.evaluateExpression("B == 1", data));
  EXPECT_FALSE(reader.evaluateExpression("B == on", data));
  EXPECT_FALSE(reader.evaluateExpression("B == On", data));
  EXPECT_FALSE(reader.evaluateExpression("B == ON", data));
  EXPECT_FALSE(reader.evaluateExpression("A == B", data));

  // Inequality
  EXPECT_TRUE(reader.evaluateExpression("X != 6", data));
  EXPECT_FALSE(reader.evaluateExpression("X != 5", data));
  EXPECT_TRUE(reader.evaluateExpression("A != B", data));

  // Greater/Less
  EXPECT_TRUE(reader.evaluateExpression("Y > X", data));
  EXPECT_FALSE(reader.evaluateExpression("X > Y", data));
  EXPECT_TRUE(reader.evaluateExpression("X < Y", data));
  EXPECT_FALSE(reader.evaluateExpression("Y < X", data));
  EXPECT_TRUE(reader.evaluateExpression("PI > 3", data));
  EXPECT_FALSE(reader.evaluateExpression("PI < 3", data));
  EXPECT_TRUE(reader.evaluateExpression("PI >= 3.14159", data));
  EXPECT_FALSE(reader.evaluateExpression("PI < 3.14159", data));
  EXPECT_TRUE(reader.evaluateExpression("PI < X", data));

  // String not empty
  EXPECT_TRUE(reader.evaluateExpression("S != ''", data));
  EXPECT_TRUE(reader.evaluateExpression("S != \"\"", data));
  EXPECT_FALSE(reader.evaluateExpression("S == \"\"", data));

  // Provoke errors
  // Non-existing operator
  EXPECT_THROW(reader.evaluateExpression("X ~= 5", data), std::runtime_error);
  EXPECT_THROW(reader.evaluateExpression("This is not an expression at all", data), std::runtime_error);
  EXPECT_TRUE(reader.evaluateExpression("S != \"This is not an expression at all\"", data));
  // Non-existing variable
  EXPECT_THROW(reader.evaluateExpression("non_existing == 5", data), urcl::UnknownVariable);
  EXPECT_THROW(reader.evaluateExpression("A == non_existing", data), urcl::UnknownVariable);
  EXPECT_THROW(reader.evaluateExpression("IDONTEXIST", data), urcl::UnknownVariable);
  // <, >, <=, >= is only available for numeric types
  EXPECT_THROW(reader.evaluateExpression("A < 5", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("S < T", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("X < True", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("A > 5", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("S > T", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("X > True", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("A <= 5", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("S <= T", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("X <= True", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("A >= 5", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("S >= T", data), std::invalid_argument);
  EXPECT_THROW(reader.evaluateExpression("X >= True", data), std::invalid_argument);
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
  data["version1"] = urcl::VersionInformation::fromString("10.7.0");
  data["version2"] = urcl::VersionInformation::fromString("5.22.1");

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
  EXPECT_TRUE(data["version1"] == urcl::VersionInformation::fromString("10.7.0"));
  EXPECT_FALSE(data["version2"] == urcl::VersionInformation::fromString("10.7.0"));
  EXPECT_FALSE(data["version2"] == data["version1"]);

  // Inequality
  EXPECT_TRUE(data["int1"] != 6);
  EXPECT_FALSE(data["int1"] != 5);
  EXPECT_TRUE(data["str1"] != std::string("bar"));
  EXPECT_FALSE(data["str1"] != std::string("foo"));
  EXPECT_TRUE(data["version1"] != urcl::VersionInformation::fromString("1.2.3"));

  // Less, Greater, etc. (numeric only)
  EXPECT_TRUE(data["int1"] < data["int2"]);
  EXPECT_TRUE(data["int2"] > data["int1"]);
  EXPECT_TRUE(data["int1"] <= data["int2"]);
  EXPECT_TRUE(data["int2"] >= data["int1"]);
  EXPECT_TRUE(data["double1"] <= data["double2"]);
  EXPECT_TRUE(data["double1"] >= data["double2"]);
  EXPECT_FALSE(data["int1"] < data["double1"]);
  EXPECT_TRUE(data["version1"] > data["version2"]);
  EXPECT_TRUE(data["version1"] >= data["version1"]);
  EXPECT_TRUE(data["version1"] <= data["version1"]);
  EXPECT_TRUE(data["version2"] < data["version1"]);

  // Invalid comparisons (should throw)
  EXPECT_THROW((void)(data["str1"] < data["str2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["bool1"] < data["bool2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["str1"] > data["str2"]), std::invalid_argument);
  EXPECT_THROW((void)(data["bool1"] > data["bool2"]), std::invalid_argument);
  EXPECT_THROW(data["str1"] == data["bool1"], std::invalid_argument);
  EXPECT_THROW(data["double1"] == data["str1"], std::invalid_argument);
}

TEST_F(ScriptReaderTest, Example)
{
  std::string existing_script_file = "resources/example_urscript_main.urscript";

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("5.9");
  data["feature_name"] = "torque control";

  std::string processed_script = reader.readScriptFile(existing_script_file, data);
  std::string expected_script = "  popup(\"The cool new feature is not supported on Software version 5.23.0\")";
  EXPECT_EQ(processed_script, expected_script);

  data["SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("5.23.0");
  processed_script = reader.readScriptFile(existing_script_file, data);
  expected_script = "  textmsg(\"torque control is a very cool feature!\")";
  EXPECT_EQ(processed_script, expected_script);
}

TEST_F(ScriptReaderTest, TestParsingExternalControl)
{
  std::string existing_script_file = "../resources/external_control.urscript";

  ScriptReader reader;
  ScriptReader::DataDict data;
  data["BEGIN_REPLACE"] = "";
  data["JOINT_STATE_REPLACE"] = std::to_string(urcl::control::ReverseInterface::MULT_JOINTSTATE);
  data["TIME_REPLACE"] = std::to_string(urcl::control::TrajectoryPointInterface::MULT_TIME);
  data["SERVO_J_REPLACE"] = "lookahead_time=0.03, gain=2000";
  data["SERVER_IP_REPLACE"] = "1.2.3.4";
  data["SERVER_PORT_REPLACE"] = "50001";
  data["TRAJECTORY_SERVER_PORT_REPLACE"] = "50003";
  data["SCRIPT_COMMAND_SERVER_PORT_REPLACE"] = "50004";

  data["ROBOT_SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("3.12.1");
  std::string processed_script = reader.readScriptFile(existing_script_file, data);
  std::string expected_pattern = "enable_external_ft_sensor(enabled, sensor_mass, sensor_offset, sensor_cog)";
  EXPECT_NE(processed_script.find(expected_pattern), std::string::npos);

  data["ROBOT_SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("5.8.0");
  processed_script = reader.readScriptFile(existing_script_file, data);
  expected_pattern = "enable_external_ft_sensor(enabled, sensor_mass, sensor_offset, sensor_cog)";
  EXPECT_NE(processed_script.find(expected_pattern), std::string::npos);

  data["ROBOT_SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("3.14.1");
  processed_script = reader.readScriptFile(existing_script_file, data);
  expected_pattern = "ft_rtde_input_enable(enabled, sensor_mass, sensor_offset, sensor_cog)";
  EXPECT_NE(processed_script.find(expected_pattern), std::string::npos);

  data["ROBOT_SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("5.9.1");
  processed_script = reader.readScriptFile(existing_script_file, data);
  expected_pattern = "ft_rtde_input_enable(enabled, sensor_mass, sensor_offset, sensor_cog)";
  EXPECT_NE(processed_script.find(expected_pattern), std::string::npos);

  data["ROBOT_SOFTWARE_VERSION"] = urcl::VersionInformation::fromString("10.7.0");
  processed_script = reader.readScriptFile(existing_script_file, data);
  expected_pattern = "ft_rtde_input_enable(enabled, sensor_mass, sensor_offset, sensor_cog)";
  EXPECT_NE(processed_script.find(expected_pattern), std::string::npos);
}
