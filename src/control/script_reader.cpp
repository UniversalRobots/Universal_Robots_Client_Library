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

#include <ur_client_library/exceptions.h>
#include <ur_client_library/control/script_reader.h>

#include <fstream>
#include <regex>
#include "ur_client_library/log.h"

namespace urcl
{
namespace control
{
std::string ScriptReader::readScriptFile(const std::string& filename, const DataDict& data)
{
  script_path_ = filename;
  std::string script_code = readFileContent(filename);

  replaceIncludes(script_code);
  replaceVariables(script_code, data);

  return script_code;
}

std::string ScriptReader::readFileContent(const std::string& filename)
{
  std::ifstream ifs;
  ifs.open(filename);
  std::string content;
  std::ifstream file(filename);
  if (ifs)
  {
    content = std::string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    ifs.close();
  }
  else
  {
    std::stringstream ss;
    ss << "Could not open script file '" << filename << "'. Please check if the file exists and is readable.";
    throw UrException(ss.str().c_str());
  }

  return content;
}

void ScriptReader::replaceIncludes(std::string& script)
{
  std::regex include_pattern(R"(\{\%\s*include\s*['|"]([^'"]+)['|"]\s*\%\})");

  std::smatch match;

  // Replace all include patterns in the line
  while (std::regex_search(script, match, include_pattern))
  {
    std::filesystem::path file_path(match[1]);
    std::string file_content = readFileContent(script_path_.parent_path() / file_path.string());
    script.replace(match.position(0), match.length(0), file_content);
  }
}

void ScriptReader::replaceVariables(std::string& script_code, const DataDict& data)
{
  std::regex pattern(R"(\{\{\s*([\w-]+)\s*\}\})");
  std::smatch match;
  while (std::regex_search(script_code, match, pattern))
  {
    std::string key = match[1];
    URCL_LOG_INFO("Found replacement pattern %s", match[0].str().c_str());
    if (data.find(key) == data.end())
    {
      std::stringstream ss;
      ss << "Variable '" << key << "' not found in data.";
      URCL_LOG_ERROR(ss.str().c_str());
      throw UrException(ss.str().c_str());
    }
    std::string replaced_value;

    if (std::holds_alternative<std::string>(data.at(key)))
    {
      replaced_value = std::get<std::string>(data.at(key));
    }
    else if (std::holds_alternative<double>(data.at(key)))
    {
      replaced_value = std::to_string(std::get<double>(data.at(key)));
    }
    else if (std::holds_alternative<int>(data.at(key)))
    {
      replaced_value = std::to_string(std::get<int>(data.at(key)));
    }
    else
    {
      std::stringstream ss;
      ss << "Unsupported type for variable '" << key << "'.";
      URCL_LOG_ERROR(ss.str().c_str());
      throw UrException(ss.str().c_str());
    }
    script_code.replace(match.position(0), match.length(0), replaced_value);
  }
}

}  // namespace control
}  // namespace urcl
