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

namespace urcl
{
namespace control
{
std::string ScriptReader::readScriptFile(const std::string& filename)
{
  script_path_ = filename;
  std::string script_code = readFileContent(filename);

  replaceIncludes(script_code);

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
  std::string line;
  std::regex include_pattern(R"(\{\%\s*include\s*['|"]([^'"]+)['|"]\s*\%\})");

  std::stringstream input_stream(script);
  std::stringstream output_stream;

  while (std::getline(input_stream, line))
  {
    std::smatch match;
    std::string processed_line = line;

    // Replace all include patterns in the line
    while (std::regex_search(processed_line, match, include_pattern))
    {
      std::filesystem::path file_path(match[1]);
      std::string file_content = readFileContent(script_path_.parent_path() / file_path.string());
      processed_line.replace(match.position(0), match.length(0), file_content);
    }

    output_stream << processed_line << '\n';
  }
  script = output_stream.str();
}

}  // namespace control
}  // namespace urcl
