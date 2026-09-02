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

#pragma once
#include <cstddef>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include <ur_client_library/ur/datatypes.h>
#include <ur_client_library/ur/version_information.h>

namespace urcl
{
namespace control
{
/*!
 * \brief This class handles reading script files parsing special instructions that will get replaced.
 *
 * When parsing the script code, it is supported to have
 *   - Variable replacements using `{{ VARIABLE_NAME }}`
 *   - Including other files using `{% include <filename> %}`.
 *     The filename has to be relative to the root script file's folder
 *   - Conditionals using
 *
 *         {% if <condition %}
 *           ...
 *         {% elif <condition> %}
 *           ...
 *         {% else %}
 *           ...
 *         {% endif %}
 *
 *
 * Those directives use Jinja2 notation.
 */
class ScriptReader
{
public:
  using DataVariant = std::variant<std::string, double, int, bool, VersionInformation>;
  using DataDict = std::unordered_map<std::string, DataVariant>;

  ScriptReader() = default;

  /*!
   * \brief Reads a script file and applies variable replacements, includes, and conditionals.
   * \param file_path Path of the script file to be loaded.
   * \param data Data dictionary used for variable replacements and expression evaluation.
   * \return The Script code with all replacements, includes and conditionals applied.
   */
  std::string readScriptFile(const std::string& file_path, const DataDict& data = DataDict());

  /*!
   * \brief Evaluate a boolean expression
   * \param expression The boolean expression to be evaluated.
   * \param data A data dictionary that will be used when evaluating the expressions
   * \return The result of evaluating the boolean expression
   */
  static bool evaluateExpression(const std::string& expression, const DataDict& data);

  /*!
   * \brief Checks whether a variable placeholder was substituted during the last script read.
   * This allows detecting which features a user-supplied script supports.
   * \param key The variable name to check.
   * \return True if the last read script, including its includes, contained the placeholder.
   */
  bool isVariableRegistered(const std::string& key) const;

private:
  enum BlockType
  {
    IF,
    ELIF,
    ELSE
  };
  struct BlockState
  {
    BlockType type;
    bool condition_matched;  // Has any previous condition in this block matched?
    bool should_render;      // Should this block render?
    bool parent_render;      // Is the parent block rendering?
  };

  // Parameters for recursive script reading and include handling
  static constexpr std::size_t MAX_INCLUDE_DEPTH = 32;
  std::filesystem::path root_dir_;
  std::filesystem::path current_dir_;
  std::unordered_set<std::string> include_stack_;
  std::size_t include_depth_ = 0;
  std::unordered_set<std::string> variable_registry_;

  std::string readScriptFileImpl(const std::filesystem::path& canonical_path, const DataDict& data);
  static std::string readFileContent(const std::string& file_path);
  void replaceIncludes(std::string& script_code, const DataDict& data);
  static void replaceVariables(std::string& script_code, const DataDict& data,
                               std::unordered_set<std::string>& variable_registry);
  static void replaceConditionals(std::string& script_code, const DataDict& data);
};

bool operator<(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs);
bool operator>(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs);
bool operator==(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs);

inline bool operator!=(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  return !(lhs == rhs);
}
inline bool operator<=(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  return (lhs < rhs || lhs == rhs);
}
inline bool operator>=(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  return (lhs > rhs || lhs == rhs);
}
}  // namespace control
}  // namespace urcl
