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
#include <ur_client_library/helpers.h>
#include <ur_client_library/log.h>

#include <stack>
#include <fstream>
#include <regex>

namespace urcl
{
namespace control
{
constexpr double ZERO_EPSILON = 0.000001;

bool operator<(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  if (std::holds_alternative<double>(lhs))
  {
    if (std::holds_alternative<double>(rhs))
    {
      return std::get<double>(lhs) < std::get<double>(rhs);
    }
    else if (std::holds_alternative<int>(rhs))
    {
      return std::get<double>(lhs) < static_cast<double>(std::get<int>(rhs));
    }
  }
  if (std::holds_alternative<int>(lhs))
  {
    if (std::holds_alternative<double>(rhs))
    {
      return static_cast<double>(std::get<int>(lhs)) < std::get<double>(rhs);
    }
    else if (std::holds_alternative<int>(rhs))
    {
      return std::get<int>(lhs) < std::get<int>(rhs);
    }
  }
  if (std::holds_alternative<VersionInformation>(lhs))
  {
    return std::get<VersionInformation>(lhs) < std::get<VersionInformation>(rhs);
  }
  throw std::invalid_argument("The comparison operator is only allowed for numeric values.");
}

bool operator>(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  return !(lhs < rhs || lhs == rhs);
}

bool operator==(const ScriptReader::DataVariant& lhs, const ScriptReader::DataVariant& rhs)
{
  if (lhs.index() != rhs.index())
  {
    // Allow comparison between int and double
    if ((std::holds_alternative<int>(lhs) && std::holds_alternative<double>(rhs)))
    {
      return static_cast<double>(std::get<int>(lhs)) == std::get<double>(rhs);
    }
    if ((std::holds_alternative<double>(lhs) && std::holds_alternative<int>(rhs)))
    {
      return std::get<double>(lhs) == static_cast<double>(std::get<int>(rhs));
    }
    // Allow comparison between bool and int/double
    if (std::holds_alternative<bool>(lhs))
    {
      if (std::holds_alternative<double>(rhs))
      {
        return std::abs((static_cast<double>(std::get<bool>(lhs)) - std::get<double>(rhs))) < ZERO_EPSILON;
      }
      if (std::holds_alternative<int>(rhs))
      {
        return std::abs((static_cast<double>(std::get<bool>(lhs)) - std::get<int>(rhs))) == 0;
      }
    }
    throw std::invalid_argument(
        "Checking equality of types is not allowed: " +
        std::string(lhs.index() == 0 ? "string" : (lhs.index() == 1 ? "double" : (lhs.index() == 2 ? "int" : "bool"))) +
        " with " +
        std::string(rhs.index() == 0 ? "string" : (rhs.index() == 1 ? "double" : (rhs.index() == 2 ? "int" : "bool"))));
  }
  if (std::holds_alternative<std::string>(lhs))
  {
    return std::get<std::string>(lhs) == std::get<std::string>(rhs);
  }
  if (std::holds_alternative<double>(lhs))
  {
    return std::get<double>(lhs) == std::get<double>(rhs);
  }
  if (std::holds_alternative<int>(lhs))
  {
    return std::get<int>(lhs) == std::get<int>(rhs);
  }
  if (std::holds_alternative<bool>(lhs))
  {
    return std::get<bool>(lhs) == std::get<bool>(rhs);
  }
  if (std::holds_alternative<VersionInformation>(lhs))
  {
    return std::get<VersionInformation>(lhs) == std::get<VersionInformation>(rhs);
  }
  throw std::runtime_error("Unknown variant type passed to equality check. Please contact the developers.");
}

namespace
{
// Returns true if `candidate` (already lexically-normalized) is inside `root` (also normalized).
bool isPathInside(const std::filesystem::path& candidate, const std::filesystem::path& root)
{
  const auto rel = candidate.lexically_relative(root);
  if (rel.empty())
  {
    // No relative path exists (e.g. different roots on Windows) -> definitely outside.
    return false;
  }
  for (const auto& part : rel)
  {
    if (part == "..")
    {
      return false;
    }
  }
  return true;
}

// Runs std::filesystem::weakly_canonical on `input` and wraps any filesystem_error in a UrException
// with a message that includes `context` (e.g. "script file path" / "include path") and the raw
// user-facing spelling of the path (`raw`). Both call sites in ScriptReader need identical wrapping
// but with a different label and different raw string, so keep those parameters explicit.
//
// weakly_canonical is preferred over canonical because it tolerates trailing components that do
// not yet exist -- we defer "file not found" reporting to readFileContent, which produces a nicer
// error. Symlink cycles and similar low-level failures still throw here, and are what this helper
// converts.
std::filesystem::path canonicalizeOrThrow(const std::filesystem::path& input, const std::string& raw,
                                          const char* context)
{
  try
  {
    return std::filesystem::weakly_canonical(input);
  }
  catch (const std::filesystem::filesystem_error& e)
  {
    std::stringstream ss;
    ss << "Could not resolve " << context << " '" << raw << "': " << e.what();
    throw UrException(ss.str().c_str());
  }
}
}  // namespace

bool ScriptReader::isVariableRegistered(const std::string& key) const
{
  return variable_registry_.count(key) != 0;
}

std::string ScriptReader::readScriptFile(const std::string& filename, const DataDict& data)
{
  // Top-level entry point: normalize the input path, set up the include root, reset the include
  // stack/depth counters, then delegate to the internal impl.
  const std::filesystem::path canonical_input =
      canonicalizeOrThrow(std::filesystem::path(filename), filename, "script file path");

  root_dir_ = canonical_input.parent_path();
  current_dir_.clear();
  include_stack_.clear();
  include_depth_ = 0;
  variable_registry_.clear();

  return readScriptFileImpl(canonical_input, data);
}

std::string ScriptReader::readScriptFileImpl(const std::filesystem::path& canonical_path, const DataDict& data)
{
  const std::string path_key = canonical_path.string();

  if (include_stack_.count(path_key) != 0)
  {
    throw UrException(("Circular include detected while processing script file: '" + path_key + "'").c_str());
  }
  if (include_depth_ >= MAX_INCLUDE_DEPTH)
  {
    std::stringstream ss;
    ss << "Maximum include depth (" << MAX_INCLUDE_DEPTH << ") exceeded while processing script file: '" << path_key
       << "'";
    throw UrException(ss.str().c_str());
  }

  // Save-restore the per-file state so a nested include cannot corrupt the parent's include base.
  const std::filesystem::path previous_dir = current_dir_;
  current_dir_ = canonical_path.parent_path();
  include_stack_.insert(path_key);
  ++include_depth_;

  std::string script_code;
  try
  {
    script_code = readFileContent(path_key);
    replaceVariables(script_code, data, variable_registry_);
    replaceConditionals(script_code, data);
    replaceIncludes(script_code, data);
  }
  catch (...)
  {
    include_stack_.erase(path_key);
    --include_depth_;
    current_dir_ = previous_dir;
    throw;
  }

  include_stack_.erase(path_key);
  --include_depth_;
  current_dir_ = previous_dir;
  return script_code;
}

std::string ScriptReader::readFileContent(const std::string& file_path)
{
  std::ifstream ifs;
  ifs.open(file_path);
  std::string content;
  if (ifs)
  {
    content = std::string((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));
    ifs.close();
  }
  else
  {
    std::stringstream ss;
    ss << "Could not open script file '" << file_path << "'. Please check if the file exists and is readable.";
    throw UrException(ss.str().c_str());
  }

  return content;
}

void ScriptReader::replaceIncludes(std::string& script, const DataDict& data)
{
  // Character class fixed: previously the regex used `['|"]` which matches `'`, `|`, or `"` (the
  // `|` is literal inside a character class, not alternation).
  std::regex include_pattern(R"(\{\%\s*include\s*['"]([^'"]+)['"]\s*\%\})");

  std::smatch match;

  // Replace all include patterns in the script.
  while (std::regex_search(script, match, include_pattern))
  {
    const std::string raw = match[1].str();
    std::filesystem::path requested(raw);

    // Reject any path with a root name and/or a root directory. On POSIX this only rules out
    // absolute paths (`/etc/passwd`). On Windows this additionally covers:
    //   * `\evil.txt`         (root directory but no drive) — operator/= would prepend LHS's drive
    //     and drop LHS's relative portion, yielding `C:\evil.txt` and escaping root_dir_.
    //   * `C:evil.txt`        (drive-relative, no root directory) — safe when the drive matches
    //     LHS, but for a different drive operator/= replaces LHS entirely.
    //   * `\\server\share\x`  (UNC), `\\?\C:\x` (extended-length) — already caught by is_absolute()
    //     but redundantly rejected here.
    // std::filesystem::path::operator/ has surprising drop-LHS semantics in all these cases, so
    // rejecting them up front is safer than relying on the containment check alone.
    if (requested.is_absolute() || requested.has_root_name() || requested.has_root_directory())
    {
      throw UrException(("Rooted or absolute include paths are not allowed: '" + raw + "'").c_str());
    }

    // weakly_canonical collapses `..` components and resolves symlinks where possible, which is
    // what we need to make the containment check trustworthy.
    const std::filesystem::path resolved = canonicalizeOrThrow(current_dir_ / requested, raw, "include path");

    if (!isPathInside(resolved, root_dir_))
    {
      std::stringstream ss;
      ss << "Include path '" << raw << "' resolves outside of the script root directory '" << root_dir_.string()
         << "' (resolved to '" << resolved.string() << "')";
      throw UrException(ss.str().c_str());
    }

    // Recurse via the impl so include_stack_/include_depth_/current_dir_ stay consistent.
    std::string file_content = readScriptFileImpl(resolved, data);
    script.replace(match.position(0), match.length(0), file_content);
  }
}

void ScriptReader::replaceVariables(std::string& script_code, const DataDict& data,
                                    std::unordered_set<std::string>& variable_registry)
{
  std::regex pattern(R"(\{\{\s*([\w-]+)\s*\}\})");
  std::smatch match;
  while (std::regex_search(script_code, match, pattern))
  {
    std::string key = match[1];
    if (data.find(key) == data.end())
    {
      std::stringstream ss;
      ss << "Variable '" << key << "' not found in data.";
      URCL_LOG_ERROR(ss.str().c_str());
      throw UnknownVariable(ss.str().c_str());
    }
    variable_registry.insert(key);
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
    else if (std::holds_alternative<bool>(data.at(key)))
    {
      std::get<bool>(data.at(key)) ? replaced_value = "True" : replaced_value = "False";
    }
    else if (std::holds_alternative<VersionInformation>(data.at(key)))
    {
      replaced_value = std::get<VersionInformation>(data.at(key)).toString();
    }
    else
    {
      // This is more of a reminder if we add types to the variant and forget to add it here.
      std::stringstream ss;
      ss << "Unsupported type for variable '" << key << "'.";
      URCL_LOG_ERROR(ss.str().c_str());
      throw UrException(ss.str().c_str());
    }
    script_code.replace(match.position(0), match.length(0), replaced_value);
  }
}

void ScriptReader::replaceConditionals(std::string& script_code, const DataDict& data)
{
  std::istringstream stream(script_code);
  std::ostringstream output;
  std::string line;
  std::stack<BlockState> block_stack;

  std::regex if_pattern(R"(\{\%\s*if\s+([^\s].*[^\s])\s+\%\})");
  std::regex elif_pattern(R"(\{\%\s*elif\s+([^\s].*[^\s])\s+\%\})");
  std::regex else_pattern(R"(\{\%\s*else\s*\%\})");
  std::regex endif_pattern(R"(\{\%\s*endif\s*\%\})");
  std::smatch match;

  bool first_line = true;
  while (std::getline(stream, line))
  {
    if (std::regex_search(line, match, if_pattern))
    {
      std::string condition = match[1];
      bool result = evaluateExpression(condition, data);
      bool parent_render = block_stack.empty() ? true : block_stack.top().should_render;
      block_stack.push({ IF, result, parent_render && result, parent_render });
    }
    else if (std::regex_search(line, match, elif_pattern))
    {
      if (!block_stack.empty())
      {
        BlockState& top = block_stack.top();
        if (top.type == ELSE)
          continue;
        std::string condition = match[1];
        bool result = evaluateExpression(condition, data);
        top.type = ELIF;
        if (!top.condition_matched && result)
        {
          top.condition_matched = true;
          top.should_render = top.parent_render;
        }
        else
        {
          top.should_render = false;
        }
      }
    }
    else if (std::regex_search(line, match, else_pattern))
    {
      if (!block_stack.empty())
      {
        BlockState& top = block_stack.top();
        top.type = ELSE;
        top.should_render = top.parent_render && !top.condition_matched;
      }
    }
    else if (std::regex_search(line, match, endif_pattern))
    {
      if (!block_stack.empty())
      {
        block_stack.pop();
      }
    }
    else
    {
      if (block_stack.empty() || block_stack.top().should_render)
      {
        if (!first_line)
        {
          output << "\n";
        }
        output << line;
        first_line = false;
      }
    }
  }

  script_code = output.str();
}

bool ScriptReader::evaluateExpression(const std::string& expression, const DataDict& data)
{
  const std::string trimmed = std::regex_replace(expression, std::regex("^\\s+|\\s+$"), "");
  std::regex expression_pattern(R"(([a-zA-Z_][a-zA-Z0-9_]*)\s*([!=<>]=?)\s*(["']?[^'"]*["']?))");

  std::smatch match;
  if (std::regex_search(trimmed, match, expression_pattern))
  {
    std::string variable_name = match[1];
    std::string comp_operator = match[2];
    std::string value_str = match[3];
    DataVariant value = value_str;

    if (!data.count(variable_name))
    {
      throw UnknownVariable(variable_name);
    }

    std::stringstream ss;
    ss << "Evaluating trimmed: " << variable_name << " " << comp_operator << " " << value_str << std::endl;
    URCL_LOG_DEBUG(ss.str().c_str());

    // Is the value a string, a number or a variable?
    std::regex string_pattern(R"(^['"]([^'"]+)?['"]$)");
    std::regex number_pattern(R"(^-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?$)");
    std::regex boolean_pattern(R"(^(true|false|yes|no|on|off)$)", std::regex::icase);
    std::regex version_pattern(R"(^v(\d+\.\d+(\.\d+)?(-\d+)?)$)");
    if (std::regex_search(value_str, match, string_pattern))
    {
      value = match[1];  // Extract the string content without quotes
    }
    else if (std::regex_search(value_str, match, number_pattern))
    {
      value = std::stod(value_str);
    }
    else if (std::regex_search(value_str, match, boolean_pattern))
    {
      value = parseBoolean(value_str);
    }
    else if (std::regex_search(value_str, match, version_pattern))
    {
      value = VersionInformation::fromString(match[1]);
    }
    else if (data.count(value_str))
    {
      value = data.at(value_str);
    }
    else
    {
      throw UnknownVariable(value_str);
    }

    if (comp_operator == "==")
    {
      return data.at(variable_name) == value;
    }
    else if (comp_operator == "!=")
    {
      return data.at(variable_name) != value;
    }
    else if (comp_operator == "<")
    {
      return data.at(variable_name) < value;
    }
    else if (comp_operator == ">")
    {
      return data.at(variable_name) > value;
    }
    else if (comp_operator == "<=")
    {
      return data.at(variable_name) <= value;
    }
    else if (comp_operator == ">=")
    {
      return data.at(variable_name) >= value;
    }
  }
  else if (std::regex_match(trimmed, std::regex(R"([a-zA-Z_][a-zA-Z0-9_]*)")))
  {
    if (!data.count(trimmed))
    {
      throw UnknownVariable(trimmed);
    }
    if (std::holds_alternative<bool>(data.at(trimmed)))
    {
      return std::get<bool>(data.at(trimmed));
    }
  }

  throw std::runtime_error("trimmed evaluation failed: `" + trimmed +
                           "`. Expected a boolean value, but got a different type.");
}

}  // namespace control
}  // namespace urcl
