// Test-only metadata for output fields extracted from the public RTDE documentation.
#ifndef URCL_TESTS_RTDE_OUTPUT_RECIPE_METADATA_H
#define URCL_TESTS_RTDE_OUTPUT_RECIPE_METADATA_H

#include <fstream>
#include <map>
#include <regex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <ur_client_library/ur/version_information.h>

namespace urcl
{
namespace test
{
// Headerless TSV: name, PolyScope 5 availability, PolyScope X availability.
// '*' means unrestricted; '-' means unavailable; otherwise use major.minor.patch.
// This is NOT the recipe format accepted by RTDEClient's file constructor.
struct OutputAvailability
{
  std::string polyscope5;
  std::string polyscopex;

  bool availableOn(const VersionInformation& controller) const
  {
    if (controller.major != 5 && controller.major != 10)
    {
      throw std::runtime_error("RTDE documentation metadata only supports PolyScope 5 and X");
    }
    const auto& requirement = controller.major == 5 ? polyscope5 : polyscopex;
    if (requirement == "*" || requirement == "-")
    {
      return requirement == "*";
    }
    const auto minimum = VersionInformation::fromString(requirement);
    // Controller build numbers do not affect documented introduction versions.
    return std::tie(controller.major, controller.minor, controller.bugfix) >=
           std::tie(minimum.major, minimum.minor, minimum.bugfix);
  }
};

using OutputMetadata = std::map<std::string, OutputAvailability>;

inline void validateAvailability(const std::string& value, uint32_t family)
{
  if (value == "*" || value == "-")
  {
    return;
  }
  if (!std::regex_match(value, std::regex("[0-9]+\\.[0-9]+\\.[0-9]+")) ||
      VersionInformation::fromString(value).major != family)
  {
    throw std::runtime_error("Invalid availability '" + value + "' for family " + std::to_string(family));
  }
}

inline OutputMetadata readOutputMetadata(std::istream& input)
{
  OutputMetadata metadata;
  std::string line;
  size_t line_number = 0;
  while (std::getline(input, line))
  {
    ++line_number;
    if (!line.empty() && line.back() == '\r')
    {
      line.pop_back();
    }
    try
    {
      const auto first = line.find('\t');
      const auto second = first == std::string::npos ? std::string::npos : line.find('\t', first + 1);
      if (first == std::string::npos || second == std::string::npos || line.find('\t', second + 1) != std::string::npos)
      {
        throw std::runtime_error("Expected name, PolyScope 5 and PolyScope X tab-separated columns");
      }
      const auto name = line.substr(0, first);
      if (!std::regex_match(name, std::regex("[A-Za-z_][A-Za-z_0-9]*")))
      {
        throw std::runtime_error("Invalid field name '" + name + "'");
      }
      const OutputAvailability availability{ line.substr(first + 1, second - first - 1), line.substr(second + 1) };
      validateAvailability(availability.polyscope5, 5);
      validateAvailability(availability.polyscopex, 10);
      if (!metadata.emplace(name, availability).second)
      {
        throw std::runtime_error("Duplicate field '" + name + "'");
      }
    }
    catch (const std::exception& error)
    {
      const auto context = "RTDE metadata line " + std::to_string(line_number) + " (" + line + "): ";
      throw std::runtime_error(context + error.what());
    }
  }
  if (input.bad() || metadata.empty())
  {
    throw std::runtime_error("Unreadable or empty RTDE output metadata");
  }
  return metadata;
}

inline OutputMetadata loadOutputMetadata(const std::string& path)
{
  std::ifstream input(path);
  if (!input)
  {
    throw std::runtime_error("Cannot open RTDE output metadata: " + path);
  }
  return readOutputMetadata(input);
}

inline bool supportsOutputMetadata(const VersionInformation& controller)
{
  return (controller.major == 5 && controller.minor >= 9) || (controller.major == 10 && controller.minor >= 10);
}

inline std::vector<std::string> filterOutputRecipe(const std::vector<std::string>& recipe,
                                                   const OutputMetadata& metadata, const VersionInformation& controller,
                                                   std::ostream& excluded)
{
  if (!supportsOutputMetadata(controller))
  {
    throw std::runtime_error("RTDE output metadata requires PolyScope 5 >= 5.9.0 or PolyScope X >= 10.10.0");
  }
  std::vector<std::string> result;
  for (const auto& name : recipe)
  {
    const auto entry = metadata.find(name);
    if (entry == metadata.end())
    {
      throw std::runtime_error("Missing RTDE documentation metadata for '" + name + "'");
    }
    if (entry->second.availableOn(controller))
    {
      result.push_back(name);
    }
    else
    {
      excluded << "Skipping RTDE field " << name << " on " << controller.toString() << " (requires "
               << (controller.major == 5 ? entry->second.polyscope5 : entry->second.polyscopex) << ")\n";
    }
  }
  return result;
}
}  // namespace test
}  // namespace urcl

#endif