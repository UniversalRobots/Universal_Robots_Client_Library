#include <gtest/gtest.h>
#include <sstream>
#include "rtde_output_recipe_metadata.h"

using namespace urcl;

namespace
{
test::OutputMetadata exampleMetadata()
{
  std::istringstream input("timestamp\t*\t*\n"
                           "ft_raw_wrench\t5.9.0\t*\n"
                           "payload_inertia\t5.11.0\t*\n"
                           "actual_current_as_torque\t5.23.0\t10.11.0\n"
                           "target_gravity\t5.26.0\t10.12.0\n"
                           "control_step\t5.26.0\t10.13.0\n"
                           "target_base_wrench\t5.26.0\t10.14.0\n"
                           "tool_output_voltage_2\t-\t10.15.0\n"
                           "safety_status_bits\t*\t*\n");
  return test::readOutputMetadata(input);
}
}  // namespace

TEST(RTDEOutputRecipeMetadataTest, independent_family_thresholds)
{
  const auto metadata = exampleMetadata();
  const auto& torque = metadata.at("actual_current_as_torque");
  for (const auto& version : { "5.22.9.99999", "10.10.9.99999" })
  {
    EXPECT_FALSE(torque.availableOn(VersionInformation::fromString(version))) << version;
  }
  for (const auto& version : { "5.23.0", "5.26.0", "10.11.0", "10.14.0" })
  {
    EXPECT_TRUE(torque.availableOn(VersionInformation::fromString(version))) << version;
  }
  const test::OutputAvailability patch{ "5.23.2", "10.11.2" };
  EXPECT_FALSE(patch.availableOn(VersionInformation::fromString("5.23.1.99999")));
  EXPECT_TRUE(patch.availableOn(VersionInformation::fromString("5.23.2")));
  EXPECT_TRUE(patch.availableOn(VersionInformation::fromString("10.12.0")));
  EXPECT_FALSE(patch.availableOn(VersionInformation::fromString("10.11.1.99999")));
}

TEST(RTDEOutputRecipeMetadataTest, missing_family_and_unrestricted_fields)
{
  const auto metadata = exampleMetadata();
  EXPECT_FALSE(metadata.at("payload_inertia").availableOn(VersionInformation::fromString("5.9.0")));
  EXPECT_TRUE(metadata.at("payload_inertia").availableOn(VersionInformation::fromString("5.11.0")));
  EXPECT_TRUE(metadata.at("payload_inertia").availableOn(VersionInformation::fromString("10.10.0")));
  EXPECT_FALSE(metadata.at("tool_output_voltage_2").availableOn(VersionInformation::fromString("5.99.0")));
  EXPECT_FALSE(metadata.at("tool_output_voltage_2").availableOn(VersionInformation::fromString("10.14.9")));
  EXPECT_TRUE(metadata.at("tool_output_voltage_2").availableOn(VersionInformation::fromString("10.15.0")));
  for (const auto& version : { "5.9.0", "10.10.0" })
  {
    EXPECT_TRUE(metadata.at("safety_status_bits").availableOn(VersionInformation::fromString(version)));
    EXPECT_TRUE(metadata.at("ft_raw_wrench").availableOn(VersionInformation::fromString(version)));
  }
  // A below-cutoff X-only introduction must still be unavailable on PolyScope 5.
  std::istringstream input("legacy_x_only\t-\t*\n");
  const auto legacy = test::readOutputMetadata(input);
  EXPECT_FALSE(legacy.at("legacy_x_only").availableOn(VersionInformation::fromString("5.26.0")));
  EXPECT_TRUE(legacy.at("legacy_x_only").availableOn(VersionInformation::fromString("10.10.0")));
}

TEST(RTDEOutputRecipeMetadataTest, filters_in_order_without_weakening_completeness)
{
  const auto metadata = exampleMetadata();
  const std::vector<std::string> recipe{ "timestamp", "target_base_wrench", "control_step", "target_gravity",
                                         "tool_output_voltage_2" };
  const std::vector<std::vector<std::string>> expected{ { "timestamp" },
                                                        { "timestamp", "target_gravity" },
                                                        { "timestamp", "control_step", "target_gravity" },
                                                        { "timestamp", "target_base_wrench", "control_step",
                                                          "target_gravity" },
                                                        recipe };
  for (uint32_t minor = 11; minor <= 15; ++minor)
  {
    std::ostringstream excluded;
    const auto controller = VersionInformation::fromString("10." + std::to_string(minor) + ".0");
    EXPECT_EQ(test::filterOutputRecipe(recipe, metadata, controller, excluded), expected.at(minor - 11));
    EXPECT_EQ(metadata.size(), 9u);  // Completeness checking retains every field, including X-only ones.
    if (minor < 15)
    {
      EXPECT_NE(excluded.str().find("tool_output_voltage_2"), std::string::npos);
      EXPECT_NE(excluded.str().find("10.15.0"), std::string::npos);
    }
  }
  std::ostringstream excluded;
  EXPECT_EQ(test::filterOutputRecipe(recipe, metadata, VersionInformation::fromString("5.26.0"), excluded),
            expected.at(3));
  EXPECT_THROW(test::filterOutputRecipe({ "unknown" }, metadata, VersionInformation::fromString("5.26.0"), excluded),
               std::runtime_error);
}

TEST(RTDEOutputRecipeMetadataTest, supported_controller_baselines)
{
  for (const auto& version : { "3.14.3", "5.8.99", "10.7.0", "10.9.99", "11.0.0" })
  {
    const auto controller = VersionInformation::fromString(version);
    EXPECT_FALSE(test::supportsOutputMetadata(controller)) << version;
    std::ostringstream excluded;
    const auto metadata = exampleMetadata();
    EXPECT_THROW(test::filterOutputRecipe({ "timestamp" }, metadata, controller, excluded), std::runtime_error);
  }
  for (const auto& version : { "5.9.0", "5.9.4", "10.10.0", "10.15.0" })
  {
    EXPECT_TRUE(test::supportsOutputMetadata(VersionInformation::fromString(version))) << version;
  }
  EXPECT_THROW(exampleMetadata().at("timestamp").availableOn(VersionInformation::fromString("3.14.3")),
               std::runtime_error);
}

TEST(RTDEOutputRecipeMetadataTest, rejects_invalid_or_incomplete_metadata)
{
  for (const auto& text : { "", "timestamp\n", "timestamp\t*\n", "\t*\t*\n", "timestamp\t*\t*\textra\n",
                            "timestamp\t\t*\n", "timestamp\t*\t\n", "timestamp\t10.11.0\t*\n", "timestamp\t*\t5.23.0\n",
                            "timestamp\t5.23\t*\n", "timestamp\t5.23.0junk\t*\n", "timestamp\t*\t*\ntimestamp\t*\t*\n",
                            "invalid name\t*\t*\n", "timestamp\t5.99999999999999999999.0\t*\n" })
  {
    std::istringstream input(text);
    EXPECT_THROW(test::readOutputMetadata(input), std::runtime_error) << text;
  }
  EXPECT_THROW(test::loadOutputMetadata("missing_rtde_metadata_file.tsv"), std::runtime_error);
}

TEST(RTDEOutputRecipeMetadataTest, reads_crlf_and_reports_bad_line_context)
{
  std::istringstream valid("timestamp\t*\t*\r\n");
  EXPECT_EQ(test::readOutputMetadata(valid).size(), 1u);
  std::istringstream invalid("timestamp\t*\t*\nactual_q\t5.23\t*\n");
  try
  {
    test::readOutputMetadata(invalid);
    FAIL() << "Malformed metadata was accepted";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("line 2"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("actual_q"), std::string::npos);
  }
}