// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 Universal Robots A/S
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
#include <ur_client_library/rtde/control_package_setup_outputs.h>

using namespace urcl;

TEST(rtde_control_package_setup_outputs, generate_serialized_setup_output_request_protocolv2)
{
  uint8_t buffer[4096];
  std::vector<std::string> output_recipe;
  output_recipe.push_back("actual_q");
  output_recipe.push_back("actual_qd");
  double output_frequency = 500;
  size_t expected_size = 29;
  size_t actual_size = rtde_interface::ControlPackageSetupOutputsRequest::generateSerializedRequest(
      buffer, output_frequency, output_recipe);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x1d, 0x4f, 0x40, 0x7f, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x63, 0x74, 0x75,
                         0x61, 0x6c, 0x5f, 0x71, 0x2c, 0x61, 0x63, 0x74, 0x75, 0x61, 0x6c, 0x5f, 0x71, 0x64 };

  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_control_package_setup_outputs, generate_serialized_setup_output_request_protocolv1)
{
  uint8_t buffer[4096];
  std::vector<std::string> output_recipe;
  output_recipe.push_back("actual_q");
  output_recipe.push_back("actual_qd");
  size_t expected_size = 21;
  size_t actual_size =
      rtde_interface::ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, output_recipe);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x15, 0x4f, 0x61, 0x63, 0x74, 0x75, 0x61, 0x6c, 0x5f, 0x71,
                         0x2c, 0x61, 0x63, 0x74, 0x75, 0x61, 0x6c, 0x5f, 0x71, 0x64 };

  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_control_package_setup_outputs, empty_output_recipe)
{
  uint8_t buffer[4096];
  std::vector<std::string> output_recipe;
  size_t expected_size = 0;
  size_t actual_size =
      rtde_interface::ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, output_recipe);

  EXPECT_EQ(expected_size, actual_size);

  double output_frequency = 100;
  actual_size = rtde_interface::ControlPackageSetupOutputsRequest::generateSerializedRequest(buffer, output_frequency,
                                                                                             output_recipe);

  EXPECT_EQ(expected_size, actual_size);
}

TEST(rtde_control_package_setup_outputs, parse_accepted_setup_output_protocolv2)
{
  uint8_t setup_outputs_answer[] = { 0x01, 0x44, 0x4f, 0x55, 0x42, 0x4c, 0x45, 0x2c,
                                     0x56, 0x45, 0x43, 0x54, 0x4f, 0x52, 0x36, 0x44 };
  comm::BinParser bp(setup_outputs_answer, sizeof(setup_outputs_answer));
  rtde_interface::ControlPackageSetupOutputs setup_outputs(2);

  EXPECT_TRUE(setup_outputs.parseWith(bp));

  uint8_t expected_output_recipe_id = 1;
  std::string expected_variable_types = "DOUBLE,VECTOR6D";

  EXPECT_EQ(expected_output_recipe_id, setup_outputs.output_recipe_id_);
  EXPECT_EQ(expected_variable_types, setup_outputs.variable_types_);
}

TEST(rtde_control_package_setup_outputs, parse_not_accepted_setup_output_protocolv2)
{
  uint8_t setup_outputs_answer[] = { 0x00, 0x4e, 0x4f, 0x54, 0x5f, 0x46, 0x4f, 0x55, 0x4e, 0x44,
                                     0x2c, 0x56, 0x45, 0x43, 0x54, 0x4f, 0x52, 0x36, 0x44 };
  comm::BinParser bp(setup_outputs_answer, sizeof(setup_outputs_answer));
  rtde_interface::ControlPackageSetupOutputs setup_outputs(2);

  EXPECT_TRUE(setup_outputs.parseWith(bp));

  uint8_t expected_output_recipe_id = 0;
  std::string expected_variable_types = "NOT_FOUND,VECTOR6D";

  EXPECT_EQ(expected_output_recipe_id, setup_outputs.output_recipe_id_);
  EXPECT_EQ(expected_variable_types, setup_outputs.variable_types_);
}

TEST(rtde_control_package_setup_outputs, parse_accepted_setup_output_protocolv1)
{
  uint8_t setup_outputs_answer[] = { 0x44, 0x4f, 0x55, 0x42, 0x4c, 0x45, 0x2c, 0x56,
                                     0x45, 0x43, 0x54, 0x4f, 0x52, 0x36, 0x44 };
  comm::BinParser bp(setup_outputs_answer, sizeof(setup_outputs_answer));
  rtde_interface::ControlPackageSetupOutputs setup_outputs(1);

  EXPECT_TRUE(setup_outputs.parseWith(bp));

  std::string expected_variable_types = "DOUBLE,VECTOR6D";

  EXPECT_EQ(expected_variable_types, setup_outputs.variable_types_);
}

TEST(rtde_control_package_setup_outputs, parse_not_accepted_setup_output_protocolv1)
{
  uint8_t setup_outputs_answer[] = { 0x4e, 0x4f, 0x54, 0x5f, 0x46, 0x4f, 0x55, 0x4e, 0x44,
                                     0x2c, 0x56, 0x45, 0x43, 0x54, 0x4f, 0x52, 0x36, 0x44 };
  comm::BinParser bp(setup_outputs_answer, sizeof(setup_outputs_answer));
  rtde_interface::ControlPackageSetupOutputs setup_outputs(1);

  EXPECT_TRUE(setup_outputs.parseWith(bp));

  std::string expected_variable_types = "NOT_FOUND,VECTOR6D";

  EXPECT_EQ(expected_variable_types, setup_outputs.variable_types_);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
