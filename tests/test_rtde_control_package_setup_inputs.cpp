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
#include <ur_client_library/rtde/control_package_setup_inputs.h>

using namespace urcl;

TEST(rtde_control_package_setup_inputs, generate_serialized_setup_inputs_request)
{
  uint8_t buffer[4096];
  size_t expected_size = 42;
  std::vector<std::string> input_recipe;
  input_recipe.push_back("speed_slider_mask");
  input_recipe.push_back("speed_slider_fraction");
  size_t actual_size =
      rtde_interface::ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe);

  EXPECT_EQ(expected_size, actual_size);

  uint8_t expected[] = { 0x00, 0x2a, 0x49, 0x73, 0x70, 0x65, 0x65, 0x64, 0x5f, 0x73, 0x6c, 0x69, 0x64, 0x65,
                         0x72, 0x5f, 0x6d, 0x61, 0x73, 0x6b, 0x2c, 0x73, 0x70, 0x65, 0x65, 0x64, 0x5f, 0x73,
                         0x6c, 0x69, 0x64, 0x65, 0x72, 0x5f, 0x66, 0x72, 0x61, 0x63, 0x74, 0x69, 0x6f, 0x6e };
  for (unsigned int i = 0; i < actual_size; ++i)
  {
    EXPECT_EQ(expected[i], buffer[i]);
  }
}

TEST(rtde_control_package_setup_inputs, empty_input_recipe)
{
  uint8_t buffer[4096];
  size_t expected_size = 0;
  std::vector<std::string> input_recipe;
  size_t actual_size =
      rtde_interface::ControlPackageSetupInputsRequest::generateSerializedRequest(buffer, input_recipe);

  EXPECT_EQ(expected_size, actual_size);
}

TEST(rtde_control_package_setup_inputs, parse_accepted_setup_inputs)
{
  uint8_t setup_inputs_answer[] = {
    0x01, 0x55, 0x49, 0x4e, 0x54, 0x33, 0x32, 0x2c, 0x44, 0x4f, 0x55, 0x42, 0x4c, 0x45
  };
  comm::BinParser bp(setup_inputs_answer, sizeof(setup_inputs_answer));
  rtde_interface::ControlPackageSetupInputs setup_inputs;

  EXPECT_TRUE(setup_inputs.parseWith(bp));

  uint8_t expected_input_recipe_id = 1;
  std::string expected_variable_types = "UINT32,DOUBLE";

  EXPECT_EQ(expected_input_recipe_id, setup_inputs.input_recipe_id_);
  EXPECT_EQ(expected_variable_types, setup_inputs.variable_types_);
}

TEST(rtde_control_package_setup_inputs, parse_not_accepted_setup_inputs)
{
  uint8_t setup_inputs_answer[] = {
    0x00, 0x49, 0x4e, 0x5f, 0x55, 0x53, 0x45, 0x2c, 0x49, 0x4e, 0x5f, 0x55, 0x53, 0x45
  };
  comm::BinParser bp(setup_inputs_answer, sizeof(setup_inputs_answer));
  rtde_interface::ControlPackageSetupInputs setup_inputs;

  EXPECT_TRUE(setup_inputs.parseWith(bp));

  uint8_t expected_input_recipe_id = 0;
  std::string expected_variable_types = "IN_USE,IN_USE";

  EXPECT_EQ(expected_input_recipe_id, setup_inputs.input_recipe_id_);
  EXPECT_EQ(expected_variable_types, setup_inputs.variable_types_);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
