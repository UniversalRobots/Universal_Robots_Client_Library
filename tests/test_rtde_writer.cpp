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
#include <condition_variable>

#include <ur_client_library/rtde/rtde_writer.h>
#include <ur_client_library/comm/tcp_server.h>
#include <ur_client_library/comm/bin_parser.h>

using namespace urcl;

class RTDEWriterTest : public ::testing::Test
{
protected:
  using input_types = std::variant<uint8_t, bool, uint32_t, int32_t, double, vector6d_t>;

  void SetUp()
  {
    // The port shouldn't collide with any of the ports the robot is using
    server_.reset(new comm::TCPServer(60004));
    server_->setMessageCallback(std::bind(&RTDEWriterTest::messageCallback, this, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3));
    server_->start();

    stream_.reset(new comm::URStream<rtde_interface::RTDEPackage>("127.0.0.1", 60004));
    stream_->connect();

    writer_.reset(new rtde_interface::RTDEWriter(stream_.get(), input_recipe_));
    writer_->init(1);
  }

  void TearDown()
  {
    // Clean up
    writer_.reset();
    stream_.reset();
    server_.reset();
  }

  void messageCallback(const socket_t filedescriptor, char* buffer, int nbytesrecv)
  {
    std::lock_guard<std::mutex> lk(message_mutex_);
    uint8_t* buf = reinterpret_cast<uint8_t*>(buffer);
    comm::BinParser bp(buf, nbytesrecv);
    // These might be needed in the test
    uint16_t size;
    uint8_t type, recipe_id;
    bp.parse(size);
    bp.parse(type);
    bp.parse(recipe_id);
    parseMessage(bp);
    message_cv_.notify_one();
    message_callback_ = true;
  }

  bool waitForMessageCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(message_mutex_);
    if (message_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds)) == std::cv_status::no_timeout ||
        message_callback_ == true)
    {
      message_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }

  // Helper function to see if data field exists in the parsed message
  bool dataFieldExist(std::string name)
  {
    if (parsed_data_.find(name) != parsed_data_.end())
    {
      return true;
    }
    std::cout << "Failed to find data field " << name << " this should not happen! Have a look at the test case"
              << std::endl;
    return false;
  }

  std::vector<std::string> input_recipe_ = { "speed_slider_mask",
                                             "speed_slider_fraction",
                                             "standard_digital_output_mask",
                                             "standard_digital_output",
                                             "configurable_digital_output_mask",
                                             "configurable_digital_output",
                                             "tool_digital_output_mask",
                                             "tool_digital_output",
                                             "standard_analog_output_mask",
                                             "standard_analog_output_type",
                                             "standard_analog_output_0",
                                             "standard_analog_output_1",
                                             "input_bit_register_65",
                                             "input_int_register_25",
                                             "input_double_register_25",
                                             "external_force_torque" };
  std::unique_ptr<rtde_interface::RTDEWriter> writer_;
  std::unique_ptr<comm::TCPServer> server_;
  std::unique_ptr<comm::URStream<rtde_interface::RTDEPackage>> stream_;
  std::unordered_map<std::string, input_types> parsed_data_;

private:
  void parseMessage(comm::BinParser bp)
  {
    for (auto& item : input_recipe_)
    {
      if (input_map_types_.find(item) != input_map_types_.end())
      {
        input_types entry = input_map_types_[item];
        std::visit([&bp](auto&& arg) { bp.parse(arg); }, entry);
        parsed_data_[item] = entry;
      }
    }
  }

  std::condition_variable message_cv_;
  std::mutex message_mutex_;
  bool message_callback_ = false;

  std::unordered_map<std::string, input_types> input_map_types_ = {
    { "speed_slider_mask", uint32_t() },
    { "speed_slider_fraction", double() },
    { "standard_digital_output_mask", uint8_t() },
    { "standard_digital_output", uint8_t() },
    { "configurable_digital_output_mask", uint8_t() },
    { "configurable_digital_output", uint8_t() },
    { "tool_digital_output_mask", uint8_t() },
    { "tool_digital_output", uint8_t() },
    { "standard_analog_output_mask", uint8_t() },
    { "standard_analog_output_type", uint8_t() },
    { "standard_analog_output_0", double() },
    { "standard_analog_output_1", double() },
    { "input_bit_register_65", bool() },
    { "input_int_register_25", int32_t() },
    { "input_double_register_25", double() },
    { "external_force_torque", vector6d_t() },
  };
};

// Use other port and create test fixture
TEST_F(RTDEWriterTest, send_speed_slider)
{
  uint32_t expected_speed_slider_mask = 1;
  double send_speed_slider_fraction = 0.5;

  EXPECT_TRUE(writer_->sendSpeedSlider(send_speed_slider_fraction));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("speed_slider_fraction"));
  ASSERT_TRUE(dataFieldExist("speed_slider_mask"));

  double received_speed_slider_fraction = std::get<double>(parsed_data_["speed_slider_fraction"]);
  uint32_t received_speed_slider_mask = std::get<uint32_t>(parsed_data_["speed_slider_mask"]);

  EXPECT_EQ(send_speed_slider_fraction, received_speed_slider_fraction);
  EXPECT_EQ(expected_speed_slider_mask, received_speed_slider_mask);

  // Setting speed slider fraction below 0 or above 1, should return false
  EXPECT_FALSE(writer_->sendSpeedSlider(-1));
  EXPECT_FALSE(writer_->sendSpeedSlider(2));
}

TEST_F(RTDEWriterTest, send_standard_digital_output)
{
  uint8_t expected_standard_digital_output_mask = 4;
  uint8_t pin = 2;
  bool send_pin_value = true;
  EXPECT_TRUE(writer_->sendStandardDigitalOutput(pin, send_pin_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("standard_digital_output"));
  ASSERT_TRUE(dataFieldExist("standard_digital_output_mask"));

  bool received_pin_value = std::get<uint8_t>(parsed_data_["standard_digital_output"]) != 0;
  uint8_t received_standard_digital_output_mask = std::get<uint8_t>(parsed_data_["standard_digital_output_mask"]);

  EXPECT_EQ(send_pin_value, received_pin_value);
  EXPECT_EQ(expected_standard_digital_output_mask, received_standard_digital_output_mask);

  // Changing pins above 7, should return false.
  pin = 8;
  EXPECT_FALSE(writer_->sendStandardDigitalOutput(pin, send_pin_value));
}

TEST_F(RTDEWriterTest, send_configurable_digital_output)
{
  uint8_t expected_configurable_digital_output_mask = 8;
  uint8_t pin = 3;
  bool send_pin_value = true;
  EXPECT_TRUE(writer_->sendConfigurableDigitalOutput(pin, send_pin_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("configurable_digital_output"));
  ASSERT_TRUE(dataFieldExist("configurable_digital_output_mask"));

  bool received_pin_value = std::get<uint8_t>(parsed_data_["configurable_digital_output"]) != 0;
  uint8_t received_standard_digital_output_mask = std::get<uint8_t>(parsed_data_["configurable_digital_output_mask"]);

  EXPECT_EQ(send_pin_value, received_pin_value);
  EXPECT_EQ(expected_configurable_digital_output_mask, received_standard_digital_output_mask);

  // Changing pins above 7, should return false.
  pin = 8;
  EXPECT_FALSE(writer_->sendStandardDigitalOutput(pin, send_pin_value));
}

TEST_F(RTDEWriterTest, send_tool_digital_output)
{
  uint8_t expected_tool_digital_output_mask = 1;
  uint8_t pin = 0;
  bool send_pin_value = true;
  EXPECT_TRUE(writer_->sendToolDigitalOutput(pin, send_pin_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("tool_digital_output"));
  ASSERT_TRUE(dataFieldExist("tool_digital_output_mask"));

  bool received_pin_value = std::get<uint8_t>(parsed_data_["tool_digital_output"]) != 0;
  uint8_t received_tool_digital_output_mask = std::get<uint8_t>(parsed_data_["tool_digital_output_mask"]);

  EXPECT_EQ(send_pin_value, received_pin_value);
  EXPECT_EQ(expected_tool_digital_output_mask, received_tool_digital_output_mask);

  // Changing pins above 1, should return false.
  pin = 2;
  EXPECT_FALSE(writer_->sendToolDigitalOutput(pin, send_pin_value));
}

TEST_F(RTDEWriterTest, send_standard_analog_output_unknown_domain)
{
  waitForMessageCallback(1000);

  uint8_t expected_standard_analog_output_mask = 1;

  uint8_t pin = 0;
  double send_analog_output = 1;
  EXPECT_TRUE(writer_->sendStandardAnalogOutput(pin, send_analog_output));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("standard_analog_output_0"));
  ASSERT_TRUE(dataFieldExist("standard_analog_output_1"));
  ASSERT_TRUE(dataFieldExist("standard_analog_output_mask"));
  ASSERT_TRUE(dataFieldExist("standard_analog_output_type"));

  double received_analog_output = std::get<double>(parsed_data_["standard_analog_output_0"]);
  uint8_t received_standard_analog_output_mask = std::get<uint8_t>(parsed_data_["standard_analog_output_mask"]);
  uint8_t received_standard_analog_output_type = std::get<uint8_t>(parsed_data_["standard_analog_output_type"]);

  EXPECT_EQ(send_analog_output, received_analog_output);
  EXPECT_EQ(expected_standard_analog_output_mask, received_standard_analog_output_mask);
  // The test server sets this to 0 if not given
  EXPECT_EQ(0, received_standard_analog_output_type);
}

TEST_F(RTDEWriterTest, send_standard_analog_output_voltage)
{
  uint8_t pin = 0;
  AnalogOutputType type = AnalogOutputType::VOLTAGE;
  double send_analog_output = 1;

  uint8_t expected_standard_analog_output_mask = 1;
  uint8_t expected_standard_analog_output_type = 1;

  EXPECT_TRUE(writer_->sendStandardAnalogOutput(pin, send_analog_output, type));

  waitForMessageCallback(1000);

  double received_analog_output = std::get<double>(parsed_data_["standard_analog_output_0"]);
  uint8_t received_standard_analog_output_mask = std::get<uint8_t>(parsed_data_["standard_analog_output_mask"]);
  uint8_t received_standard_analog_output_type = std::get<uint8_t>(parsed_data_["standard_analog_output_type"]);

  EXPECT_EQ(send_analog_output, received_analog_output);
  EXPECT_EQ(expected_standard_analog_output_mask, received_standard_analog_output_mask);
  EXPECT_EQ(expected_standard_analog_output_type, received_standard_analog_output_type);

  pin = 1;
  expected_standard_analog_output_mask = 2;
  expected_standard_analog_output_type = 2;

  EXPECT_TRUE(writer_->sendStandardAnalogOutput(pin, send_analog_output, type));

  waitForMessageCallback(1000);

  received_analog_output = std::get<double>(parsed_data_["standard_analog_output_1"]);
  received_standard_analog_output_mask = std::get<uint8_t>(parsed_data_["standard_analog_output_mask"]);
  received_standard_analog_output_type = std::get<uint8_t>(parsed_data_["standard_analog_output_type"]);

  EXPECT_EQ(send_analog_output, received_analog_output);
  EXPECT_EQ(expected_standard_analog_output_mask, received_standard_analog_output_mask);
  EXPECT_EQ(expected_standard_analog_output_type, received_standard_analog_output_type);
}

TEST_F(RTDEWriterTest, send_standard_analog_output_current)
{
  uint8_t pin = 0;
  AnalogOutputType type = AnalogOutputType::CURRENT;
  double send_analog_output = 1;

  uint8_t expected_standard_analog_output_mask = 1;
  uint8_t expected_standard_analog_output_type = 0;

  EXPECT_TRUE(writer_->sendStandardAnalogOutput(pin, send_analog_output, type));

  waitForMessageCallback(1000);

  double received_analog_output = std::get<double>(parsed_data_["standard_analog_output_0"]);
  uint8_t received_standard_analog_output_mask = std::get<uint8_t>(parsed_data_["standard_analog_output_mask"]);
  uint8_t received_standard_analog_output_type = std::get<uint8_t>(parsed_data_["standard_analog_output_type"]);

  EXPECT_EQ(send_analog_output, received_analog_output);
  EXPECT_EQ(expected_standard_analog_output_mask, received_standard_analog_output_mask);
  EXPECT_EQ(expected_standard_analog_output_type, received_standard_analog_output_type);

  pin = 1;
  expected_standard_analog_output_mask = 2;
  expected_standard_analog_output_type = 0;

  EXPECT_TRUE(writer_->sendStandardAnalogOutput(pin, send_analog_output, type));

  waitForMessageCallback(1000);

  received_analog_output = std::get<double>(parsed_data_["standard_analog_output_1"]);
  received_standard_analog_output_mask = std::get<uint8_t>(parsed_data_["standard_analog_output_mask"]);
  received_standard_analog_output_type = std::get<uint8_t>(parsed_data_["standard_analog_output_type"]);

  EXPECT_EQ(send_analog_output, received_analog_output);
  EXPECT_EQ(expected_standard_analog_output_mask, received_standard_analog_output_mask);
  EXPECT_EQ(expected_standard_analog_output_type, received_standard_analog_output_type);
}

TEST_F(RTDEWriterTest, send_standard_analog_output_illegal_input_fails)
{
  uint8_t pin = 0;
  double send_analog_output = 1;

  // Pin should be either 0 or 1
  pin = 2;
  EXPECT_FALSE(writer_->sendStandardAnalogOutput(pin, send_analog_output));

  // Setting analog output below 0 or above 1, should return false
  pin = 1;
  EXPECT_FALSE(writer_->sendStandardAnalogOutput(pin, 1.1));
  EXPECT_FALSE(writer_->sendStandardAnalogOutput(pin, -0.1));
}

TEST_F(RTDEWriterTest, send_input_bit_register)
{
  uint32_t register_id = 65;
  bool send_register_value = true;
  EXPECT_TRUE(writer_->sendInputBitRegister(register_id, send_register_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("input_bit_register_65"));

  bool received_register_value = std::get<bool>(parsed_data_["input_bit_register_65"]);

  EXPECT_EQ(send_register_value, received_register_value);

  // Changing registers below 64 and above 127, should return false.
  register_id = 63;
  EXPECT_FALSE(writer_->sendInputBitRegister(register_id, send_register_value));
  register_id = 128;
  EXPECT_FALSE(writer_->sendInputBitRegister(register_id, send_register_value));
}

TEST_F(RTDEWriterTest, send_input_int_register)
{
  uint32_t register_id = 25;
  int32_t send_register_value = 21;
  EXPECT_TRUE(writer_->sendInputIntRegister(register_id, send_register_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("input_int_register_25"));

  int32_t received_register_value = std::get<int32_t>(parsed_data_["input_int_register_25"]);

  EXPECT_EQ(send_register_value, received_register_value);

  // Changing registers below 23 and above 48, should return false.
  register_id = 23;
  EXPECT_FALSE(writer_->sendInputIntRegister(register_id, send_register_value));
  register_id = 48;
  EXPECT_FALSE(writer_->sendInputIntRegister(register_id, send_register_value));
}

TEST_F(RTDEWriterTest, send_input_double_register)
{
  uint32_t register_id = 25;
  double send_register_value = 2.1;
  EXPECT_TRUE(writer_->sendInputDoubleRegister(register_id, send_register_value));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("input_double_register_25"));

  double received_register_value = std::get<double>(parsed_data_["input_double_register_25"]);

  EXPECT_EQ(send_register_value, received_register_value);

  // Changing registers below 23 and above 48, should return false.
  register_id = 23;
  EXPECT_FALSE(writer_->sendInputDoubleRegister(register_id, send_register_value));
  register_id = 48;
  EXPECT_FALSE(writer_->sendInputDoubleRegister(register_id, send_register_value));
}

TEST_F(RTDEWriterTest, send_external_force_torque)
{
  vector6d_t send_external_force_torque = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
  EXPECT_TRUE(writer_->sendExternalForceTorque(send_external_force_torque));

  waitForMessageCallback(1000);

  ASSERT_TRUE(dataFieldExist("external_force_torque"));

  vector6d_t received_external_force_torque = std::get<vector6d_t>(parsed_data_["external_force_torque"]);

  EXPECT_EQ(send_external_force_torque[0], received_external_force_torque[0]);
  EXPECT_EQ(send_external_force_torque[1], received_external_force_torque[1]);
  EXPECT_EQ(send_external_force_torque[2], received_external_force_torque[2]);
  EXPECT_EQ(send_external_force_torque[3], received_external_force_torque[3]);
  EXPECT_EQ(send_external_force_torque[4], received_external_force_torque[4]);
  EXPECT_EQ(send_external_force_torque[5], received_external_force_torque[5]);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
