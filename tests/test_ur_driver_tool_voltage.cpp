// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
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

// Drives UrDriver's tool voltage handling against the fake primary and RTDE servers, so no robot is
// needed. In headless mode the startup program and the fallback script code both go out over the
// primary interface, where the fake server captures them.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>

#include <ur_client_library/comm/tcp_server.h>
#include <ur_client_library/comm/tcp_socket.h>
#include <ur_client_library/control/reverse_interface.h>
#include <ur_client_library/exceptions.h>
#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/ur/ur_driver.h>
#include <urcl_3rdparty/portable_endian.h>

#include "fake_primary_server.h"
#include "fake_rtde_server.h"

using namespace urcl;

namespace
{
const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::vector<std::string> OUTPUT_RECIPE{ "timestamp", "actual_q", "target_speed_fraction", "runtime_state" };
const std::vector<std::string> INPUT_RECIPE{ "speed_slider_mask", "speed_slider_fraction" };

// ConfigurationData payload without the optional control box and tool flange fields. Its content does not matter,
// UrDriver only waits for one to arrive.
constexpr size_t CONFIGURATION_DATA_PAYLOAD_BYTES = 440;

constexpr std::chrono::seconds SCRIPT_TIMEOUT{ 5 };
}  // namespace

class UrDriverToolVoltageTest : public ::testing::Test
{
protected:
  // UrDriver always connects to the default ports, so another robot or simulator listening on them (such as a
  // URSim container publishing its ports) makes these tests impossible rather than failed.
  void SetUp() override
  {
    for (const int port : { static_cast<int>(UR_RTDE_PORT), primary_interface::UR_PRIMARY_PORT })
    {
      try
      {
        comm::TCPServer probe(port, 1, std::chrono::milliseconds(10));
      }
      catch (const std::system_error&)
      {
        GTEST_SKIP() << "Port " << port << " is in use, so the fake robot cannot be started.";
      }
    }
  }

  void TearDown() override
  {
    driver_.reset();
    stopSendingConfigurationData();
    primary_server_.reset();
    rtde_server_.reset();
  }

  void startServers(const ToolFlangeType tool_flange_type)
  {
    rtde_server_ = std::make_unique<RTDEServer>(UR_RTDE_PORT);
    rtde_server_->setStartTime(std::chrono::steady_clock::now() - std::chrono::seconds(42));
    rtde_server_->setHighestAcceptedProtocolVersion(3);
    rtde_server_->setReportedToolFlangeType(tool_flange_type);
    rtde_server_->setReportedUrControlVersion(10, 15, 0, 0);

    primary_server_ = std::make_unique<FakePrimaryServer>(primary_interface::UR_PRIMARY_PORT);
    primary_server_->setScriptCallback([this](const std::string& data) {
      std::lock_guard<std::mutex> lock(received_mutex_);
      received_ += data;
    });

    // UrDriver's constructor waits for configuration data, so keep sending it until the driver is up.
    sending_configuration_data_ = true;
    configuration_data_thread_ = std::thread([this]() {
      const std::vector<uint8_t> payload(CONFIGURATION_DATA_PAYLOAD_BYTES, 0);
      while (sending_configuration_data_)
      {
        primary_server_->sendRobotState({ { primary_interface::RobotStateType::CONFIGURATION_DATA, payload } });
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    });
  }

  void stopSendingConfigurationData()
  {
    sending_configuration_data_ = false;
    if (configuration_data_thread_.joinable())
    {
      configuration_data_thread_.join();
    }
  }

  UrDriverConfiguration makeConfig(std::unique_ptr<ToolCommSetup> tool_comm_setup = nullptr)
  {
    UrDriverConfiguration config;
    config.robot_ip = "127.0.0.1";
    config.script_file = SCRIPT_FILE;
    config.output_recipe = OUTPUT_RECIPE;
    config.input_recipe = INPUT_RECIPE;
    config.headless_mode = true;
    config.tool_comm_setup = std::move(tool_comm_setup);
    config.socket_reconnect_attempts = 1;
    config.socket_reconnection_timeout = std::chrono::milliseconds(100);
    return config;
  }

  void startDriver(std::unique_ptr<ToolCommSetup> tool_comm_setup = nullptr)
  {
    driver_ = std::make_unique<UrDriver>(makeConfig(std::move(tool_comm_setup)));
    stopSendingConfigurationData();
  }

  bool waitForReceived(const std::string& expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + SCRIPT_TIMEOUT;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (received().find(expected) != std::string::npos)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received().find(expected) != std::string::npos;
  }

  std::string received()
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    return received_;
  }

  void clearReceived()
  {
    std::lock_guard<std::mutex> lock(received_mutex_);
    received_.clear();
  }

  std::unique_ptr<RTDEServer> rtde_server_;
  std::unique_ptr<FakePrimaryServer> primary_server_;
  std::unique_ptr<UrDriver> driver_;

  std::atomic<bool> sending_configuration_data_{ false };
  std::thread configuration_data_thread_;

  std::mutex received_mutex_;
  std::string received_;
};

TEST_F(UrDriverToolVoltageTest, startup_program_sets_t2_voltage_on_flange_v2)
{
  startServers(ToolFlangeType::V2);
  auto tool_comm_setup = std::make_unique<ToolCommSetup>();
  tool_comm_setup->setToolVoltage(ToolVoltage::_24V);
  tool_comm_setup->setToolVoltageT2(ToolVoltage::_48V);

  startDriver(std::move(tool_comm_setup));

  EXPECT_TRUE(waitForReceived("set_tool_voltage(24)"));
  EXPECT_TRUE(waitForReceived("set_power_output(\"T2_V\", 48)"));
}

TEST_F(UrDriverToolVoltageTest, startup_program_leaves_t2_alone_when_not_set)
{
  startServers(ToolFlangeType::V2);
  auto tool_comm_setup = std::make_unique<ToolCommSetup>();
  tool_comm_setup->setToolVoltage(ToolVoltage::_12V);

  startDriver(std::move(tool_comm_setup));

  ASSERT_TRUE(waitForReceived("set_tool_voltage(12)"));
  // The program's script command handler sets T2 from a variable, so only look for literal voltages.
  for (const char* voltage : { "0", "24", "48" })
  {
    EXPECT_EQ(received().find(std::string("set_power_output(\"T2_V\", ") + voltage + ")"), std::string::npos);
  }
}

TEST_F(UrDriverToolVoltageTest, startup_throws_for_t2_voltage_without_flange_v2)
{
  startServers(ToolFlangeType::V1);
  auto tool_comm_setup = std::make_unique<ToolCommSetup>();
  tool_comm_setup->setToolVoltageT2(ToolVoltage::_24V);

  EXPECT_THROW(startDriver(std::move(tool_comm_setup)), UrException);
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_t2_falls_back_to_script_on_flange_v2)
{
  startServers(ToolFlangeType::V2);
  startDriver();
  clearReceived();

  EXPECT_TRUE(driver_->setToolVoltageT2(ToolVoltage::_24V));
  EXPECT_TRUE(waitForReceived("set_power_output(\"T2_V\", 24)"));
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_t2_uses_script_command_interface_when_connected)
{
  startServers(ToolFlangeType::V2);
  const UrDriverConfiguration config = makeConfig();
  startDriver();
  clearReceived();

  // Stands in for the external control program connecting to the script command interface.
  comm::TCPSocket script_command_client;
  ASSERT_TRUE(script_command_client.connect("127.0.0.1", static_cast<int>(config.script_command_port), 1));
  timeval timeout{ 1, 0 };
  script_command_client.setReceiveTimeout(timeout);
  // The connection is accepted on the server's own thread.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ASSERT_TRUE(driver_->setToolVoltageT2(ToolVoltage::_48V));

  int32_t command_and_voltage[2];
  size_t read = 0;
  uint8_t* buffer = reinterpret_cast<uint8_t*>(command_and_voltage);
  size_t total = 0;
  while (total < sizeof(command_and_voltage) &&
         script_command_client.read(buffer + total, sizeof(command_and_voltage) - total, read))
  {
    total += read;
  }
  ASSERT_EQ(total, sizeof(command_and_voltage));
  EXPECT_EQ(be32toh(command_and_voltage[0]), 13);  // SET_TOOL_T2_VOLTAGE
  EXPECT_EQ(be32toh(command_and_voltage[1]) / control::ReverseInterface::MULT_JOINTSTATE, 48);
  EXPECT_EQ(received().find("T2_V"), std::string::npos);
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_t2_rejected_without_flange_v2)
{
  startServers(ToolFlangeType::V1);
  startDriver();
  clearReceived();

  EXPECT_FALSE(driver_->setToolVoltageT2(ToolVoltage::_24V));
  EXPECT_EQ(received().find("T2_V"), std::string::npos);
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_t2_rejects_invalid_voltage)
{
  startServers(ToolFlangeType::V2);
  startDriver();

  EXPECT_FALSE(driver_->setToolVoltageT2(ToolVoltage::_12V));
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_falls_back_to_script)
{
  startServers(ToolFlangeType::V1);
  startDriver();
  clearReceived();

  EXPECT_TRUE(driver_->setToolVoltage(ToolVoltage::_12V));
  EXPECT_TRUE(waitForReceived("set_tool_voltage(12)"));
}

TEST_F(UrDriverToolVoltageTest, set_tool_voltage_rejects_48v)
{
  startServers(ToolFlangeType::V2);
  startDriver();

  EXPECT_FALSE(driver_->setToolVoltage(ToolVoltage::_48V));
}
