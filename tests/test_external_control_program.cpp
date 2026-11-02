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

#include <gtest/gtest.h>

#include "test_utils.h"
#include "ur_client_library/example_robot_wrapper.h"

using namespace urcl;
const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

class ExternalControlProgramTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!(robotVersionLessThan(g_ROBOT_IP, "10.0.0") || g_HEADLESS))
    {
      GTEST_SKIP_("Running URCap tests for PolyScope X is currently not supported.");
    }
  }
  void SetUp() override
  {
    std::string modified_script_path = extendScript(SCRIPT_FILE);

    g_my_robot = std::make_unique<ExampleRobotWrapper>(g_ROBOT_IP, OUTPUT_RECIPE, INPUT_RECIPE, g_HEADLESS,
                                                       "external_control.urp", modified_script_path);
    if (!g_my_robot->isHealthy())
    {
      ASSERT_TRUE(g_my_robot->resendRobotProgram());
      ASSERT_TRUE(g_my_robot->waitForProgramRunning(500));
    }
    server_.reset(new TestableTcpServer(60005));
    server_->start();
  }

  void TearDown() override
  {
    server_.reset();
  }

  std::string extendScript(const std::string& script_path)
  {
    char modified_script_path[] = "urscript.XXXXXX";
#ifdef _WIN32
#  define mkstemp _mktemp_s
#endif
    std::ignore = mkstemp(modified_script_path);

    std::ofstream ofs(modified_script_path);
    if (ofs.bad())
    {
      std::cout << "Failed to create temporary files" << std::endl;
      throw std::runtime_error("Failed to create temporary files");
    }
    std::ifstream in_file(script_path);
    std::string prog((std::istreambuf_iterator<char>(in_file)), (std::istreambuf_iterator<char>()));
    prog += "\nsocket_open(\"{{SERVER_IP_REPLACE}}\", 60005, \"test_socket\")\n";
    prog += additionalScript();
    prog += "\nsleep(0.6)\n";
    prog += "\ntextmsg(\"sleeping done.\")\n";
    ofs << prog;
    ofs.close();

    return modified_script_path;
  }

  virtual std::string additionalScript() const
  {
    return "";
  }

  std::unique_ptr<TestableTcpServer> server_;
};

class ExternalControlDecodingTest : public ExternalControlProgramTest
{
protected:
  std::string additionalScript() const override
  {
    return R"(
local packet = [22, 110000, -120000, 130000, -140000, 150000, -160000, 210000, -220000, 230000, -240000, 250000, -260000, 310000, -320000, 330000, -340000, 350000, -360000, 190000, 200000, 3, 42]
local original_packet = packet
local decoded_target = extract_pose_data(packet, MULT_jointstate, TRAJ_TARGET_INDEX)
local decoded_via = extract_pose_data(packet, MULT_jointstate, TRAJ_VIA_INDEX)
local decoded_q = extract_array_data(packet, MULT_jointstate, TRAJ_TARGET_INDEX, ZERO_JOINT_VECTOR)
local decoded_qnear = extract_array_data(packet, MULT_jointstate, TRAJ_QNEAR_INDEX, ZERO_JOINT_VECTOR)
local decoded_qd = extract_array_data(packet, MULT_velacc, TRAJ_SPLINE_QD_INDEX, ZERO_JOINT_VECTOR)
local decoded_qdd = extract_array_data(packet, MULT_velacc, TRAJ_SPLINE_QDD_INDEX, ZERO_JOINT_VECTOR)
local short_template = [1.0, 2.0, 3.0]
local decoded_short = extract_array_data(packet, 1, 2, short_template)
local expected_target = [0.11, -0.12, 0.13, -0.14, 0.15, -0.16]
local expected_via = [0.21, -0.22, 0.23, -0.24, 0.25, -0.26]
local expected_qd = [210000, -220000, 230000, -240000, 250000, -260000] / MULT_velacc
local expected_qdd = [310000, -320000, 330000, -340000, 350000, -360000] / MULT_velacc
local reverse_packet = [8, 100, 110000, -120000, 130000, -140000, 150000, -160000, 5]
local decoded_reverse_pose = extract_pose_data(reverse_packet, MULT_jointstate, REVERSE_PAYLOAD_INDEX)
local transformed_pose = pose_trans(p[0, 0, 0, 0, 0, 0], decoded_target)
local valid = JOINT_VECTOR_LENGTH == 6
valid = valid and length(decoded_short) == 3
valid = valid and norm(decoded_short - [-120000, 130000, -140000]) == 0
valid = valid and TRAJ_TIME_INDEX == 19 and TRAJ_BLEND_INDEX == 20
valid = valid and TRAJ_POINT_TYPE_INDEX == 21 and TRAJ_MOVE_ID_INDEX == 22
valid = valid and TRAJECTORY_DATA_DIMENSION == 22
valid = valid and SOCKET_READ_COUNT_INDEX == 0
valid = valid and TRAJ_TARGET_INDEX == 1 and TRAJ_VIA_INDEX == 7 and TRAJ_QNEAR_INDEX == 7
valid = valid and TRAJ_SPLINE_QD_INDEX == 7 and TRAJ_SPLINE_QDD_INDEX == 13
valid = valid and TRAJ_VELOCITY_INDEX == 13 and TRAJ_ACCELERATION_INDEX == 14 and TRAJ_MOTION_PARAMETER_INDEX == 15
valid = valid and TRAJ_SPLINE_TYPE_INDEX == 20
valid = valid and REVERSE_INTERFACE_DATA_DIMENSION == 8 and REVERSE_READ_TIMEOUT_INDEX == 1
valid = valid and REVERSE_PAYLOAD_INDEX == 2 and REVERSE_TRAJECTORY_ACTION_INDEX == 2 and REVERSE_FREEDRIVE_ACTION_INDEX == 2
valid = valid and REVERSE_TRAJECTORY_POINT_COUNT_INDEX == 3 and REVERSE_TRAJECTORY_MOVE_ID_INDEX == 4
valid = valid and REVERSE_CONTROL_MODE_INDEX == 8
valid = valid and SCRIPT_COMMAND_DATA_DIMENSION == 28 and SCRIPT_CMD_ID_INDEX == 1
valid = valid and SCRIPT_CMD_MASS_INDEX == 2 and SCRIPT_CMD_COG_INDEX == 3 and SCRIPT_CMD_INERTIA_INDEX == 6
valid = valid and SCRIPT_CMD_TRANSITION_TIME_INDEX == 12 and SCRIPT_CMD_GRAVITY_INDEX == 2
valid = valid and SCRIPT_CMD_TOOL_VOLTAGE_INDEX == 2 and SCRIPT_CMD_TCP_OFFSET_INDEX == 2
valid = valid and SCRIPT_CMD_FORCE_TASK_FRAME_INDEX == 2 and SCRIPT_CMD_FORCE_SELECTION_VECTOR_INDEX == 8
valid = valid and SCRIPT_CMD_FORCE_WRENCH_INDEX == 14 and SCRIPT_CMD_FORCE_TYPE_INDEX == 20
valid = valid and SCRIPT_CMD_FORCE_LIMITS_INDEX == 21 and SCRIPT_CMD_FORCE_DAMPING_INDEX == 27
valid = valid and SCRIPT_CMD_FORCE_GAIN_SCALING_INDEX == 28
valid = valid and SCRIPT_CMD_FRICTION_COMP_ENABLED_INDEX == 2
valid = valid and SCRIPT_CMD_VISCOUS_SCALE_INDEX == 2 and SCRIPT_CMD_COULOMB_SCALE_INDEX == 8
valid = valid and SCRIPT_CMD_FT_ENABLED_INDEX == 2 and SCRIPT_CMD_FT_SENSOR_MASS_INDEX == 3
valid = valid and SCRIPT_CMD_FT_SENSOR_OFFSET_INDEX == 4 and SCRIPT_CMD_FT_SENSOR_COG_INDEX == 7
valid = valid and packet[TRAJ_TIME_INDEX] == 190000 and packet[TRAJ_BLEND_INDEX] == 200000
valid = valid and packet[TRAJ_POINT_TYPE_INDEX] == 3 and packet[TRAJ_MOVE_ID_INDEX] == 42
valid = valid and packet[TRAJ_VELOCITY_INDEX] == 310000
valid = valid and packet[TRAJ_ACCELERATION_INDEX] == -320000
valid = valid and packet[TRAJ_MOTION_PARAMETER_INDEX] == 330000
local idx = 0
while idx < 6:
  valid = valid and norm(decoded_target[idx] - expected_target[idx]) < 1e-9
  valid = valid and norm(decoded_via[idx] - expected_via[idx]) < 1e-9
  valid = valid and norm(decoded_reverse_pose[idx] - expected_target[idx]) < 1e-9
  valid = valid and norm(transformed_pose[idx] - expected_target[idx]) < 1e-9
  valid = valid and norm(decoded_q[idx] - expected_target[idx]) < 1e-9
  valid = valid and norm(decoded_qnear[idx] - expected_via[idx]) < 1e-9
  valid = valid and norm(decoded_qd[idx] - expected_qd[idx]) < 1e-9
  valid = valid and norm(decoded_qdd[idx] - expected_qdd[idx]) < 1e-9
  idx = idx + 1
end
decoded_target[0] = 99
decoded_q[0] = 99
decoded_short[0] = 99
valid = valid and norm(short_template - [1.0, 2.0, 3.0]) == 0
valid = valid and norm(ZERO_JOINT_VECTOR) == 0
idx = 0
while idx < length(packet):
  valid = valid and packet[idx] == original_packet[idx]
  idx = idx + 1
end
if valid:
  socket_send_string("1", "test_socket")
else:
  socket_send_string("0", "test_socket")
end
)";
  }
};

TEST_F(ExternalControlDecodingTest, decodes_pose_and_joint_socket_fields)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  ASSERT_TRUE(
      g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::off()));
  ASSERT_TRUE(g_my_robot->getUrDriver()->stopControl());
  ASSERT_TRUE(server_->waitForConnectionCallback(1000));
  ASSERT_TRUE(server_->waitForMessageCallback(1000));
  EXPECT_EQ(server_->getReceivedMessage(), "1");
}

TEST_F(ExternalControlProgramTest, program_halts_on_timeout)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::millisec(200));
  EXPECT_FALSE(server_->waitForConnectionCallback(1000));
}

TEST_F(ExternalControlProgramTest, stop_control_does_not_halt_program)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::off());

  // Make sure that we can stop the robot control, when robot receive timeout has been set off
  g_my_robot->getUrDriver()->stopControl();
  EXPECT_TRUE(server_->waitForConnectionCallback(1000));
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--robot_ip" && i + 1 < argc)
    {
      g_ROBOT_IP = argv[i + 1];
      ++i;
    }
    if (std::string(argv[i]) == "--headless" && i + 1 < argc)
    {
      std::string headless = argv[i + 1];
      g_HEADLESS = headless == "true" || headless == "1" || headless == "True" || headless == "TRUE";
      ++i;
    }
  }

  return RUN_ALL_TESTS();
}
