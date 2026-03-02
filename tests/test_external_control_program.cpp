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
#include "ur_client_library/log.h"

using namespace urcl;

const std::string SCRIPT_FILE = "../resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";
std::string g_ROBOT_IP = "192.168.56.101";
bool g_HEADLESS = true;

std::unique_ptr<ExampleRobotWrapper> g_my_robot;

class ExternalControlProgramTest : public ::testing::Test
{
public:
  // callback functions
  void connectionCallback(const socket_t filedescriptor)
  {
    std::lock_guard<std::mutex> lk(connect_mutex_);
    client_fd_ = filedescriptor;
    connect_cv_.notify_one();
    connection_callback_ = true;
  }

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
    server_.reset(new comm::TCPServer(60005));
    server_->setConnectCallback(
        std::bind(&ExternalControlProgramTest::connectionCallback, this, std::placeholders::_1));
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
    prog += "\nsleep(0.6)\n";
    prog += "\ntextmsg(\"sleeping done.\")\n";
    std::ofstream out_file;
    out_file.open(modified_script_path);
    out_file << prog;
    out_file.close();

    return modified_script_path;
  }

  bool waitForConnectionCallback(int milliseconds = 100)
  {
    std::unique_lock<std::mutex> lk(connect_mutex_);
    if (connect_cv_.wait_for(lk, std::chrono::milliseconds(milliseconds),
                             [this]() { return connection_callback_ == true; }))
    {
      connection_callback_ = false;
      return true;
    }
    else
    {
      return false;
    }
  }
  std::unique_ptr<comm::TCPServer> server_;

private:
  std::condition_variable connect_cv_;
  std::mutex connect_mutex_;
  socket_t client_fd_ = INVALID_SOCKET;
  bool connection_callback_ = false;
};

TEST_F(ExternalControlProgramTest, program_halts_on_timeout)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::millisec(200));
  EXPECT_FALSE(waitForConnectionCallback(1000));
}

TEST_F(ExternalControlProgramTest, stop_control_does_not_halt_program)
{
  vector6d_t zeros = { 0, 0, 0, 0, 0, 0 };
  g_my_robot->getUrDriver()->writeJointCommand(zeros, comm::ControlMode::MODE_IDLE, RobotReceiveTimeout::off());

  // Make sure that we can stop the robot control, when robot receive timeout has been set off
  g_my_robot->getUrDriver()->stopControl();
  EXPECT_TRUE(waitForConnectionCallback(1000));
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
